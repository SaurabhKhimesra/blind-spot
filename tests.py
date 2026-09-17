"""Regression tests. Run these before and after any change.

These lock down the results that took a day to establish. If a test fails,
the change is wrong, not the test - check the sign conventions in
partitioned.py first, because that is where two bugs already hid.

    python3 tests.py
"""

import numpy as np

from ibvs_core import (se3_exp, rot_z, rot_x, make_pose, square_target,
                       transform_points, project, interaction_matrix,
                       run_ibvs, scenario_camera_retreat, sigma_6)
from partitioned import run_partitioned, polygon_sigma, line_alpha
from truncated import run_truncated, pinv_truncated
from switched import (run_switched, use_partition, calibrate_sigma6,
                      calibrate_area, calibrate_alpha, calibrate_sigma6_table)
from partitioned import polygon_sigma as _poly
from partitioned import XY_COLS

PASS, FAIL = "  ok  ", " FAIL "
_results = []


def check(name, cond, detail=""):
    _results.append(bool(cond))
    print("[%s] %s %s" % (PASS if cond else FAIL, name, detail))


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def spike(lg, a=30, b=90):
    """Peak commanded velocity inside the degradation window, relative to the
    controller's own level immediately before it."""
    return (np.linalg.norm(lg["v"][a:b + 1], axis=1).max()
            / np.linalg.norm(lg["v"][a - 1]))


# ----------------------------------------------------------------------
# 1. The maths. If this fails, nothing downstream means anything.
# ----------------------------------------------------------------------
print("\n--- foundations ---")

T = se3_exp(np.array([0.1, 0, 0, 0, 0, 0.0]))
check("se3_exp pure translation",
      np.allclose(T[:3, 3], [0.1, 0, 0]) and np.allclose(T[:3, :3], np.eye(3)))

T = se3_exp(np.array([0, 0, 0, 0, 0, np.pi / 2]))
check("se3_exp pure rotation", np.allclose(T[:3, :3], rot_z(np.pi / 2)))

# The interaction matrix must match a numerical derivative of the projection.
P_o = square_target()
cTo = make_pose(R=rot_x(0.2), t=np.array([0.02, 0.01, 0.8]))
P_c = transform_points(cTo, P_o)
s = project(P_c)
L = interaction_matrix(s, P_c[:, 2])
v = np.array([0.05, -0.03, 0.04, 0.1, -0.05, 0.08])
h = 1e-5
s2 = project(transform_points(se3_exp(-v * h) @ cTo, P_o))
num = (s2 - s).reshape(-1) / h
rel = np.linalg.norm(num - L @ v) / np.linalg.norm(L @ v)
check("interaction matrix vs numerical derivative", rel < 1e-5,
      "(rel err %.1e)" % rel)

# Sign conventions that were wrong once and must stay right.
P6 = ring()
cTo = make_pose(R=rot_z(0.3) @ rot_x(0.1), t=np.array([0.03, -0.02, 0.7]))
s0 = project(transform_points(cTo, P6))
vv = np.zeros(6); vv[5] = 1.0
s1 = project(transform_points(se3_exp(-vv * h) @ cTo, P6))
d_alpha_d_wz = (line_alpha(s1) - line_alpha(s0)) / h
check("d(alpha)/d(wz) == -1", abs(d_alpha_d_wz + 1.0) < 1e-3,
      "(got %.4f)" % d_alpha_d_wz)

vv = np.zeros(6); vv[2] = 1.0
s1 = project(transform_points(se3_exp(-vv * h) @ cTo, P6))
d_lnsig_d_vz = (np.log(polygon_sigma(s1)) - np.log(polygon_sigma(s0))) / h
check("d(ln sigma)/d(vz) > 0", d_lnsig_d_vz > 0.5,
      "(got %+.4f)" % d_lnsig_d_vz)

# ----------------------------------------------------------------------
# 2. Camera retreat. Classic fails, partitioned fixes it, truncation does not.
# ----------------------------------------------------------------------
print("\n--- camera retreat (a coupling failure) ---")

P_o, ci, cs = scenario_camera_retreat(angle_deg=180.0)
cl = run_ibvs(P_o, ci, cs, lam=0.5, steps=4000)
pt = run_partitioned(P_o, ci, cs, lam=0.5, steps=4000)
tr = run_truncated(P_o, ci, cs, rel_tau=1e-3, lam=0.5, steps=4000)
cb = run_partitioned(P_o, ci, cs, lam=0.5, steps=4000, rel_tau=1e-3)

check("classic retreats past 50 m", np.abs(cl["cam_pos"][:, 2]).max() > 50,
      "(%.1f m)" % np.abs(cl["cam_pos"][:, 2]).max())
check("classic fails to converge", cl["err"][-1] > 0.1,
      "(err %.3f)" % cl["err"][-1])
check("partitioned does not retreat", np.abs(pt["cam_pos"][:, 2]).max() < 0.85,
      "(%.2f m)" % np.abs(pt["cam_pos"][:, 2]).max())
check("partitioned converges", pt["err"][-1] < 1e-4,
      "(err %.6f)" % pt["err"][-1])
check("truncation alone does NOT fix retreat",
      np.abs(tr["cam_pos"][:, 2]).max() > 5.0,
      "(%.1f m - this is expected, not a bug)"
      % np.abs(tr["cam_pos"][:, 2]).max())
check("combined does not retreat", np.abs(cb["cam_pos"][:, 2]).max() < 0.85,
      "(%.2f m)" % np.abs(cb["cam_pos"][:, 2]).max())
check("combined converges", cb["err"][-1] < 1e-4, "(err %.6f)" % cb["err"][-1])

# ----------------------------------------------------------------------
# 3. Feature dropout. Classic lurches, the others do not.
# ----------------------------------------------------------------------
print("\n--- feature dropout (a conditioning failure) ---")

P_o = ring()
cs = make_pose(t=np.array([0.0, 0.0, 0.6]))
ci = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))


def occ3(k, s):
    vis = np.ones(6, dtype=bool)
    if 30 <= k <= 90:
        vis[:3] = False
    return vis


cl = run_ibvs(P_o, ci, cs, lam=0.5, steps=2000, visible_mask_fn=occ3)
pt = run_partitioned(P_o, ci, cs, lam=0.5, steps=2000, visible_mask_fn=occ3)
tr = run_truncated(P_o, ci, cs, rel_tau=1e-3, lam=0.5, steps=2000,
                   visible_mask_fn=occ3)
cb = run_partitioned(P_o, ci, cs, lam=0.5, steps=2000, visible_mask_fn=occ3,
                     rel_tau=1e-3)

check("classic lurches more than 20x", spike(cl) > 20, "(%.1fx)" % spike(cl))
check("partitioned barely lurches", spike(pt) < 1.5, "(%.1fx)" % spike(pt))
check("truncation barely lurches", spike(tr) < 1.5, "(%.1fx)" % spike(tr))
check("combined barely lurches", spike(cb) < 1.5, "(%.1fx)" % spike(cb))
check("image error is blind to the lurch",
      abs(cl["cos_align"].min() + 1.0) < 1e-6 if "cos_align" in cl else True,
      "(alignment stays at -1.0 throughout)")

# ----------------------------------------------------------------------
# 4. Truncation is free when nothing is degraded.
# ----------------------------------------------------------------------
print("\n--- cost of truncation on a clean run ---")

cl = run_ibvs(P_o, ci, cs, lam=0.5, steps=2000)
tr = run_truncated(P_o, ci, cs, rel_tau=1e-3, lam=0.5, steps=2000)
check("clean: truncation keeps full rank", tr["rank"].min() == 6)
check("clean: truncation converges as well as classic",
      abs(tr["err"][-1] - cl["err"][-1]) < 1e-6)

Linv, rank = pinv_truncated(np.diag([1.0, 0.5, 1e-9]), 1e-3)
check("pinv_truncated drops tiny singular directions", rank == 2)

# ----------------------------------------------------------------------
# 5. Two features left. The case the fixed partition fails catastrophically.
#    Only features 0 and 3 of the ring survive, so L has 4 rows: classic is
#    underdetermined and protected by the minimum-norm solution, while the
#    partition's reduced 4x4 solve is exactly determined and brittle.
#    min_features=2 is required to let a genuine 2-feature step happen.
# ----------------------------------------------------------------------
print("\n--- two features left (the case no fixed rule wins) ---")


def occ_two(k, s):
    if 30 <= k <= 90:
        vis = np.zeros(6, dtype=bool)
        vis[[0, 3]] = True
        return vis
    return np.ones(6, dtype=bool)


kw2 = dict(lam=0.5, steps=2000, visible_mask_fn=occ_two, min_features=2)
cl2 = run_ibvs(P_o, ci, cs, **kw2)
pt2 = run_partitioned(P_o, ci, cs, **kw2)
tr2 = run_truncated(P_o, ci, cs, rel_tau=1e-3, **kw2)
cb2 = run_partitioned(P_o, ci, cs, rel_tau=1e-3, **kw2)

check("two features actually reach the control law", cl2["n_vis"].min() == 2,
      "(min visible %d)" % cl2["n_vis"].min())
check("classic is protected by minimum-norm", spike(cl2) < 1.5,
      "(%.1fx)" % spike(cl2))
check("truncation alone is fine on two features", spike(tr2) < 1.5,
      "(%.1fx)" % spike(tr2))
check("fixed partition fails catastrophically", spike(pt2) > 100,
      "(%.1fx)" % spike(pt2))
check("combined reduces it but does not fix it", 5 < spike(cb2) < 30,
      "(%.1fx)" % spike(cb2))

# ----------------------------------------------------------------------
# 6. The switched controller: the partition as a runtime decision.
# ----------------------------------------------------------------------
print("\n--- switched controller (Task 1) ---")

for rule in ["count", "rank_margin"]:
    sw_r = run_switched(P_o, ci, cs, rel_tau=1e-3, rule=rule, **kw2)
    check("switched/%s absorbs two features under 2x" % rule,
          spike(sw_r) < 2.0, "(%.1fx)" % spike(sw_r))

    sw_d = run_switched(P_o, ci, cs, rel_tau=1e-3, rule=rule, lam=0.5,
                        steps=2000, visible_mask_fn=occ3)
    check("switched/%s still absorbs the dropout" % rule,
          spike(sw_d) < 1.5, "(%.1fx)" % spike(sw_d))

    Pr_, cir_, csr_ = scenario_camera_retreat(angle_deg=180.0)
    sw_r2 = run_switched(Pr_, cir_, csr_, lam=0.5, steps=4000, rel_tau=1e-3,
                         rule=rule)
    check("switched/%s does not retreat" % rule,
          np.abs(sw_r2["cam_pos"][:, 2]).max() < 0.85,
          "(%.2f m)" % np.abs(sw_r2["cam_pos"][:, 2]).max())
    check("switched/%s converges on retreat" % rule,
          sw_r2["err"][-1] < 1e-4, "(err %.6f)" % sw_r2["err"][-1])

# The switch must be inert when nothing is degraded, so a clean run is
# bit-identical to the fixed combined controller.
sw_cl = run_switched(P_o, ci, cs, lam=0.5, steps=2000, rel_tau=1e-3)
cb_cl = run_partitioned(P_o, ci, cs, lam=0.5, steps=2000, rel_tau=1e-3)
check("clean: switch never drops the partition",
      bool(sw_cl["partition"].all()))
check("clean: switched is identical to combined",
      np.allclose(sw_cl["v"], cb_cl["v"], atol=1e-12))

# It must fire exactly on the degraded window, and not chatter.
sw2 = run_switched(P_o, ci, cs, rel_tau=1e-3, **kw2)
off = np.where(~sw2["partition"])[0]
check("switch fires exactly on the two-feature window",
      off.min() == 30 and off.max() == 90 and off.size == 61,
      "(steps %d..%d, %d of them)" % (off.min(), off.max(), off.size))
check("switch does not chatter",
      int(np.sum(sw2["partition"][1:] != sw2["partition"][:-1])) == 2,
      "(2 flips)")

# The decision itself, on real interaction matrices.
Pc_t = transform_points(make_pose(R=rot_z(0.37) @ rot_x(0.21),
                                  t=np.array([0.031, -0.019, 0.71])), P_o)
s_t = project(Pc_t)
for n, idx, expect in [(6, [0, 1, 2, 3, 4, 5], True), (3, [0, 1, 2], True),
                       (2, [0, 3], False)]:
    Ln = interaction_matrix(s_t[idx], Pc_t[idx, 2])
    got = use_partition(Ln, Ln[:, XY_COLS], 1e-6, "rank_margin")
    check("rank_margin keeps partition at N=%d: %s" % (n, expect),
          got == expect)

# tau independence: the switch threshold is decoupled from the truncation
# threshold, so the retreat fix must survive the whole swept range.
for tau in [1e-4, 1e-3, 1e-2, 1e-1]:
    swt = run_switched(Pr_, cir_, csr_, lam=0.5, steps=4000, rel_tau=tau)
    check("switched survives tau=%.0e on retreat" % tau,
          np.abs(swt["cam_pos"][:, 2]).max() < 0.85 and swt["err"][-1] < 1e-4,
          "(%.2f m, err %.1e)"
          % (np.abs(swt["cam_pos"][:, 2]).max(), swt["err"][-1]))

# ----------------------------------------------------------------------
# 7. The health metric: sigma_6, the 6th singular value with the spectrum
#    padded to length 6. Zero exactly when DOF are unobservable.
# ----------------------------------------------------------------------
print("\n--- sigma_6 as the health metric ---")

Pc_h = transform_points(make_pose(R=rot_z(0.37) @ rot_x(0.21),
                                  t=np.array([0.031, -0.019, 0.71])), P_o)
s_h = project(Pc_h)
L6 = interaction_matrix(s_h, Pc_h[:, 2])
L2 = interaction_matrix(s_h[[0, 3]], Pc_h[[0, 3], 2])

check("sigma_6 is EXACTLY zero when DOF are unobservable", sigma_6(L2) == 0.0,
      "(L is %dx%d)" % L2.shape)
check("the shape-dependent S[-1] cannot see that null space",
      np.linalg.svd(L2, compute_uv=False)[-1] > 1e-2,
      "(S[-1] = %.3e looks healthy)"
      % np.linalg.svd(L2, compute_uv=False)[-1])
check("sigma_6 is healthy on a full-rank L", sigma_6(L6) > 1e-2,
      "(%.3e)" % sigma_6(L6))

# ----------------------------------------------------------------------
# 8. Fourth case: geometric degeneracy at FULL feature count. All 6 features
#    visible, target collapsed toward a line. Counting features cannot see
#    this, by construction, because the row count never drops.
# ----------------------------------------------------------------------
print("\n--- collapsed target (degeneracy at full feature count) ---")

P_col = ring()
P_col[:, 1] *= 0.02
Pc_c = transform_points(ci, P_col)
L_col = interaction_matrix(project(Pc_c), Pc_c[:, 2])

check("collapsed target keeps all 6 features", L_col.shape[0] == 12)
check("count says partition-on, so counting is blind here",
      use_partition(L_col, L_col[:, XY_COLS], 1e-6, "count"))
check("sigma_6 sees the collapse", sigma_6(L_col) < 1e-3,
      "(%.3e)" % sigma_6(L_col))

THR_RING = calibrate_sigma6(ring())
THR_SQ = calibrate_sigma6(scenario_camera_retreat(angle_deg=180.0)[0])
check("threshold is target dependent, not a global constant",
      THR_SQ / THR_RING > 3,
      "(ring %.2e vs square %.2e, %.1fx apart)"
      % (THR_RING, THR_SQ, THR_SQ / THR_RING))

kwc = dict(lam=0.5, steps=2000)
co_cb = run_partitioned(P_col, ci, cs, rel_tau=1e-3, **kwc)
co_tr = run_truncated(P_col, ci, cs, rel_tau=1e-3, **kwc)
co_ct = run_switched(P_col, ci, cs, rel_tau=1e-3, rule="count", **kwc)
co_s6 = run_switched(P_col, ci, cs, rel_tau=1e-3, rule="sigma6",
                     sigma6_thresh=THR_RING, **kwc)


def peak(lg):
    return float(np.linalg.norm(lg["v"], axis=1).max())


check("combined fails to converge on the collapsed target",
      co_cb["err"][-1] > 1e-4, "(err %.2e, peak |v| %.2f)"
      % (co_cb["err"][-1], peak(co_cb)))
check("switched/count inherits that failure",
      co_ct["err"][-1] > 1e-4 and co_ct["partition"].all(),
      "(err %.2e, partition on all %d steps)"
      % (co_ct["err"][-1], len(co_ct["t"])))
check("switched/sigma6 converges on the collapsed target",
      co_s6["err"][-1] < 1e-4, "(err %.2e)" % co_s6["err"][-1])
check("switched/sigma6 drops the partition throughout",
      not co_s6["partition"].any())
check("switched/sigma6 cuts peak |v| vs count",
      peak(co_s6) < peak(co_ct) / 3.0,
      "(%.2f vs %.2f)" % (peak(co_s6), peak(co_ct)))
check("switched/sigma6 matches truncation-only here",
      abs(peak(co_s6) - peak(co_tr)) < 1e-9)

# ----------------------------------------------------------------------
# 9. The sigma6 rule must not lose the first three cases to win the fourth.
# ----------------------------------------------------------------------
print("\n--- sigma6 rule on the other three cases ---")

Pr_s, cir_s, csr_s = scenario_camera_retreat(angle_deg=180.0)
r_s6 = run_switched(Pr_s, cir_s, csr_s, lam=0.5, steps=4000, rel_tau=1e-3,
                    rule="sigma6", sigma6_thresh=THR_SQ)
check("sigma6: retreat still fixed",
      np.abs(r_s6["cam_pos"][:, 2]).max() < 0.85 and r_s6["err"][-1] < 1e-4,
      "(%.2f m, err %.1e)"
      % (np.abs(r_s6["cam_pos"][:, 2]).max(), r_s6["err"][-1]))

d_s6 = run_switched(P_o, ci, cs, lam=0.5, steps=2000, visible_mask_fn=occ3,
                    rel_tau=1e-3, rule="sigma6", sigma6_thresh=THR_RING)
check("sigma6: dropout still absorbed", spike(d_s6) < 1.5,
      "(%.1fx)" % spike(d_s6))

t_s6 = run_switched(P_o, ci, cs, rel_tau=1e-3, rule="sigma6",
                    sigma6_thresh=THR_RING, **kw2)
check("sigma6: two features under 2x", spike(t_s6) < 2.0,
      "(%.1fx)" % spike(t_s6))

c_s6 = run_switched(P_o, ci, cs, lam=0.5, steps=2000, rel_tau=1e-3,
                    rule="sigma6", sigma6_thresh=THR_RING)
check("sigma6: clean run keeps the partition", c_s6["partition"].all())

# The calibration percentile is the free parameter. Lock its working range.
for pct in [0.1, 5.0]:
    thr_r = calibrate_sigma6(ring(), pct=pct)
    thr_s = calibrate_sigma6(Pr_s, pct=pct)
    rr = run_switched(Pr_s, cir_s, csr_s, lam=0.5, steps=4000, rel_tau=1e-3,
                      rule="sigma6", sigma6_thresh=thr_s)
    cc = run_switched(P_col, ci, cs, rel_tau=1e-3, rule="sigma6",
                      sigma6_thresh=thr_r, **kwc)
    check("sigma6 holds at calibration percentile %.1f" % pct,
          np.abs(rr["cam_pos"][:, 2]).max() < 0.85 and cc["err"][-1] < 1e-4,
          "(retreat %.2f m, collapsed err %.1e)"
          % (np.abs(rr["cam_pos"][:, 2]).max(), cc["err"][-1]))

# ----------------------------------------------------------------------
# 10. Guarding the partition's OWN features, with no spectrum involved.
#     This is the honest competitor to sigma6: the collapsed-target failure is
#     one the partition introduces, so guarding its inputs should suffice.
# ----------------------------------------------------------------------
print("\n--- guard rules vs the spectral rule ---")

AREA_R, ALPHA_R = calibrate_area(ring()), calibrate_alpha(ring())
AREA_S = calibrate_area(scenario_camera_retreat(angle_deg=180.0)[0])
ALPHA_S = calibrate_alpha(scenario_camera_retreat(angle_deg=180.0)[0])

ag = run_switched(P_col, ci, cs, rel_tau=1e-3, rule="area_guard",
                  guard_thresh=AREA_R, **kwc)
check("area_guard converges on the collapsed target",
      ag["err"][-1] < 1e-4 and not ag["partition"].any(),
      "(err %.2e)" % ag["err"][-1])

ag_r = run_switched(Pr_s, cir_s, csr_s, lam=0.5, steps=4000, rel_tau=1e-3,
                    rule="area_guard", guard_thresh=AREA_S)
check("area_guard keeps the retreat fix",
      np.abs(ag_r["cam_pos"][:, 2]).max() < 0.85 and ag_r["err"][-1] < 1e-4,
      "(%.2f m)" % np.abs(ag_r["cam_pos"][:, 2]).max())

ag_2 = run_switched(P_o, ci, cs, rel_tau=1e-3, rule="area_guard",
                    guard_thresh=AREA_R, **kw2)
check("area_guard absorbs two features", spike(ag_2) < 2.0,
      "(%.1fx)" % spike(ag_2))

# alpha_guard alone is NOT sufficient: the two survivors are antipodal, so the
# baseline stays wide and the guard never fires.
al_2 = run_switched(P_o, ci, cs, rel_tau=1e-3, rule="alpha_guard",
                    guard_thresh=ALPHA_R, **kw2)
check("alpha_guard alone FAILS the two-feature case", spike(al_2) > 5,
      "(%.1fx - baseline stays wide, so it must be folded in, not used alone)"
      % spike(al_2))

# ----------------------------------------------------------------------
# 11. Where the two signals genuinely differ: the three-point danger cylinder
#     (Michel & Rives 1993). L is singular while the polygon area is constant.
#     Detection differs; see BRIEF notes for why it does not become a control
#     advantage.
# ----------------------------------------------------------------------
print("\n--- danger cylinder: detection separates, outcome does not ---")

R_ring = 0.05
on_cyl = make_pose(R=np.eye(3), t=np.array([-R_ring, 0.0, 0.6]))
Pc_cyl = transform_points(on_cyl, P_o[[0, 2, 4]])
L_cyl = interaction_matrix(project(Pc_cyl), Pc_cyl[:, 2])
check("camera sits on the danger cylinder",
      abs(np.linalg.norm((-on_cyl[:3, :3].T @ on_cyl[:3, 3])[:2]) - R_ring)
      < 1e-12)
check("L is singular there: sigma_6 ~ 0", sigma_6(L_cyl) < 1e-14,
      "(%.2e, cond %.1e)"
      % (sigma_6(L_cyl),
         np.linalg.svd(L_cyl, compute_uv=False)[0] / max(sigma_6(L_cyl), 1e-300)))
check("the area feature is BLIND to it", _poly(project(Pc_cyl)) > AREA_R,
      "(area %.3e > threshold %.3e, so area_guard cannot fire)"
      % (_poly(project(Pc_cyl)), AREA_R))

# Area is constant as the camera slides along the cylinder; sigma_6 is not.
areas, s6s = [], []
for f in [0.9, 0.98, 1.0, 1.02, 1.1]:
    pose = make_pose(R=np.eye(3), t=np.array([-f * R_ring, 0.0, 0.6]))
    Pcf = transform_points(pose, P_o[[0, 2, 4]])
    areas.append(_poly(project(Pcf)))
    s6s.append(sigma_6(interaction_matrix(project(Pcf), Pcf[:, 2])))
check("area is constant across the singularity",
      (max(areas) - min(areas)) / max(areas) < 1e-9,
      "(spread %.1e)" % ((max(areas) - min(areas)) / max(areas)))
check("sigma_6 collapses across the singularity",
      min(s6s) / max(s6s) < 1e-10, "(min/max %.1e)" % (min(s6s) / max(s6s)))

# ----------------------------------------------------------------------
# 12. The thin-margin fix: calibrate conditioned on the visible count.
# ----------------------------------------------------------------------
print("\n--- count-conditioned calibration widens the thin margin ---")

TAB = calibrate_sigma6_table(ring())
check("threshold table is monotone in feature count",
      all(TAB[n] <= TAB[n + 1] for n in range(2, 6)),
      "(%s)" % {k: "%.1e" % v for k, v in TAB.items()})
check("at N=2 the healthy threshold is exactly 0", TAB[2] == 0.0,
      "(so sigma_6 > 0.0 is False and the partition is still dropped)")

d_s6n = run_switched(P_o, ci, cs, lam=0.5, steps=2000, visible_mask_fn=occ3,
                     rel_tau=1e-3, rule="sigma6_n", sigma6_thresh=TAB)
op = d_s6n["sigma_6"][30:91].min()
check("dropout margin at N=6 calibration is thin", op / TAB[6] < 1.5,
      "(%.2fx)" % (op / TAB[6]))
check("dropout margin at N=3 calibration is wide", op / TAB[3] > 20,
      "(%.1fx)" % (op / TAB[3]))
check("sigma6_n still absorbs the dropout", spike(d_s6n) < 1.5,
      "(%.1fx)" % spike(d_s6n))

t_s6n = run_switched(P_o, ci, cs, rel_tau=1e-3, rule="sigma6_n",
                     sigma6_thresh=TAB, **kw2)
check("sigma6_n absorbs two features", spike(t_s6n) < 2.0,
      "(%.1fx)" % spike(t_s6n))
c_s6n = run_switched(P_col, ci, cs, rel_tau=1e-3, rule="sigma6_n",
                     sigma6_thresh=TAB, **kwc)
check("sigma6_n converges on the collapsed target", c_s6n["err"][-1] < 1e-4,
      "(err %.2e)" % c_s6n["err"][-1])
r_s6n = run_switched(Pr_s, cir_s, csr_s, lam=0.5, steps=4000, rel_tau=1e-3,
                     rule="sigma6_n",
                     sigma6_thresh=calibrate_sigma6_table(Pr_s))
check("sigma6_n keeps the retreat fix",
      np.abs(r_s6n["cam_pos"][:, 2]).max() < 0.85 and r_s6n["err"][-1] < 1e-4,
      "(%.2f m)" % np.abs(r_s6n["cam_pos"][:, 2]).max())

# ----------------------------------------------------------------------
# 13. Is the collapsed-target row evidence, or a tau coincidence?
#     At tau=1e-3 the truncation threshold on L_xy lands ~1% under sigma_min,
#     so the rank flips mid-run. If "both combined" passed the case at a
#     larger tau, the row would be weak evidence for switching. It does not,
#     but HALF the row is still tau-sensitive and must not be quoted as if it
#     were not. These tests pin both halves.
# ----------------------------------------------------------------------
print("\n--- collapsed row: how much of it is a tau artefact ---")

Pc_col = transform_points(ci, P_col)
Lxy_col = interaction_matrix(project(Pc_col), Pc_col[:, 2])[:, XY_COLS]
S_col = np.linalg.svd(Lxy_col, compute_uv=False)
frac = (1e-3 * S_col[0]) / S_col[-1]
check("tau=1e-3 sits within 2% of sigma_min on the collapsed L_xy",
      0.98 < frac < 1.0,
      "(threshold is %.2f%% of sigma_min = %.3e - a 1%% margin)"
      % (100 * frac, S_col[-1]))

# The tau-SENSITIVE half: peak |v|. The dramatic 1.86-vs-0.32 gap at tau=1e-3
# largely closes at larger tau, so it is weak evidence on its own.
peaks = {}
for tau in [1e-4, 1e-3, 1e-2]:
    lg = run_partitioned(P_col, ci, cs, lam=0.5, steps=2000, rel_tau=tau)
    peaks[tau] = float(np.linalg.norm(lg["v"], axis=1).max())
tr_col = run_truncated(P_col, ci, cs, rel_tau=1e-3, lam=0.5, steps=2000)
peak_tr = float(np.linalg.norm(tr_col["v"], axis=1).max())
check("combined's peak |v| on collapsed IS tau sensitive",
      peaks[1e-4] / peaks[1e-2] > 10,
      "(%.2f at tau=1e-4 vs %.2f at tau=1e-2)" % (peaks[1e-4], peaks[1e-2]))
check("at large tau it essentially matches truncation-only",
      abs(peaks[1e-2] - peak_tr) / peak_tr < 0.05,
      "(%.3f vs %.3f - so the peak gap is NOT tau-independent evidence)"
      % (peaks[1e-2], peak_tr))

# The tau-ROBUST half: convergence. Combined never reaches 1e-4, at any tau.
worst = 0.0
for tau in [1e-4, 3e-4, 1e-3, 2e-3, 3e-3, 1e-2, 3e-2, 1e-1]:
    lg = run_partitioned(P_col, ci, cs, lam=0.5, steps=2000, rel_tau=tau)
    worst = max(worst, 0.0)
    check("combined fails to converge on collapsed at tau=%.0e" % tau,
          lg["err"][-1] > 1e-4, "(err %.3e)" % lg["err"][-1])

# ... and that residual is a floor, not slow convergence.
long_run = run_partitioned(P_col, ci, cs, lam=0.5, steps=6000, rel_tau=2e-3)
check("the residual is a floor, not slow convergence",
      abs(long_run["err"][-1] - long_run["err"][2000]) < 1e-12,
      "(err identical at step 2000 and 6000: %.3e)" % long_run["err"][-1])
check("truncation-only settles far below that floor",
      tr_col["err"][-1] < long_run["err"][-1] / 10,
      "(%.2e vs %.2e, %.0fx lower)"
      % (tr_col["err"][-1], long_run["err"][-1],
         long_run["err"][-1] / tr_col["err"][-1]))

# ----------------------------------------------------------------------
# 14. Does the switch chatter under feature noise, and is hysteresis needed?
#     Noise is specified in PIXELS and converted with the focal length of the
#     rendered camera (1920 px wide, 45 deg vertical FOV) so the numbers mean
#     the same thing here and in the MuJoCo study.
# ----------------------------------------------------------------------
print("\n--- partition flips under feature noise ---")

FOCAL_PX = 1738.2


def noisy_features(P_o, sigma_px, seed=7):
    """feature_fn that returns exact projection plus per-point pixel noise."""
    rng = np.random.default_rng(seed)
    n = P_o.shape[0]

    def f(k, cTo):
        s = project(transform_points(cTo, P_o))
        if sigma_px:
            s = s + rng.normal(0.0, sigma_px / FOCAL_PX, s.shape)
        return s, np.ones(n, dtype=bool)
    return f


def off_segments(lg):
    """Contiguous stretches where the partition was dropped."""
    off = np.where(~lg["partition"])[0]
    if not off.size:
        return []
    parts = np.split(off, np.where(np.diff(off) != 1)[0] + 1)
    return [(int(s[0]), int(s[-1]), len(s)) for s in parts]


AREA_RING = calibrate_area(ring())
Pr_n, cir_n, csr_n = scenario_camera_retreat(angle_deg=180.0)
AREA_SQ = calibrate_area(Pr_n)

P_col_n = ring()
P_col_n[:, 1] *= 0.02

NOISE_CASES = [
    # name, target, init, star, threshold, mask, min_feat, legit flips
    ("retreat", Pr_n, cir_n, csr_n, AREA_SQ, None, 3, 0),
    ("dropout", P_o, ci, cs, AREA_RING, occ3, 3, 2),
    ("two features", P_o, ci, cs, AREA_RING, AREA_RING and occ_two, 2, 2),
    ("collapsed", P_col_n, ci, cs, AREA_RING, None, 3, 0),
]

for name, Pn, cin, csn, thr_n, mask, minf, legit in NOISE_CASES:
    for sigma in (0.0, 0.5, 2.0):
        lg = run_switched(P_o=Pn, cTo_init=cin, cTo_star=csn, lam=0.5,
                          steps=600, rel_tau=1e-3, rule="area_guard",
                          guard_thresh=thr_n, min_features=minf,
                          visible_mask_fn=mask,
                          feature_fn=noisy_features(Pn, sigma))
        segs = off_segments(lg)
        # spurious = an isolated blip, i.e. the partition dropped and restored
        # within a couple of steps. A long segment is the guard responding to
        # a real sustained dip, which is what it is for.
        # NOTE: this counts chatter only. On retreat, 0.5 px of noise also
        # causes one SUSTAINED switch that the clean run never makes: noise
        # breaks the start's symmetry and triggers the orbit false positive
        # locked in section 21. It passes here because it is not chatter.
        spurious = sum(1 for s in segs if s[2] <= 2)
        allowed = 0 if sigma <= 0.5 else 1
        check("no chatter: %-12s at %.1f px" % (name, sigma),
              spurious <= allowed,
              "(%d spurious, segments %s)" % (spurious, [s[2] for s in segs]))
        if sigma == 0.0:
            n_flips = int(np.sum(lg["partition"][1:] != lg["partition"][:-1]))
            check("  %-12s clean flips == %d" % (name, legit),
                  n_flips == legit, "(got %d)" % n_flips)

# ----------------------------------------------------------------------
# 15. Hysteresis was TESTED and REJECTED, not omitted.
#     It suppresses the one isolated blip, but it delays BOTH edges - and the
#     edge that matters is dropping the partition when the degeneracy starts.
# ----------------------------------------------------------------------
print("\n--- hysteresis: tested, and it makes things worse ---")

base = run_switched(P_o=P_o, cTo_init=ci, cTo_star=cs, rel_tau=1e-3,
                    rule="area_guard", guard_thresh=AREA_RING, hysteresis=0,
                    **kw2)
check("without hysteresis the two-feature case is protected",
      spike(base) < 2.0, "(%.1fx)" % spike(base))

worse = []
for H in (2, 3, 5):
    lgH = run_switched(P_o=P_o, cTo_init=ci, cTo_star=cs, rel_tau=1e-3,
                       rule="area_guard", guard_thresh=AREA_RING,
                       hysteresis=H, **kw2)
    worse.append(spike(lgH))
    seg = off_segments(lgH)
    check("hysteresis H=%d delays the drop and costs protection" % H,
          spike(lgH) > spike(base) * 3,
          "(spike %.1fx vs %.1fx, drop delayed to step %d)"
          % (spike(lgH), spike(base), seg[0][0] if seg else -1))
check("the cost grows with the hold-off length",
      worse[0] < worse[1] < worse[2],
      "(%.1fx -> %.1fx -> %.1fx)" % tuple(worse))

# ----------------------------------------------------------------------
# 16. The calibration trap: the threshold must come from a KNOWN-GOOD target.
#     Calibrating in situ on a degenerate target makes the degeneracy the norm
#     and silently disables the guard.
# ----------------------------------------------------------------------
print("\n--- calibrating in situ on a degenerate target disables the guard ---")

thr_good = calibrate_area(ring())
thr_situ = calibrate_area(P_col_n)          # calibrated ON the collapsed target
check("in-situ threshold is far below the known-good one",
      thr_situ < thr_good / 5,
      "(%.3e vs %.3e)" % (thr_situ, thr_good))

lg_good = run_switched(P_o=P_col_n, cTo_init=ci, cTo_star=cs, lam=0.5,
                       steps=600, rel_tau=1e-3, rule="area_guard",
                       guard_thresh=thr_good, min_features=3)
lg_situ = run_switched(P_o=P_col_n, cTo_init=ci, cTo_star=cs, lam=0.5,
                       steps=600, rel_tau=1e-3, rule="area_guard",
                       guard_thresh=thr_situ, min_features=3)
check("known-good threshold: guard fires and the run converges",
      (not lg_good["partition"].any()) and lg_good["err"][-1] < 1e-4,
      "(err %.2e)" % lg_good["err"][-1])
check("in-situ threshold: guard never fires and the run does not converge",
      lg_situ["partition"].all() and lg_situ["err"][-1] > 1e-4,
      "(partition on all %d steps, err %.2e)"
      % (len(lg_situ["t"]), lg_situ["err"][-1]))

# ----------------------------------------------------------------------
# 17. Forward prediction of the guard signal (dead claim 6).
#     The predictor is good; the idea still fails, for a structural reason.
# ----------------------------------------------------------------------
print("\n--- forward prediction: good horizon, useless where it matters ---")

from predict import roll_forward, truth, steps_to_fire   # noqa: E402

DT_P, HOR = 0.033, 20
rng_p = np.random.default_rng(0)

# one-step accuracy over a real dt. The 4.8e-07 figure elsewhere is the
# DERIVATIVE at h=1e-5; a step over a real dt carries O(dt^2) truncation.
one_step = []
for _ in range(20):
    cT = make_pose(R=rot_z(rng_p.uniform(-np.pi, np.pi))
                   @ rot_x(rng_p.uniform(-0.3, 0.3)),
                   t=np.array([rng_p.uniform(-0.08, 0.08),
                               rng_p.uniform(-0.08, 0.08),
                               rng_p.uniform(0.5, 0.95)]))
    vv_p = np.concatenate([rng_p.normal(0, 0.15, 3), rng_p.normal(0, 0.3, 3)])
    Pc_p = transform_points(cT, P_o)
    sp, _ = roll_forward(project(Pc_p), Pc_p[:, 2], vv_p, DT_P, 1)
    st, _ = truth(cT, P_o, vv_p, DT_P, 1)
    one_step.extend((np.linalg.norm(sp - st, axis=1) * FOCAL_PX).tolist())
one_step = np.array(one_step)
check("one-step prediction is sub-pixel", np.median(one_step) < 1.0,
      "(median %.3f px, p95 %.3f px)"
      % (np.median(one_step), np.percentile(one_step, 95)))

# the horizon, measured on the DECISION-RELEVANT statistic at high speed
cT_h = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))
Pc_h = transform_points(cT_h, P_o)
errs_h = []
for _ in range(20):
    vv_h = rng_p.normal(0, 1.0, 6)
    vv_h = vv_h / np.linalg.norm(vv_h) * 2.0            # |v| = 2.0
    sp, _ = roll_forward(project(Pc_h), Pc_h[:, 2], vv_h, DT_P, HOR)
    st, _ = truth(cT_h, P_o, vv_h, DT_P, HOR)
    if sp is None:
        continue
    errs_h.append(abs(_poly(sp) - _poly(st)) / AREA_RING * 100)
check("K=20 (660 ms) is trustworthy even at |v|=2",
      np.median(errs_h) < 10.0,
      "(median %.2f%% of threshold, vs a guard margin of 13-62%%)"
      % np.median(errs_h))

# already-degenerate at t=0: nothing to predict
Pc_c = transform_points(ci, P_col_n)
check("collapsed target: already below threshold at t=0",
      steps_to_fire(project(Pc_c), Pc_c[:, 2], np.zeros(6), DT_P, AREA_RING,
                    HOR) == 0)


def warning_ms(runner, Po, cin, csn, thr_w, mask, minf):
    """Milliseconds between the first warning and the guard actually firing."""
    rec = {"s": [], "Z": []}

    def probe(k, cTo):
        Pc = transform_points(cTo, Po)
        rec["s"].append(project(Pc))
        rec["Z"].append(Pc[:, 2].copy())
        return rec["s"][-1], np.ones(Po.shape[0], dtype=bool)

    lg = runner(P_o=Po, cTo_init=cin, cTo_star=csn, lam=0.5, dt=DT_P,
                steps=400, min_features=minf, visible_mask_fn=mask,
                feature_fn=probe)
    V = lg["v"]
    n = min(len(V), len(rec["s"]))
    fire = None
    for k in range(n):
        vis = np.ones(Po.shape[0], dtype=bool) if mask is None \
            else mask(k, rec["s"][k])
        if vis.sum() >= 1 and _poly(rec["s"][k][vis]) < thr_w:
            fire = k
            break
    if fire is None:
        return None, None
    for k in range(fire):
        vis = np.ones(Po.shape[0], dtype=bool) if mask is None \
            else mask(k, rec["s"][k])
        if steps_to_fire(rec["s"][k][vis], rec["Z"][k][vis], V[k], DT_P,
                         thr_w, HOR) is not None:
            return fire, (fire - k) * DT_P * 1000.0
    return fire, 0.0


_f, ms_cam = warning_ms(run_ibvs, Pr_n, cir_n, csr_n, AREA_SQ, None, 3)
check("camera-driven transition gives real warning", ms_cam > 200,
      "(fires at step %d, %.0f ms of warning)" % (_f, ms_cam))

for nm_w, mask_w, mf_w in [("dropout", occ3, 3), ("two features", occ_two, 2)]:
    _f2, ms_w = warning_ms(
        lambda **k: run_switched(rel_tau=1e-3, rule="area_guard",
                                 guard_thresh=AREA_RING, **k),
        P_o, ci, cs, AREA_RING, mask_w, mf_w)
    check("exogenous transition gives NO warning: %s" % nm_w, ms_w == 0.0,
          "(fires at step %d, %.0f ms - an occluder is not in the state)"
          % (_f2, ms_w))

# ----------------------------------------------------------------------
# 18. Where the controller stops and observability begins.
#     Three configurations the controller genuinely cannot finish - and in all
#     three EVERY control law lands on the identical pose error, because the
#     residual is information the features never carried.
# ----------------------------------------------------------------------
print("\n--- observability limits, not control failures ---")


def _cam(T):
    return -T[:3, :3].T @ T[:3, 3]


def pose_err(lg, csn):
    return float(np.linalg.norm(lg["cam_pos"][-1] - _cam(csn)))


def all_controllers(Po, cin, csn, mask, minf, steps=2000):
    """switched / classic / truncation / partitioned on the same case."""
    kw_a = dict(P_o=Po, cTo_init=cin, cTo_star=csn, lam=0.5, dt=0.033,
                steps=steps, min_features=minf, visible_mask_fn=mask)
    return {
        "switched": run_switched(rel_tau=1e-3, rule="area_guard",
                                 guard_thresh=AREA_RING, **kw_a),
        "classic": run_ibvs(**kw_a),
        "truncation": run_truncated(rel_tau=1e-3, **kw_a),
        "partitioned": run_partitioned(**kw_a),
    }


def perm_two(k, s):
    v = np.zeros(6, dtype=bool)
    v[[0, 3]] = True
    return v


P_lin = np.stack([np.linspace(-0.06, 0.06, 3), np.zeros(3), np.zeros(3)],
                 axis=1)
ci_lin = make_pose(R=rot_z(0.3) @ rot_x(0.2), t=np.array([0.03, -0.02, 0.85]))

_R = 0.05
_a = np.array([0.0, 2 * np.pi / 3, 4 * np.pi / 3])
P_cyl = np.stack([_R * np.cos(_a), _R * np.sin(_a), np.zeros(3)], axis=1)
cs_cyl = make_pose(R=np.eye(3), t=np.array([-_R, 0.0, 0.6]))
ci_cyl = make_pose(R=rot_z(0.4) @ rot_x(0.15), t=np.array([-0.02, 0.03, 0.85]))

# per-case expectations, because the three cases do NOT behave uniformly
OBS_CASES = [
    # name, target, init, star, mask, min_feat, pose err, trio agrees?
    ("permanent 2 features", P_o, ci, cs, perm_two, 2, 0.0695, True),
    ("3 collinear points", P_lin, ci_lin, cs, None, 3, 0.1130, True),
    ("goal on danger cylinder", P_cyl, ci_cyl, cs_cyl, None, 3, 0.1405, False),
]

for name, Po, cin, csn, mask, minf, expect, trio_agrees in OBS_CASES:
    R = all_controllers(Po, cin, csn, mask, minf)
    pe = {k: pose_err(v, csn) for k, v in R.items()}
    check("%-24s: pose error is %.4f m" % (name, expect),
          abs(pe["switched"] - expect) < 5e-3, "(%.4f m)" % pe["switched"])
    check("  %-22s: switched and classic land identically" % name,
          abs(pe["switched"] - pe["classic"]) < 1e-3,
          "(%.4f / %.4f m)" % (pe["switched"], pe["classic"]))
    if trio_agrees:
        check("  %-22s: truncation lands there too" % name,
              abs(pe["truncation"] - pe["switched"]) < 1e-3,
              "(%.4f m)" % pe["truncation"])
        check("  %-22s: partitioned is the exception, and worse" % name,
              pe["partitioned"] > pe["switched"] * 5,
              "(%.4f m vs %.4f)" % (pe["partitioned"], pe["switched"]))
    else:
        # on the danger cylinder truncation is the odd one out, and its
        # smaller pose number is NOT a better outcome: it stops early with
        # seven orders more image error than the others.
        check("  %-22s: truncation stops elsewhere" % name,
              abs(pe["truncation"] - pe["switched"]) > 0.02,
              "(%.4f m vs %.4f)" % (pe["truncation"], pe["switched"]))
        check("  %-22s: and it did not converge in image space" % name,
              R["truncation"]["err"][-1] > 1e-6
              and R["switched"]["err"][-1] < 1e-8,
              "(|e| %.1e vs %.1e)"
              % (R["truncation"]["err"][-1], R["switched"]["err"][-1]))
    v_end = float(np.linalg.norm(R["switched"]["v"][-1]))
    check("  %-22s: settles, does not thrash" % name, v_end < 1e-6,
          "(final |v| = %.1e)" % v_end)

# the controller zeroes everything it can actually see
s_star_2 = project(transform_points(cs, P_o))
seen = []


def _probe2(k, cTo):
    s2 = project(transform_points(cTo, P_o))
    seen.append(float(np.linalg.norm((s2[[0, 3]] - s_star_2[[0, 3]])
                                     .reshape(-1))))
    return s2, np.ones(6, dtype=bool)


run_switched(P_o=P_o, cTo_init=ci, cTo_star=cs, lam=0.5, dt=0.033, steps=2000,
             rel_tau=1e-3, rule="area_guard", guard_thresh=AREA_RING,
             min_features=2, visible_mask_fn=perm_two, feature_fn=_probe2)
check("with 2 features it zeroes the error on the pair it can see",
      seen[-1] < 1e-9, "(%.1e, from %.1e)" % (seen[-1], seen[0]))

# ----------------------------------------------------------------------
# 19. The shipped tau has a constructible case where it is WORSE than no
#     intervention at all. This is a caveat, not a closure.
# ----------------------------------------------------------------------
print("\n--- tau=1e-3 has a case where doing nothing is better ---")

ci_rot = make_pose(R=rot_x(0.5), t=np.array([0.0, 0.0, 0.9]))
kw_c = dict(P_o=P_col_n, cTo_init=ci_rot, cTo_star=cs, lam=0.5, dt=0.033,
            steps=2000, min_features=3)

sw_default = run_switched(rel_tau=1e-3, rule="area_guard",
                          guard_thresh=AREA_RING, **kw_c)
sw_tuned = run_switched(rel_tau=1e-5, rule="area_guard",
                        guard_thresh=AREA_RING, **kw_c)
cl_plain = run_ibvs(**kw_c)

check("at the shipped tau=1e-3 the switched controller stalls",
      pose_err(sw_default, cs) > 0.2,
      "(pose err %.4f m)" % pose_err(sw_default, cs))
check("plain classic IBVS converges exactly on the same case",
      pose_err(cl_plain, cs) < 0.01 and cl_plain["err"][-1] < 1e-4,
      "(pose err %.4f m, |e| %.1e)"
      % (pose_err(cl_plain, cs), cl_plain["err"][-1]))
check("tau=1e-5 completes it exactly", pose_err(sw_tuned, cs) < 0.01,
      "(pose err %.4f m)" % pose_err(sw_tuned, cs))
check("so the stall is tau tuning, not structure",
      pose_err(sw_tuned, cs) < pose_err(sw_default, cs) / 10,
      "(%.4f m at 1e-5 vs %.4f m at 1e-3)"
      % (pose_err(sw_tuned, cs), pose_err(sw_default, cs)))

# ----------------------------------------------------------------------
# 20. The area guard cannot tell obliquity from degeneracy; sigma_6 can.
#     Surfaced by the Gazebo arm demo, reproduced and locked here in numpy.
#     This revises dead claim 4: the spectrum is not decoration, the original
#     four-case suite simply contained no case that separates the two signals.
# ----------------------------------------------------------------------
print("\n--- obliquity vs degeneracy: where the two signals part ways ---")

from ibvs_core import rot_y as _rot_y  # noqa: E402
from partitioned import polygon_sigma as _psig  # noqa: E402

A_THR20 = calibrate_area(ring())
S_THR20 = calibrate_sigma6(ring())


def _margins(P, tilt_deg, Zc=0.6):
    cT = make_pose(R=_rot_y(np.deg2rad(tilt_deg)), t=np.array([0.0, 0.0, Zc]))
    Pc = transform_points(cT, P)
    s_ = project(Pc)
    return (_psig(s_) / A_THR20,
            sigma_6(interaction_matrix(s_, Pc[:, 2])) / S_THR20)


a0, s0 = _margins(ring(), 0)
for deg in (70, 75):
    am, sm = _margins(ring(), deg)
    check("healthy target at %d deg: area guard FIRES (false positive)" % deg,
          am < 1.0, "(area margin %.2fx)" % am)
    check("  same view: sigma_6 stays silent" , sm > 1.0,
          "(sigma_6 margin %.2fx)" % sm)
a70, s70 = _margins(ring(), 70)
check("tilting a healthy planar target IMPROVES conditioning",
      s70 > 5 * s0,
      "(sigma_6 margin %.2fx face-on -> %.2fx at 70 deg; area %.2fx -> %.2fx)"
      % (s0, s70, a0, a70))

P_mild = ring()
P_mild[:, 1] *= 0.5
am, sm = _margins(P_mild, 0)
check("mild 3D collapse (c=0.5): sigma_6 fires, area misses it",
      sm < 1.0 <= am, "(sigma_6 %.2fx, area %.2fx)" % (sm, am))

P_hard = ring()
P_hard[:, 1] *= 0.05
am, sm = _margins(P_hard, 0)
check("strong 3D collapse (c=0.05): both fire",
      am < 1.0 and sm < 1.0, "(area %.2fx, sigma_6 %.2fx)" % (am, sm))

# ...and on a TILTING TARGET the false positive is, measured, harmless.
# A spike seen once in the Gazebo arm demo was attributed to the switch and
# did NOT reproduce: across 16 onset/speed combinations peak |v| was identical
# under all three rules in 15. Locked so nobody reintroduces the claim.
# It is not harmless everywhere: section 21.
from partitioned import line_alpha as _la, wrap as _wrap  # noqa: E402
from partitioned import Z_COLS as _ZC, XY_COLS as _XC  # noqa: E402


def _tilt_servo(rule, onset=80, ramp=40, steps=420):
    cT = make_pose(t=np.array([0.04, -0.03, 0.75]))
    goal = make_pose(t=np.array([0.0, 0.0, 0.6]))
    peak, sw, prev = 0.0, 0, None
    for k in range(steps):
        ang = np.deg2rad(75) * float(np.clip((k - onset) / ramp, 0, 1))
        P = ring() @ _rot_y(ang).T
        Pc = transform_points(cT, P)
        s_ = project(Pc)
        ss = project(transform_points(goal, P))
        e_ = (s_ - ss).reshape(-1)
        L_ = interaction_matrix(s_, Pc[:, 2])
        on = (True if rule == "always" else
              _psig(s_) > A_THR20 if rule == "area" else
              sigma_6(L_) > S_THR20)
        if prev is not None and on != prev:
            sw += 1
        prev = on
        if on:
            vz = 0.6 * np.log(_psig(ss) / _psig(s_))
            wz = 0.6 * _wrap(_la(s_) - _la(ss))
            vxy = -pinv_truncated(L_[:, _XC], 1e-3)[0] @ (
                0.5 * e_ + L_[:, _ZC] @ [vz, wz])
            v_ = np.zeros(6)
            v_[_XC] = vxy
            v_[2], v_[5] = vz, wz
        else:
            v_ = -0.5 * pinv_truncated(L_, 1e-3)[0] @ e_
        if k >= onset - 5:
            peak = max(peak, float(np.linalg.norm(v_)))
        cT = se3_exp(-v_ * 0.033) @ cT
    return peak, sw


pk = {r: _tilt_servo(r) for r in ("always", "area", "sigma6")}
check("area guard switches on the tilting healthy target",
      pk["area"][1] >= 1, "(%d switch)" % pk["area"][1])
check("sigma_6 never switches on it", pk["sigma6"][1] == 0)
check("but the false positive costs nothing: peak |v| within 10%",
      abs(pk["area"][0] - pk["always"][0]) <= 0.10 * pk["always"][0],
      "(%.3f area vs %.3f partition-always vs %.3f sigma_6)"
      % (pk["area"][0], pk["always"][0], pk["sigma6"][0]))

# ----------------------------------------------------------------------
# 21. The retreat row, from a start that is not exactly symmetric.
#     The Results table starts the retreat case from an exact rotation about
#     the optical axis, where the partition's xy solve is exactly zero by
#     symmetry. From any small start error, all of that xy command lands in
#     the two weakest directions of L_xy - an orbit about the target - and
#     the view swings nearly edge-on. The area guard reads the foreshortening
#     as degeneracy and hands control to the law that retreats. sigma_6 stays
#     silent (target and matrix are healthy) and inherits the orbit, including
#     the starts where it goes fully edge-on. A trade, not a win. The README
#     rates come from 100 random starts per angle (lateral <= 2 cm, tilt <=
#     3 deg per axis, seed 2026); the cases locked here are drawn from them.
# ----------------------------------------------------------------------
print("\n--- the retreat row from an imperfect start: a trade, not a win ---")

P21 = square_target()
CS21 = make_pose(t=np.array([0.0, 0.0, 0.8]))
A21, S21 = calibrate_area(P21), calibrate_sigma6(P21)


def _start(dx=0.0, dy=0.0, tx=0.0, ty=0.0, ang=180.0):
    return make_pose(R=rot_z(np.deg2rad(ang)) @ rot_x(np.deg2rad(tx))
                     @ _rot_y(np.deg2rad(ty)), t=np.array([dx, dy, 0.8]))


def _seeded_start(i, n=100, seed=2026):
    """Start i of the random sample the README rates come from."""
    r = np.random.default_rng(seed)
    for j in range(n):
        ph, rad = r.uniform(0, 2 * np.pi), r.uniform(0, 0.02)
        tx, ty = r.uniform(-3, 3, 2)
        if j == i:
            return _start(rad * np.cos(ph), rad * np.sin(ph), tx, ty)


def _retreat(ci_, rule, thr=None, steps=1500, noise_rng=None):
    tilt = []

    def fn(k, cTo):
        # angle between the optical axis and the target normal
        tilt.append(np.degrees(np.arccos(min(1.0, abs(cTo[2, 2])))))
        s_ = project(transform_points(cTo, P21))
        if noise_rng is not None:
            s_ = s_ + noise_rng.normal(0.0, 0.5 / FOCAL_PX, s_.shape)
        return s_, np.ones(4, dtype=bool)
    lg = run_switched(P_o=P21, cTo_init=ci_, cTo_star=CS21, lam=0.5,
                      steps=steps, rel_tau=1e-3, rule=rule,
                      guard_thresh=A21 if thr is None else thr,
                      sigma6_thresh=S21, min_features=3, feature_fn=fn)
    # a shortened log means the loop broke: a target point reached the
    # camera plane, i.e. the view went edge-on
    done = len(lg["err"]) == steps
    return dict(aborted=not done, conv=done and lg["err"][-1] < 1e-4,
                dist=float(np.linalg.norm(lg["cam_pos"], axis=1).max()),
                off=int(np.sum(~np.asarray(lg["partition"], dtype=bool))),
                tilt=max(tilt))


def _xy_command(ci_):
    Pc = transform_points(ci_, P21)
    L_ = interaction_matrix(project(Pc), Pc[:, 2])
    _, sv_, Vt_ = np.linalg.svd(L_[:, XY_COLS], full_matrices=False)
    lg = run_switched(P_o=P21, cTo_init=ci_, cTo_star=CS21, lam=0.5, steps=1,
                      rel_tau=1e-3, rule="always", min_features=3)
    vxy = np.asarray(lg["v"][0])[XY_COLS]
    n = float(np.linalg.norm(vxy))
    share = float(np.linalg.norm(Vt_[-2:] @ vxy)) / n if n > 0 else 0.0
    return n, share, sv_[0] / sv_[-1], Vt_[-2:]


n0 = _xy_command(_start())[0]
n2, share2, amp2, weak2 = _xy_command(_start(tx=2.0))
check("exact 180 deg start: the partition commands no xy motion at all",
      n0 < 1e-9, "(|v_xy| %.1e)" % n0)
check("2 deg start error: ALL of v_xy lies in L_xy's two weakest directions",
      share2 > 0.99 and amp2 > 100,
      "(share %.3f, |v_xy| %.2f, weakest direction %.0fx below strongest)"
      % (share2, n2, amp2))
# The two weak singular values are equal, so the SVD may return any mix of
# the pair; test the subspace, not the vectors. An orbit about a point at
# depth Z pairs vx = -Z wy, or vy = +Z wx. Columns are [vx, vy, wx, wy].
_orbits = [np.array([-0.8, 0.0, 0.0, 1.0]), np.array([0.0, 0.8, 1.0, 0.0])]
_in_weak = [float(np.linalg.norm(weak2 @ o) / np.linalg.norm(o))
            for o in _orbits]
check("  and that pair of directions is the orbit about the target",
      min(_in_weak) > 0.99,
      "(orbits at Z = 0.8 lie %s inside it)" % np.round(_in_weak, 4))

_R3 = ("always", "sigma6", "area_guard")
r_ex = {r: _retreat(_start(), r) for r in _R3}
check("exact start (the Results row): nobody switches or retreats",
      all(x["off"] == 0 and x["dist"] < 0.81 and x["conv"]
          for x in r_ex.values()))

r2 = {r: _retreat(_start(tx=2.0), r) for r in _R3}
check("2 deg start: the partition still avoids the retreat...",
      r2["always"]["conv"] and r2["always"]["dist"] < 0.82,
      "(%.2f m)" % r2["always"]["dist"])
check("  ...but swings the view nearly edge-on",
      r2["always"]["tilt"] > 70, "(peak tilt %.0f deg)" % r2["always"]["tilt"])
check("  sigma_6 never fires",
      r2["sigma6"]["off"] == 0
      and abs(r2["sigma6"]["dist"] - r2["always"]["dist"]) < 1e-9)
check("  area guard fires on the foreshortening; the fallback retreats",
      r2["area_guard"]["off"] > 100 and r2["area_guard"]["dist"] > 2.0
      and r2["area_guard"]["conv"],
      "(%d steps off, %.2f m, converges)"
      % (r2["area_guard"]["off"], r2["area_guard"]["dist"]))

s61 = _seeded_start(61)
r61 = {r: _retreat(s61, r) for r in _R3}
check("random start 61: the orbit goes fully edge-on and the partition aborts",
      r61["always"]["aborted"] and r61["always"]["tilt"] > 85,
      "(peak tilt %.1f deg)" % r61["always"]["tilt"])
check("  sigma_6 stays silent and aborts with it",
      r61["sigma6"]["aborted"] and r61["sigma6"]["off"] == 0)
check("  area guard switches early and converges",
      r61["area_guard"]["conv"], "(backs away to %.2f m)"
      % r61["area_guard"]["dist"])

# Calibration cannot buy both. A wider healthy tilt range shrinks the retreat
# until the guard stops firing, and then it aborts with the partition.
r09 = _retreat(s61, "area_guard", thr=calibrate_area(P21, tilt=0.9))
r12 = _retreat(s61, "area_guard", thr=calibrate_area(P21, tilt=1.2))
check("area calibrated over 0.9 rad of tilt: smaller retreat, converges",
      r09["conv"] and r09["dist"] < 1.5, "(%.2f m)" % r09["dist"])
check("  over 1.2 rad: stops firing and aborts with the partition",
      r12["aborted"] and r12["off"] == 0)
_Pm = ring()
_Pm[:, 1] *= 0.5
_m12 = (_psig(project(transform_points(make_pose(t=np.array([0.0, 0.0, 0.6])),
                                       _Pm)))
        / calibrate_area(ring(), tilt=1.2))
check("  and that range blinds it further to mild 3D collapse",
      _m12 > 2.0, "(c=0.5 margin %.2fx, 1.19x at the shipped range)" % _m12)

# Detector noise is enough to trigger it on the EXACT start.
rn = {r: _retreat(_start(), r, steps=600,
                  noise_rng=np.random.default_rng(7))
      for r in ("sigma6", "area_guard")}
check("0.5 px noise on the exact start: area guard switches and retreats",
      rn["area_guard"]["off"] > 100 and rn["area_guard"]["dist"] > 1.5,
      "(%d steps off, %.2f m)" % (rn["area_guard"]["off"],
                                  rn["area_guard"]["dist"]))
check("  sigma_6 under the same noise: no switch, no retreat",
      rn["sigma6"]["off"] == 0 and rn["sigma6"]["dist"] < 0.82)


def _jitter(P, pose, thr_a, thr_s, trials=5000, px=2.0):
    """Mean |change| of each statistic under per-point noise, % of threshold."""
    rr = np.random.default_rng(0)
    Pc = transform_points(pose, P)
    s_, Z_ = project(Pc), Pc[:, 2]
    a_0, s_0 = _psig(s_), sigma_6(interaction_matrix(s_, Z_))
    da = ds = 0.0
    for _ in range(trials):
        sn = s_ + rr.normal(0.0, px / FOCAL_PX, s_.shape)
        da += abs(_psig(sn) - a_0)
        ds += abs(sigma_6(interaction_matrix(sn, Z_)) - s_0)
    return 100 * da / trials / thr_a, 100 * ds / trials / thr_s


ja_sq, js_sq = _jitter(P21, CS21, A21, S21)
ja_rg, js_rg = _jitter(ring(), make_pose(t=np.array([0.0, 0.0, 0.6])),
                       A_THR20, S_THR20)
check("the area STATISTIC is steadier: 2 px moves sigma_6 >2x as much",
      js_sq > 2 * ja_sq and js_rg > 2 * ja_rg,
      "(%% of threshold. square: area %.2f vs sigma_6 %.2f; "
      "ring: %.2f vs %.2f)" % (ja_sq, js_sq, ja_rg, js_rg))

# ----------------------------------------------------------------------
print("\n%d/%d passed" % (sum(_results), len(_results)))
raise SystemExit(0 if all(_results) else 1)
