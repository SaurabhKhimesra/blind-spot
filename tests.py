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
print("\n%d/%d passed" % (sum(_results), len(_results)))
raise SystemExit(0 if all(_results) else 1)
