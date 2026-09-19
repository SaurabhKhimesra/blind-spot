"""Finite-difference verification of every derivative and sign the switched
controller depends on. No reasoning about sign conventions - measure them.

Both bugs previously found in partitioned.py were found this way, so anything
new that depends on a sign gets checked here before it is trusted.
"""

import numpy as np

from blindspot.study.ibvs_core import (se3_exp, make_pose, rot_z, rot_x, transform_points,
                                       project, interaction_matrix, sigma_6)
from blindspot.study.partitioned import (polygon_sigma, line_alpha, wrap, Z_COLS, XY_COLS)
from blindspot.study.truncated import pinv_truncated
from blindspot.study.switched import use_partition, numerical_rank

H = 1e-6
ok = []


def report(name, passed, detail=""):
    ok.append(bool(passed))
    print("[%s] %s %s" % ("  ok  " if passed else " FAIL ", name, detail))


def fd_sdot(cTo, P_o, v, h=H):
    """Measured d(s)/dt under camera velocity v, by central difference."""
    sp = project(transform_points(se3_exp(-v * h) @ cTo, P_o))
    sm = project(transform_points(se3_exp(+v * h) @ cTo, P_o))
    return (sp - sm).reshape(-1) / (2 * h)


# A generic, non-symmetric state so no accidental zero hides a sign error.
def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


P_o = ring()
cTo = make_pose(R=rot_z(0.37) @ rot_x(0.21), t=np.array([0.031, -0.019, 0.71]))
P_c = transform_points(cTo, P_o)
s = project(P_c)
L = interaction_matrix(s, P_c[:, 2])

print("--- 1. interaction matrix is the true derivative ---")
v_probe = np.array([0.043, -0.027, 0.038, 0.11, -0.052, 0.081])
num = fd_sdot(cTo, P_o, v_probe)
rel = np.linalg.norm(num - L @ v_probe) / np.linalg.norm(L @ v_probe)
report("L v == measured sdot", rel < 1e-6, "(rel err %.1e)" % rel)

print("\n--- 2. the two partitioned features, measured not assumed ---")
# d(alpha)/d(wz): the sign that decides whether wz rotates toward the goal.
vv = np.zeros(6); vv[5] = 1.0
a_p = line_alpha(project(transform_points(se3_exp(-vv * H) @ cTo, P_o)))
a_m = line_alpha(project(transform_points(se3_exp(+vv * H) @ cTo, P_o)))
d_alpha_d_wz = wrap(a_p - a_m) / (2 * H)
report("d(alpha)/d(wz) == -1", abs(d_alpha_d_wz + 1.0) < 1e-4,
       "(measured %+.6f)" % d_alpha_d_wz)
# Therefore wz = +lam_a * wrap(alpha - alpha*) is the descent direction:
# alpha_dot = -wz, so wz>0 when alpha is above alpha* drives alpha down.
report("  => wz = +lam_a*wrap(alpha-alpha*) is descent",
       d_alpha_d_wz < 0)

vv = np.zeros(6); vv[2] = 1.0
sg_p = np.log(polygon_sigma(project(transform_points(se3_exp(-vv * H) @ cTo, P_o))))
sg_m = np.log(polygon_sigma(project(transform_points(se3_exp(+vv * H) @ cTo, P_o))))
d_lnsig_d_vz = (sg_p - sg_m) / (2 * H)
report("d(ln sigma)/d(vz) > 0", d_lnsig_d_vz > 0.5,
       "(measured %+.6f)" % d_lnsig_d_vz)
report("  => vz = +lam_z*log(sigma*/sigma) is descent",
       d_lnsig_d_vz > 0)

print("\n--- 3. the coupling-term sign in the partitioned branch ---")
# The claim: v_xy = -pinv(L_xy) @ (lam*e + L_z @ v_z) makes the TOTAL image
# motion equal -lam*e in the directions the reduced inverse can reach.
# Measure the actual sdot of the assembled command and compare.
lam, lam_z, lam_a = 0.5, 0.6, 0.6
cTo_star = make_pose(t=np.array([0.0, 0.0, 0.6]))
s_star = project(transform_points(cTo_star, P_o))
e = (s - s_star).reshape(-1)

sig, sig_star = polygon_sigma(s), polygon_sigma(s_star)
vz = lam_z * np.log(sig_star / sig)
wz = lam_a * wrap(line_alpha(s) - line_alpha(s_star))
v_z = np.array([vz, wz])
L_xy, L_z = L[:, XY_COLS], L[:, Z_COLS]

for sign, label in [(+1.0, "ADDED (as implemented)"),
                    (-1.0, "SUBTRACTED (the wrong sign)")]:
    v_xy = -np.linalg.pinv(L_xy) @ (lam * e + sign * (L_z @ v_z))
    v = np.zeros(6); v[XY_COLS] = v_xy; v[2], v[5] = vz, wz
    sdot = fd_sdot(cTo, P_o, v)               # measured, not L @ v
    resid = np.linalg.norm(sdot + lam * e) / np.linalg.norm(lam * e)
    print("    coupling %-28s residual ||sdot+lam*e||/||lam*e|| = %.4f"
          % (label, resid))
    if sign > 0:
        added_resid = resid
    else:
        sub_resid = resid
report("coupling ADDED cancels the z-axis image motion",
       added_resid < sub_resid / 2,
       "(%.4f vs %.4f wrong-sign)" % (added_resid, sub_resid))

print("\n--- 4. the switching rule fires on the right cases ---")
# Built from real interaction matrices, not hand-made ones: N features from
# the same ring, so the spectrum is physical.
for N, expect in [(6, True), (4, True), (3, True), (2, False)]:
    idx = {6: [0, 1, 2, 3, 4, 5], 4: [0, 1, 2, 3],
           3: [0, 1, 2], 2: [0, 3]}[N]
    Ln = interaction_matrix(s[idx], P_c[idx, 2])
    Lxy_n = Ln[:, XY_COLS]
    got = use_partition(Ln, Lxy_n, 1e-6, "rank_margin")
    report("rank_margin, N=%d -> partition %s" % (N, got), got == expect,
           "(rank L=%d, rank L_xy=%d)"
           % (numerical_rank(Ln, 1e-6), numerical_rank(Lxy_n, 1e-3)))

# Can a single global rank threshold separate a HEALTHY weakly-observable
# target from a genuinely collapsing one? Measure the band, do not assume it.
def spec(P):
    Pc = transform_points(cTo, P)
    S = np.linalg.svd(interaction_matrix(project(Pc), Pc[:, 2]), compute_uv=False)
    return S / S[0]


healthy_weak = spec(P_o)[4]               # sigma_5 of a healthy 6-point ring
P_flat = P_o.copy(); P_flat[:, 1] *= 2e-4  # nearly collinear: genuinely blind
collapsed_weak = spec(P_flat)[4]
# rank_tau(L) > 4 needs tau < healthy_weak to keep the partition on when
# healthy, and tau > collapsed_weak to drop it when collapsed.
band = healthy_weak / collapsed_weak
print("    healthy sigma_5/sigma_1   = %.3e" % healthy_weak)
print("    collapsed sigma_5/sigma_1 = %.3e" % collapsed_weak)
print("    separating band for tau   = (%.2e, %.2e), a factor of %.2f"
      % (collapsed_weak, healthy_weak, band))
print("    -> reported, not asserted: this band is pose dependent (1.65x at the")
print("       desired pose, %.2fx here), far too narrow to set a global tau."
      % band)
print("       Hence tau_switch is an EXISTENCE threshold (1e-6), and")
print("       rank_margin then reduces to `count` on row-limited cases.")

# What the existence threshold does guarantee: the two-feature case is
# row-limited, so it is caught regardless of pose or conditioning.
L2 = interaction_matrix(s[[0, 3]], P_c[[0, 3], 2])
report("two features are row-limited to rank 4 at any tau_switch",
       numerical_rank(L2, 1e-12) <= 4 and not use_partition(L2, L2[:, XY_COLS], 1e-6, "rank_margin"),
       "(rank %d, 4 rows)" % numerical_rank(L2, 1e-12))

print("\n--- 5. the collapsed target really is blind, measured not asserted ---")
# sigma_6 claims a direction is unobservable. Verify by finite difference that
# moving the camera along it produces no measurable image motion.
P_flat2 = P_o.copy(); P_flat2[:, 1] *= 0.02
Pc_f = transform_points(cTo, P_flat2)
Lf2 = interaction_matrix(project(Pc_f), Pc_f[:, 2])
Uf, Sf, Vtf = np.linalg.svd(Lf2, full_matrices=False)

v_blind = Vtf[-1]              # right singular vector of the smallest sigma
v_strong = Vtf[0]              # and of the largest, for comparison
m_blind = np.linalg.norm(fd_sdot(cTo, P_flat2, v_blind))
m_strong = np.linalg.norm(fd_sdot(cTo, P_flat2, v_strong))
print("    measured |sdot| along the weakest direction = %.3e" % m_blind)
print("    measured |sdot| along the strongest         = %.3e" % m_strong)
report("motion along the sigma_6 direction is ~invisible in the image",
       m_blind / m_strong < 1e-3,
       "(ratio %.2e, predicted sigma_6/sigma_1 = %.2e)"
       % (m_blind / m_strong, Sf[-1] / Sf[0]))
report("the measured ratio matches sigma_6/sigma_1",
       abs(m_blind / m_strong - Sf[-1] / Sf[0]) / (Sf[-1] / Sf[0]) < 1e-6,
       "(so sigma_6 is measuring a real unobservable direction)")

# And the padding: with 2 features sigma_6 must be exactly 0, not merely small.
L2f = interaction_matrix(s[[0, 3]], P_c[[0, 3], 2])
report("sigma_6 is EXACTLY zero on a 2-feature L", sigma_6(L2f) == 0.0,
       "(L is %dx%d, so 2 DOF are unobservable)" % L2f.shape)
raw_min = np.linalg.svd(L2f, compute_uv=False)[-1]
report("the old shape-dependent S[-1] hides that null space", raw_min > 1e-2,
       "(S[-1] = %.3e looks healthy; sigma_6 = 0.0)" % raw_min)

# The full-feature-count degeneracy that counting cannot see.
report("collapsed target keeps all 6 features but is caught by sigma_6",
       Lf2.shape[0] == 12 and sigma_6(Lf2) < 1e-3
       and use_partition(Lf2, Lf2[:, XY_COLS], 1e-6, "count"),
       "(12 rows so `count` says partition-on; sigma_6 = %.2e says no)"
       % sigma_6(Lf2))

print("\n%d/%d finite-difference checks passed" % (sum(ok), len(ok)))
raise SystemExit(0 if all(ok) else 1)
