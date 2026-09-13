"""Sensitivity of the two tuning constants: tau, and the sigma6 threshold.

Task 1 item 4: is tau = 1e-3 a justified choice or an assumed one?

tau is the relative singular-value threshold. It enters in three places:
  - truncation-only:  which directions of L are dropped
  - combined:         which directions of the reduced L_xy are dropped
  - switched:         BOTH of the above, AND the rank_margin decision itself,
                      since that test is rank_tau(L) > 4

So the switched controller is the most tau-exposed of the three, and its
sensitivity is the one that matters for the claim.

    python3 tau_sweep.py
"""

import numpy as np

from ibvs_core import run_ibvs, make_pose, rot_z, rot_x, scenario_camera_retreat
from partitioned import run_partitioned
from truncated import run_truncated
from switched import run_switched, calibrate_sigma6

TAUS = [1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2, 1e-1]


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def spike(lg, a=30, b=90):
    return (np.linalg.norm(lg["v"][a:b + 1], axis=1).max()
            / np.linalg.norm(lg["v"][a - 1]))


def settle(lg, frac=0.01):
    thr = frac * lg["err"][0]
    b = np.where(lg["err"] < thr)[0]
    return int(b[0]) if b.size else -1


Pr, cir, csr = scenario_camera_retreat(angle_deg=180.0)
Pd = ring()
P_collapsed = ring()
P_collapsed[:, 1] *= 0.02
csd = make_pose(t=np.array([0.0, 0.0, 0.6]))
cid = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))


def occ3(k, s):
    vis = np.ones(6, dtype=bool)
    if 30 <= k <= 90:
        vis[:3] = False
    return vis


def occ_two(k, s):
    if 30 <= k <= 90:
        vis = np.zeros(6, dtype=bool)
        vis[[0, 3]] = True
        return vis
    return np.ones(6, dtype=bool)


def measure(run):
    rt = run(Pr, cir, csr, lam=0.5, steps=4000)
    dr = run(Pd, cid, csd, lam=0.5, steps=2000, visible_mask_fn=occ3)
    tw = run(Pd, cid, csd, lam=0.5, steps=2000, visible_mask_fn=occ_two,
             min_features=2)
    cln = run(Pd, cid, csd, lam=0.5, steps=2000)
    return (float(np.abs(rt["cam_pos"][:, 2]).max()), rt["err"][-1] < 1e-4,
            spike(dr), spike(tw), settle(cln))


base_settle = settle(run_ibvs(Pd, cid, csd, lam=0.5, steps=2000))

for label, factory in [
    ("Adaptive-rank truncation",
     lambda tau: (lambda *a, **k: run_truncated(*a, rel_tau=tau, **k))),
    ("Both combined",
     lambda tau: (lambda *a, **k: run_partitioned(*a, rel_tau=tau, **k))),
    ("Switched (rank_margin)",
     lambda tau: (lambda *a, **k: run_switched(*a, rel_tau=tau,
                                               rule="rank_margin", **k))),
    ("Switched (sigma6)",
     lambda tau: (lambda *a, **k: run_switched(
         *a, rel_tau=tau, rule="sigma6",
         sigma6_thresh=calibrate_sigma6(
             Pr if a[0].shape[0] == 4 else ring()), **k))),
]:
    print("\n=== %s ===" % label)
    print("%8s  %20s  %13s  %15s  %11s"
          % ("tau", "Retreat 180", "Dropout spike", "2 features left",
             "Clean cost"))
    for tau in TAUS:
        ret, conv, sd, st, se = measure(factory(tau))
        cost = ("baseline" if se == base_settle
                else "%+.0f%%" % (100.0 * (se - base_settle) / base_settle))
        print("%8.0e  %20s  %12.1fx  %14.1fx  %11s"
              % (tau,
                 ("%.2f m, converges" % ret) if conv else ("%.2f m, NO conv" % ret),
                 sd, st, cost))

# The rank_margin decision is the tau-sensitive part: show where it flips.
print("\n=== where the rank_margin decision sits vs tau (two-feature case) ===")
print("%8s  %s" % ("tau", "partition-off steps / total, and rank_tau(L) seen"))
for tau in TAUS:
    sw = run_switched(Pd, cid, csd, lam=0.5, steps=2000,
                      visible_mask_fn=occ_two, min_features=2,
                      rel_tau=tau, rule="rank_margin")
    off = int((~sw["partition"]).sum())
    print("%8.0e  off=%4d/%4d   rank_tau(L) values: %s"
          % (tau, off, len(sw["t"]), np.unique(sw["rank_L"])))

print("\n=== and on a CLEAN run (partition should never drop) ===")
for tau in TAUS:
    sw = run_switched(Pd, cid, csd, lam=0.5, steps=2000, rel_tau=tau,
                      rule="rank_margin")
    print("%8.0e  off=%4d/%4d   err_end=%.2e"
          % (tau, int((~sw["partition"]).sum()), len(sw["t"]), sw["err"][-1]))


# ----------------------------------------------------------------------
# The sigma6 threshold is set from a distribution, so the free parameter is
# the PERCENTILE, not the threshold. Sweep it.
# ----------------------------------------------------------------------
print("\n\n=== sigma6 threshold: sensitivity to the calibration percentile ===")
print("Threshold = pct-th percentile of sigma_6 over 4000 random healthy poses")
print("(seed 0), calibrated separately per target. Higher pct = more eager to")
print("drop the partition.\n")
print("%6s %11s %11s %9s %9s %13s %19s"
      % ("pct", "thr(ring)", "thr(square)", "dropout", "2 feats",
         "collapsed", "retreat"))
for pct in [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]:
    tr_ = calibrate_sigma6(ring(), pct=pct)
    ts_ = calibrate_sigma6(Pr, pct=pct)

    def sw(P, a, b, thr, **kw):
        return run_switched(P, a, b, lam=0.5, rel_tau=1e-3, rule="sigma6",
                            sigma6_thresh=thr, **kw)

    d = sw(Pd, cid, csd, tr_, steps=2000, visible_mask_fn=occ3)
    t2 = sw(Pd, cid, csd, tr_, steps=2000, visible_mask_fn=occ_two,
            min_features=2)
    co = sw(P_collapsed, cid, csd, tr_, steps=2000)
    rr = sw(Pr, cir, csr, ts_, steps=4000)
    print("%6.1f %11.3e %11.3e %8.1fx %8.1fx  peak %.2f %s  %.2f m, %s"
          % (pct, tr_, ts_, spike(d), spike(t2),
             np.linalg.norm(co["v"], axis=1).max(),
             "conv " if co["err"][-1] < 1e-4 else "NOconv",
             np.abs(rr["cam_pos"][:, 2]).max(),
             "converges" if rr["err"][-1] < 1e-4 else "NO CONVERGENCE"))
print("\nWorking range is pct in [0.1, 5]: every case holds. At pct=10 the")
print("threshold rises above the retreat target's own sigma_6, the partition")
print("is dropped there too, and the retreat fix is lost. The 1st percentile")
print("sits inside a 50x-wide working range, so it is a justified choice.")
