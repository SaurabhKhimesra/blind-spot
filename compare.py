"""Regenerates the controller comparison table, including the switched rows.

Metric definitions, pinned once and used for every row.

  Retreat 180  : max |camera z| in the object frame over the run, plus whether
                 the run converges (final ||e|| < 1e-4).
  Dropout spike: spike() = peak commanded |v| inside the degradation window
                 (steps 30..90) divided by that controller's own |v| at step
                 29, immediately before the window. 3 of 6 features occluded.
  2 features   : the same spike(), with only features 0 and 3 surviving.
  Collapsed    : target geometry scaled to 2% of its width along one axis, all
                 6 features visible for the whole run. The degeneracy is
                 present at every step, so there is NO pre-degradation window
                 to normalise against and spike() is undefined. Reported
                 instead as peak commanded |v| over the run (absolute, same
                 scenario for every controller) and whether it converges.
  Clean cost   : steps until ||e|| first falls below 1% of its initial value
                 on an undegraded run of the 6-point ring, relative to classic
                 IBVS. The 1% threshold is part of the definition: the number
                 moves with it (partitioned is +2.4% at 50%, +13% at 1%,
                 +7.4% at 0.01%), so quoting a cost without the threshold is
                 meaningless.

    python3 compare.py
"""

import numpy as np

from ibvs_core import run_ibvs, make_pose, rot_z, rot_x, scenario_camera_retreat
from partitioned import run_partitioned
from truncated import run_truncated
from switched import (run_switched, calibrate_sigma6, calibrate_area,
                      calibrate_alpha, calibrate_sigma6_table)

TAU = 1e-3
SETTLE_FRAC = 0.01
CONVERGED = 1e-4


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def spike(lg, a=30, b=90):
    return (np.linalg.norm(lg["v"][a:b + 1], axis=1).max()
            / np.linalg.norm(lg["v"][a - 1]))


def settle(lg, frac=SETTLE_FRAC):
    thr = frac * lg["err"][0]
    b = np.where(lg["err"] < thr)[0]
    return int(b[0]) if b.size else -1


# --- scenarios -------------------------------------------------------
Pr, cir, csr = scenario_camera_retreat(angle_deg=180.0)

Pd = ring()
csd = make_pose(t=np.array([0.0, 0.0, 0.6]))
cid = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))

P_collapsed = ring()
P_collapsed[:, 1] *= 0.02        # collapsed toward a line, all 6 still visible


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


# --- sigma6 thresholds, calibrated per target from healthy poses -----
# Calibrated on the HEALTHY target of the same nominal geometry, never on the
# degraded one. sigma_6 carries the scale of the target and the viewing
# distance, so a single global threshold does not transfer between targets.
THR_RING = calibrate_sigma6(ring())
THR_SQUARE = calibrate_sigma6(Pr)
AREA_RING, AREA_SQUARE = calibrate_area(ring()), calibrate_area(Pr)
ALPHA_RING, ALPHA_SQUARE = calibrate_alpha(ring()), calibrate_alpha(Pr)
TAB_RING, TAB_SQUARE = calibrate_sigma6_table(ring()), calibrate_sigma6_table(Pr)

CONTROLLERS = [
    ("Classic IBVS", lambda thr: (lambda *a, **k: run_ibvs(*a, **k))),
    ("Partitioned (2001)", lambda thr: (lambda *a, **k: run_partitioned(*a, **k))),
    ("Adaptive-rank truncation",
     lambda thr: (lambda *a, **k: run_truncated(*a, rel_tau=TAU, **k))),
    ("Both combined",
     lambda thr: (lambda *a, **k: run_partitioned(*a, rel_tau=TAU, **k))),
    ("Switched (count)",
     lambda thr: (lambda *a, **k: run_switched(*a, rel_tau=TAU, rule="count", **k))),
    ("Switched (rank_margin)",
     lambda thr: (lambda *a, **k: run_switched(*a, rel_tau=TAU,
                                               rule="rank_margin", **k))),
    ("Switched (sigma6)",
     lambda thr: (lambda *a, **k: run_switched(*a, rel_tau=TAU, rule="sigma6",
                                               sigma6_thresh=thr[0], **k))),
    ("Switched (sigma6 per-N)",
     lambda thr: (lambda *a, **k: run_switched(*a, rel_tau=TAU,
                                               rule="sigma6_n",
                                               sigma6_thresh=thr[3], **k))),
    ("Switched (area_guard)",
     lambda thr: (lambda *a, **k: run_switched(*a, rel_tau=TAU,
                                               rule="area_guard",
                                               guard_thresh=thr[1], **k))),
    ("Switched (alpha_guard)",
     lambda thr: (lambda *a, **k: run_switched(*a, rel_tau=TAU,
                                               rule="alpha_guard",
                                               guard_thresh=thr[2], **k))),
]

rows = []
for name, factory in CONTROLLERS:
    run_sq = factory((THR_SQUARE, AREA_SQUARE, ALPHA_SQUARE, TAB_SQUARE))
    run_rg = factory((THR_RING, AREA_RING, ALPHA_RING, TAB_RING))

    rt = run_sq(Pr, cir, csr, lam=0.5, steps=4000)
    dr = run_rg(Pd, cid, csd, lam=0.5, steps=2000, visible_mask_fn=occ3)
    tw = run_rg(Pd, cid, csd, lam=0.5, steps=2000, visible_mask_fn=occ_two,
                min_features=2)
    co = run_rg(P_collapsed, cid, csd, lam=0.5, steps=2000)
    cl = run_rg(Pd, cid, csd, lam=0.5, steps=2000)

    rows.append(dict(
        name=name,
        retreat=float(np.abs(rt["cam_pos"][:, 2]).max()),
        rconv=rt["err"][-1] < CONVERGED,
        drop=spike(dr), two=spike(tw),
        cpeak=float(np.linalg.norm(co["v"], axis=1).max()),
        cconv=co["err"][-1] < CONVERGED,
        settle=settle(cl)))

base = rows[0]["settle"]
hdr = ("%-24s %18s %9s %9s %22s %11s"
       % ("Controller", "Retreat 180", "Dropout", "2 feats",
          "Collapsed (6 feats)", "Clean cost"))
print(hdr)
print("-" * len(hdr))
for r in rows:
    ret = ("%.2f m, conv" % r["retreat"] if r["rconv"]
           else "%.2f m, NO conv" % r["retreat"])
    col = ("peak %.2f, %s" % (r["cpeak"], "conv" if r["cconv"] else "NO conv"))
    cost = ("baseline" if r["settle"] == base
            else "%+.0f%%" % (100.0 * (r["settle"] - base) / base))
    print("%-24s %18s %8.1fx %8.1fx %22s %11s"
          % (r["name"], ret, r["drop"], r["two"], col, cost))

print("\ntau = %g. Convergence means final ||e|| < %g. Clean cost is steps to"
      % (TAU, CONVERGED))
print("||e|| < %g of initial, relative to classic IBVS." % SETTLE_FRAC)
print("All thresholds: healthy 1st percentile over 4000 random poses (seed 0),")
print("calibrated per target. Guard rules use the identical procedure, so the")
print("rules differ only in WHICH quantity they test.")
print("            %12s %12s %12s" % ("sigma6", "area", "alpha"))
print("   ring     %12.3e %12.3e %12.3e" % (THR_RING, AREA_RING, ALPHA_RING))
print("   square   %12.3e %12.3e %12.3e"
      % (THR_SQUARE, AREA_SQUARE, ALPHA_SQUARE))
print("   sigma6 per-N table (ring): %s"
      % {k: "%.2e" % v for k, v in TAB_RING.items()})
print("Collapsed column is peak |v|, not a ratio: the degeneracy is present")
print("for the whole run so there is no pre-degradation level to divide by.")
