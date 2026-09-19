"""Figure 2: four failure modes, and which controllers survive each.

Each column is judged on the measure that is actually ROBUST for that failure,
which is not the same measure in every column:

  retreat, collapsed  -> convergence (final ||e||). For the collapsed target
                         the velocity spike is NOT used: at tau=1e-3 the
                         truncation threshold lands 98.8% of the way to
                         sigma_min, so the spike is a 1.2%-margin artefact
                         (combined's peak runs 5.96 at tau=1e-4 down to 0.33
                         at tau=1e-2, where it matches truncation-only). The
                         convergence failure is tau-independent: combined
                         floors above 1e-4 at every tau from 1e-4 to 1e-1.
  dropout, two feats  -> commanded-velocity spike, relative to each
                         controller's own pre-degradation level.

Because the measures differ per column, this is a status matrix and NOT a bar
chart: a shared value axis would invite comparing a spike against an error
norm. Every column states its own measure and threshold.

    ros2 run blindspot fig_failure_modes     # writes fig2_failure_modes.png
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np

from blindspot.study.ibvs_core import run_ibvs, make_pose, rot_z, rot_x, scenario_camera_retreat
from blindspot.study.partitioned import run_partitioned
from blindspot.study.truncated import run_truncated
from blindspot.study.switched import run_switched, calibrate_sigma6, calibrate_area

TAU = 1e-3
CONVERGED = 1e-4

# palette: status + ink roles (documented reference palette)
GOOD, BAD = "#0ca30c", "#d03b3b"
SURFACE, PLANE = "#fcfcfb", "#f9f9f7"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def spike(lg, a=30, b=90):
    return float(np.linalg.norm(lg["v"][a:b + 1], axis=1).max()
                 / np.linalg.norm(lg["v"][a - 1]))


# ---------------- scenarios ----------------
Pr, cir, csr = scenario_camera_retreat(angle_deg=180.0)
Pd = ring()
csd = make_pose(t=np.array([0.0, 0.0, 0.6]))
cid = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))
Pcol = ring()
Pcol[:, 1] *= 0.02


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


T6R, T6S = calibrate_sigma6(ring()), calibrate_sigma6(Pr)
TAR, TAS = calibrate_area(ring()), calibrate_area(Pr)

CONTROLLERS = [
    ("Classic IBVS", lambda t, a: (lambda *x, **k: run_ibvs(*x, **k))),
    ("Partitioned (2001)",
     lambda t, a: (lambda *x, **k: run_partitioned(*x, **k))),
    ("Adaptive-rank truncation",
     lambda t, a: (lambda *x, **k: run_truncated(*x, rel_tau=TAU, **k))),
    ("Both combined (fixed)",
     lambda t, a: (lambda *x, **k: run_partitioned(*x, rel_tau=TAU, **k))),
    ("Switched, $\\sigma_6$ rule",
     lambda t, a: (lambda *x, **k: run_switched(*x, rel_tau=TAU, rule="sigma6",
                                                sigma6_thresh=t, **k))),
    ("Switched, area guard",
     lambda t, a: (lambda *x, **k: run_switched(*x, rel_tau=TAU,
                                                rule="area_guard",
                                                guard_thresh=a, **k))),
]

# columns: (title, subtitle, measure line, threshold line)
COLS = [
    ("Camera retreat", "180$^\\circ$ about optical axis",
     "convergence", "pass: final $\\|e\\| < 10^{-4}$"),
    ("Feature dropout", "3 of 6 occluded",
     "velocity spike", "pass: $< 1.5\\times$"),
    ("Two features left", "only 2 of 6 survive",
     "velocity spike", "pass: $< 2\\times$"),
    ("Collapsed target", "all 6 visible, near-collinear",
     "convergence", "pass: final $\\|e\\| < 10^{-4}$"),
]

cells = []
for name, factory in CONTROLLERS:
    run_sq, run_rg = factory(T6S, TAS), factory(T6R, TAR)

    rt = run_sq(Pr, cir, csr, lam=0.5, steps=4000)
    dr = run_rg(Pd, cid, csd, lam=0.5, steps=2000, visible_mask_fn=occ3)
    tw = run_rg(Pd, cid, csd, lam=0.5, steps=2000, visible_mask_fn=occ_two,
                min_features=2)
    co = run_rg(Pcol, cid, csd, lam=0.5, steps=2000)

    row = [
        (rt["err"][-1] < CONVERGED,
         "converges" if rt["err"][-1] < CONVERGED else "diverges",
         "%.2f m off-target" % abs(rt["cam_pos"][:, 2]).max()),
        (spike(dr) < 1.5, "%.1f$\\times$" % spike(dr), "peak / pre-window"),
        (spike(tw) < 2.0, "%.1f$\\times$" % spike(tw), "peak / pre-window"),
        (co["err"][-1] < CONVERGED,
         "converges" if co["err"][-1] < CONVERGED else "floors",
         "$\\|e\\|$ = %.0e" % co["err"][-1]),
    ]
    cells.append((name, row))

# ---------------- draw ----------------
fig = plt.figure(figsize=(13.6, 8.9))
fig.patch.set_facecolor(PLANE)
ax = fig.add_axes([0.205, 0.150, 0.780, 0.632])
ax.set_facecolor(PLANE)
ax.set_xlim(0, 4)
ax.set_ylim(0, len(cells))
ax.axis("off")

GAP = 0.035          # 2px-equivalent surface gap between fills
for r, (name, row) in enumerate(cells):
    y = len(cells) - 1 - r
    for c, (ok, big, small) in enumerate(row):
        face = GOOD if ok else BAD
        ax.add_patch(FancyBboxPatch(
            (c + GAP, y + GAP), 1 - 2 * GAP, 1 - 2 * GAP,
            boxstyle="round,pad=0,rounding_size=0.045",
            linewidth=0, facecolor=face))
        ax.text(c + 0.5, y + 0.62, ("\u2713  " if ok else "\u2717  ") + big,
                ha="center", va="center", color="white",
                fontsize=15, fontweight="bold")
        ax.text(c + 0.5, y + 0.31, small, ha="center", va="center",
                color="white", fontsize=9.5, alpha=0.92)
    ax.text(-0.04, y + 0.5, name, ha="right", va="center",
            color=INK, fontsize=11.5,
            fontweight="bold" if "Switched" in name else "normal")

# column headers, each stating its OWN measure and threshold
for c, (title, sub, measure, thr) in enumerate(COLS):
    ax.text(c + 0.5, len(cells) + 0.70, title, ha="center", va="center",
            color=INK, fontsize=12.5, fontweight="bold")
    ax.text(c + 0.5, len(cells) + 0.49, sub, ha="center", va="center",
            color=INK2, fontsize=9.5)
    ax.text(c + 0.5, len(cells) + 0.30, "judged on " + measure,
            ha="center", va="center", color=MUTED, fontsize=9.0,
            fontstyle="italic")
    ax.text(c + 0.5, len(cells) + 0.12, thr, ha="center", va="center",
            color=MUTED, fontsize=8.8)

fig.text(0.5, 0.972, "Guard the features your controller is built on, "
         "not the matrix it inverts",
         ha="center", va="center", fontsize=17, fontweight="bold", color=INK)
fig.text(0.5, 0.928, "Four IBVS failure modes. Only a controller that "
         "switches the 2001 partition off at runtime survives all four.",
         ha="center", va="center", color=INK2, fontsize=11)

FOOT = [
    ("Columns are judged on DIFFERENT measures \u2014 convergence for retreat "
     "and collapsed, velocity spike for the two occlusion", INK2),
    ("cases \u2014 so cells are comparable down a column, never across one.",
     INK2),
    ("Collapsed is reported on convergence, not spike: its spike is a "
     "1.2%-margin artefact of $\\tau$. Combined peaks at 5.96 "
     "($\\tau$=10$^{-4}$)", MUTED),
    ("but 0.33 ($\\tau$=10$^{-2}$), matching truncation-only; the "
     "convergence failure holds at every $\\tau$ from 10$^{-4}$ to "
     "10$^{-1}$.", MUTED),
    ("$\\tau$ = 10$^{-3}$. Guard and $\\sigma_6$ thresholds: healthy 1st "
     "percentile over 4000 random poses (seed 0), calibrated per target. "
     "Reproduce: ros2 run blindspot compare", MUTED),
]
for i, (line, col) in enumerate(FOOT):
    fig.text(0.022, 0.098 - i * 0.0215, line, ha="left", va="center",
             color=col, fontsize=9.0)

fig.savefig("fig2_failure_modes.png", dpi=170, facecolor=PLANE)
print("wrote fig2_failure_modes.png")
for name, row in cells:
    print("  %-26s %s" % (name, "".join("P" if c[0] else "F" for c in row)))
