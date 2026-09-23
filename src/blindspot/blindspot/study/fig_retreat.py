"""Figure 1: image error is a liar.

During a classic IBVS camera-retreat failure the feature error decreases
monotonically and the commanded velocity is perfectly aligned with error
reduction at every step, while the camera flies metres away from the target.
The interaction-matrix conditioning is the signal that sees it coming.
"""
import os

import matplotlib
matplotlib.use("Agg")                        # headless: must precede pyplot

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from blindspot.study.ibvs_core import scenario_camera_retreat, run_ibvs  # noqa: E402


def _out(name):
    """Write beside the committed copy in docs/, not into whatever cwd we ran from."""
    d = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", "..", "..", "..", "docs"))
    return os.path.join(d, name) if os.path.isdir(d) else name


INK = "#1c1c1f"
MUTED = "#6b6b73"
BAD = "#c0392b"
GOOD = "#2c7fb8"
WARN = "#e08214"

plt.rcParams.update({
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9.5,
    "axes.edgecolor": "#c8c8cf", "axes.labelcolor": INK,
    "text.color": INK, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": "white", "axes.facecolor": "white",
    "legend.frameon": False, "lines.linewidth": 1.7,
})

P_o, ci, cs = scenario_camera_retreat(angle_deg=160.0)
log = run_ibvs(P_o, ci, cs, lam=0.5, steps=1200)

t = log["t"]
Z = np.abs(log["cam_pos"][:, 2])
err = log["err"]
cond = log["cond"]
cos_align = log["cos_align"]

peak = int(Z.argmax())
# first index where conditioning has grown 50% above its starting value
warn = int(np.argmax(cond > 1.5 * cond[0]))
lead = t[peak] - t[warn]

fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.5))

ax = axes[0]
ax.plot(t, err, color=GOOD)
ax.set_xlabel("time (s)")
ax.set_ylabel("image feature error  ‖e‖")
ax.set_title("What the controller sees", loc="left", fontweight="bold")
ax.text(0.5, 0.62, "monotonic decrease\nthe whole way down",
        transform=ax.transAxes, color=GOOD, fontsize=8.5, ha="left")
ax.set_xlim(0, 9)

ax = axes[1]
ax.plot(t, Z, color=BAD)
ax.axhline(Z[-1], color=MUTED, lw=0.8, ls=":")
ax.set_xlabel("time (s)")
ax.set_ylabel("camera distance to target (m)")
ax.set_title("What the robot actually does", loc="left", fontweight="bold")
ax.annotate("retreats to %.1f m\nbefore recovering" % Z[peak],
            xy=(t[peak], Z[peak]), xytext=(t[peak] + 1.4, Z[peak] * 0.82),
            color=BAD, fontsize=8.5,
            arrowprops=dict(arrowstyle="-", color=BAD, lw=0.9))
ax.set_xlim(0, 9)

ax = axes[2]
ax.semilogy(t, cond, color=WARN)
ax.axvline(t[warn], color=WARN, lw=0.9, ls="--")
ax.axvline(t[peak], color=BAD, lw=0.9, ls="--")
ax.set_xlabel("time (s)")
ax.set_ylabel("cond($L$)   (log scale)")
ax.set_title("The signal that saw it coming", loc="left", fontweight="bold")
ax.annotate("", xy=(t[warn], cond.max() * 1.9), xytext=(t[peak], cond.max() * 1.9),
            arrowprops=dict(arrowstyle="<->", color=INK, lw=0.9))
ax.text(t[peak] + 0.35, cond.max() * 1.9,
        "%.1f s of warning" % lead, ha="left", va="center", fontsize=8.5, color=INK)
ax.set_ylim(cond.min() * 0.7, cond.max() * 4.5)
ax.text(t[warn] + 0.25, cond.min() * 1.15, "conditioning\nbreaks away", color=WARN, fontsize=8.2, va="bottom")
ax.set_xlim(0, 9)

fig.suptitle("Image error is a liar", x=0.008, y=0.985, ha="left",
             fontsize=11.5, fontweight="bold")
fig.text(0.008, 0.905,
         "IBVS camera retreat, 160\u00b0 optical-axis rotation. The commanded velocity reduces image error\n"
         "optimally at every single step (alignment = %.3f throughout), yet the camera leaves the workspace."
         % cos_align.mean(),
         fontsize=8.4, color=MUTED, va="top")

fig.tight_layout(rect=[0, 0, 1, 0.80])
fig.savefig(_out("fig1_retreat.png"), dpi=190)
print("cos_align min/max: %.6f %.6f" % (cos_align.min(), cos_align.max()))
print("warning at t=%.2f s, peak excursion at t=%.2f s, lead = %.2f s" % (t[warn], t[peak], lead))
print("peak distance %.2f m (started at %.2f m)" % (Z[peak], Z[0]))
print("saved %s" % _out("fig1_retreat.png"))
