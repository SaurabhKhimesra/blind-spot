"""Does the feature guard survive noise that CO-VARIES with the degeneracy?

Measured the same way as everything else: a margin ratio (operating value of
the guard statistic divided by its calibrated threshold) plus the controller
outcome. The detector's own failure rate is reported SEPARATELY, so a guard
failure and a detection failure never blur together.

    python3 mj_correlated.py
"""

import numpy as np

from ibvs_core import make_pose, rot_z, rot_x
from partitioned import run_partitioned, polygon_sigma
from truncated import run_truncated
from switched import run_switched, calibrate_area
from mj_scene import RenderedFeatureSource, ring
from mj_cases import occluder_boxes, WINDOW, MARKER_M, spike
from mj_degrade import CorrelatedDegrader

P_RING = ring()
CS = make_pose(t=np.array([0.0, 0.0, 0.6]))
CI = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))
STEPS = 150
TAU = 1e-3
# Cycle-time pressure: 10 fps once processing latency is counted, which
# triples inter-frame motion, and a harder-tuned gain to match.
DT = 0.1
LAM = 1.5


class Instrumented:
    """Wraps a feature source and records what the guard would see."""

    def __init__(self, inner, expected_fn, thresh, table=None):
        self.inner = inner
        self.expected_fn = expected_fn
        self.thresh = thresh
        self.table = table or {}
        self.sig = []        # polygon sigma of the detected set
        self.margin = []     # AREA-GUARD margin: area sigma / area threshold.
                             # NOT the sigma_6 margin quoted in the numpy
                             # study - different statistic, different numbers.
        self.trace = []      # (step, n_detected, threshold_used, sigma, margin)
        self.missed = 0      # expected-visible markers the detector lost
        self.expected_total = 0
        self.frames = 0

    def __call__(self, k, cTo):
        s, det = self.inner(k, cTo)
        exp = self.expected_fn(k)
        self.missed += int((exp & ~det).sum())
        self.expected_total += int(exp.sum())
        self.frames += 1
        if det.sum() >= 3:
            sg = polygon_sigma(s[det])
            # margin against the threshold calibrated for THIS feature count.
            # A single N=6 threshold is the known thin-margin defect.
            thr = self.table.get(int(det.sum()), self.thresh)
            self.margin.append(sg / thr)
            self.trace.append((k, int(det.sum()), thr, sg, sg / thr))
        else:
            # fewer than 3 features: there is no polygon at all, so the guard
            # fires by construction. Margin 0, not NaN.
            sg = 0.0
            self.margin.append(0.0)
            self.trace.append((k, int(det.sum()),
                               self.table.get(int(det.sum()), self.thresh),
                               0.0, 0.0))
        self.sig.append(sg)
        return s, det


def build(P_o, occ_idx, thresh, degrade, window=WINDOW, table=None,
          exposure=0.25):
    clear = RenderedFeatureSource(P_o, marker_m=MARKER_M)
    occl = (RenderedFeatureSource(P_o, occluders=occluder_boxes(P_o, occ_idx),
                                  marker_m=MARKER_M)
            if len(occ_idx) else None)
    a, b = window

    def expected(k):
        e = np.ones(P_o.shape[0], dtype=bool)
        if occl is not None and a <= k <= b:
            e[list(occ_idx)] = False
        return e

    class Switcher:
        def render(self, cTo):
            return self._src.render(cTo)

        def detect(self, img):
            return self._src.detect(img)

        def features(self, cTo):
            return self._src.features(cTo)

    sw = Switcher()

    def pick(k):
        sw._src = occl if (occl is not None and a <= k <= b) else clear
        return sw

    if degrade:
        deg = CorrelatedDegrader(sw, exposure_frac=exposure, P_o=P_o,
                                 max_sub=28)

        def base(k, cTo):
            pick(k)
            return deg(k, cTo)
    else:
        def base(k, cTo):
            return pick(k).features(cTo)
        deg = None

    inst = Instrumented(base, expected, thresh, table)
    inst._close = lambda: (clear.close(),
                           occl.close() if occl is not None else None)
    inst._deg = deg
    return inst


def main():
    thr = calibrate_area(P_RING, z_range=(0.5, 1.0), lateral=0.1)
    print("area-guard threshold (healthy 1st pct, 4000 poses, seed 0): %.4e"
          % thr)
    print("Degradation: 4-subframe motion blur along the camera's own measured")
    print("inter-frame motion, plus lagged auto-exposure (lag 4 frames).")
    print("Both are closed loops around the run.\n")

    table = {n: calibrate_area(P_RING, n_visible=n, z_range=(0.5, 1.0),
                               lateral=0.1) for n in (2, 3, 4, 5, 6)}
    print("per-feature-count thresholds: %s\n"
          % {k: "%.3e" % v for k, v in table.items()})

    all_trace = []
    cases = [
        ("clean            ", (), 3, "silent"),
        ("dropout 3of6     ", (0, 1, 2), 3, "silent"),
        ("two features     ", (1, 2, 4, 5), 2, "fire"),
    ]

    print("dt = %.3f s (%.0f fps), lambda = %.2f\n" % (DT, 1.0 / DT, LAM))
    print("Margin = AREA-guard statistic / the threshold calibrated for the")
    print("feature count actually seen that step. This is NOT the sigma_6")
    print("margin of the numpy study: different statistic, not comparable.")
    print("Per-step (step, N, threshold, sigma, margin) written to "
          "docs/guard_trace.csv\n")
    print("%-28s %7s %8s %6s %11s %10s %10s %8s"
          % ("case / degradation", "spike", "blur px", "N win", "thr used",
             "margin pre", "margin WIN", "detector"))
    for cname, occ, minf, want in cases:
        for degrade, expo in ((False, 0.0), (True, 0.04), (True, 0.08),
                              (True, 0.12), (True, 0.20), (True, 0.35)):
            inst = build(P_RING, occ, thr, degrade, table=table,
                         exposure=expo)
            try:
                lg = run_switched(P_o=P_RING, cTo_init=CI, cTo_star=CS,
                                  lam=LAM, dt=DT, steps=STEPS, rel_tau=TAU,
                                  rule="area_guard", guard_thresh=thr,
                                  min_features=minf, feature_fn=inst)
            finally:
                inst._close()
            m = np.array(inst.margin)
            a, b = WINDOW
            pre = m[:a] if a <= len(m) else m
            win = m[a:b + 1] if len(m) > a else np.array([np.nan])
            fail = 100.0 * inst.missed / max(inst.expected_total, 1)
            vmax = float(np.linalg.norm(lg["v"], axis=1).max())
            if want == "silent":
                ok = np.nanmin(pre) > 1.0 and np.nanmin(win) > 1.0
            else:
                ok = np.nanmax(win) < 1.0
            deg = inst._deg
            blur = (max(deg.span_px) if (deg is not None and deg.span_px)
                    else 0.0)
            win_rows = [r for r in inst.trace if a <= r[0] <= b]
            n_win = min((r[1] for r in win_rows), default=0)
            thr_win = min((r[2] for r in win_rows), default=float("nan"))
            print("%-28s %6.1fx %8.1f %6d %11.3e %9.2fx %9.2fx %7.2f%%"
                  % (cname + ("  blur x%.2f" % expo if degrade
                              else "  clean     "),
                     spike(lg, *WINDOW) if len(lg["v"]) > b else float("nan"),
                     blur, n_win, thr_win,
                     np.nanmin(pre), np.nanmin(win), fail))
            all_trace.extend([(cname.strip(), expo) + r for r in inst.trace])
        print()
    _dump(all_trace)


def _dump(all_trace):
    import csv, os
    os.makedirs("docs", exist_ok=True)
    with open(os.path.join("docs", "guard_trace.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case", "exposure", "step", "n_detected",
                    "threshold_used", "area_sigma", "margin"])
        w.writerows(all_trace)


if __name__ == "__main__":
    main()
