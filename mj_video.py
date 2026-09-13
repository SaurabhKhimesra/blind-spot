"""Side-by-side video: a fixed controller failing, the guard-switched one not.

Same scene, same occlusion, same correlated degradation, same random seed.
The ONLY difference between the two panels is the control law.

    python3 mj_video.py            # writes docs/guard_vs_fixed.mp4
"""

import os
import numpy as np
import cv2
import imageio.v2 as imageio

from ibvs_core import make_pose, rot_z, rot_x
from partitioned import run_partitioned, polygon_sigma
from switched import run_switched, calibrate_area
from mj_scene import RenderedFeatureSource, ring, intrinsics, MARKER_M
from mj_cases import occluder_boxes
from mj_degrade import CorrelatedDegrader

OUT = os.path.join("docs", "guard_vs_fixed.mp4")
DT, LAM, TAU = 0.1, 0.5, 1e-3
STEPS = 300
FPS = 10
WINDOW = (4, 184)          # occlusion opens while the error is still large
EXPOSURE = 0.04
PANEL = (480, 360)          # per-panel size in the video
OCC = (1, 2, 4, 5)          # leaves features 0 and 3

P_RING = ring()
CS = make_pose(t=np.array([0.0, 0.0, 0.6]))
CI = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))
fx, fy, cx, cy = intrinsics()


class Recorder:
    """Feature source that also keeps a small copy of every rendered frame."""

    def __init__(self, P_o, occ_idx, window, expose):
        self.clear = RenderedFeatureSource(P_o, marker_m=MARKER_M)
        self.occl = RenderedFeatureSource(
            P_o, occluders=occluder_boxes(P_o, occ_idx), marker_m=MARKER_M)
        self.a, self.b = window
        self.frames, self.info = [], []
        self._cur = None
        outer = self

        class Sw:
            def render(self, cTo):
                return outer._cur.render(cTo)

            def detect(self, img):
                return outer._cur.detect(img)
        self.deg = CorrelatedDegrader(Sw(), exposure_frac=expose, P_o=P_o,
                                      max_sub=20)

    def __call__(self, k, cTo):
        self._cur = self.occl if self.a <= k <= self.b else self.clear
        img = self.deg._exposure_stack(cTo)
        img = np.clip(img, 0, 255).astype(np.uint8)
        self.deg.prev_cTo = cTo.copy()
        s, det = self._cur.detect(img)
        small = cv2.resize(img, PANEL, interpolation=cv2.INTER_AREA)
        self.frames.append(small)
        self.info.append((int(det.sum()),
                          float(polygon_sigma(s[det])) if det.sum() >= 3
                          else 0.0))
        return s, det

    def close(self):
        self.clear.close()
        self.occl.close()


def annotate(frame, title, k, n_det, v, guard, lost, colour):
    f = frame.copy()
    h, w = f.shape[:2]
    cv2.rectangle(f, (0, 0), (w, 34), (255, 255, 255), -1)
    cv2.putText(f, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                colour, 2, cv2.LINE_AA)
    bar = "features %d/6   |v| %5.2f" % (n_det, v)
    cv2.rectangle(f, (0, h - 30), (w, h), (255, 255, 255), -1)
    cv2.putText(f, bar, (10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (30, 30, 30), 1, cv2.LINE_AA)
    if guard is not None:
        cv2.putText(f, guard, (w - 150, h - 9), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (10, 120, 10), 1, cv2.LINE_AA)
    if lost:
        cv2.putText(f, "TARGET LOST", (w // 2 - 110, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.95, (40, 40, 220), 3,
                    cv2.LINE_AA)
    return f


def main():
    os.makedirs("docs", exist_ok=True)
    thr = calibrate_area(P_RING, z_range=(0.5, 1.0), lateral=0.1)

    runs = []
    for label, fn in [
        ("Fixed: partition always on", lambda **k: run_partitioned(
            rel_tau=TAU, **k)),
        ("Switched: area guard", lambda **k: run_switched(
            rel_tau=TAU, rule="area_guard", guard_thresh=thr, **k)),
    ]:
        rec = Recorder(P_RING, OCC, WINDOW, EXPOSURE)
        try:
            lg = fn(P_o=P_RING, cTo_init=CI, cTo_star=CS, lam=LAM, dt=DT,
                    steps=STEPS, min_features=2, feature_fn=rec)
        finally:
            rec.close()
        runs.append((label, lg, rec.frames, rec.info))
        print("%-30s ran %d steps, final |e| %.2e"
              % (label, len(lg["t"]), lg["err"][-1]))

    n = max(len(r[2]) for r in runs)
    W, H = PANEL
    writer = imageio.get_writer(OUT, fps=FPS, quality=8,
                                macro_block_size=8)
    for k in range(n):
        panels = []
        for i, (label, lg, frames, info) in enumerate(runs):
            lost = k >= len(frames)
            idx = min(k, len(frames) - 1)
            nd, sg = info[idx]
            v = (float(np.linalg.norm(lg["v"][idx]))
                 if idx < len(lg["v"]) else 0.0)
            guard = None
            if i == 1:
                guard = "guard: DROP" if sg < thr else "guard: keep"
            colour = (30, 30, 200) if i == 0 else (20, 120, 20)
            panels.append(annotate(frames[idx], label, k, nd, v, guard,
                                   lost, colour))
        gap = np.full((H, 6, 3), 255, np.uint8)
        row = np.hstack([panels[0], gap, panels[1]])
        strip = np.full((32, row.shape[1], 3), 255, np.uint8)
        occ_on = WINDOW[0] <= k <= WINDOW[1]
        cv2.putText(strip, "t=%4.1fs   only 2 of 6 features visible"
                    % (k * DT) if occ_on else "t=%4.1fs   all 6 visible"
                    % (k * DT), (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
                    (20, 20, 20) if not occ_on else (200, 60, 20), 1,
                    cv2.LINE_AA)
        writer.append_data(np.vstack([strip, row]))
    writer.close()
    print("wrote %s  (%d frames, %.0f s at %d fps)" % (OUT, n, n / FPS, FPS))


if __name__ == "__main__":
    main()
