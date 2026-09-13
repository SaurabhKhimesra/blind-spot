"""The four failure cases, with features from the rendered camera.

Geometry is scaled so a marker subtends ~40 px: ring R=0.05 with 15 mm
markers at a working distance of ~0.3 m. Both the numpy and the rendered run
use the SAME cTo and the SAME control code (feature_fn=None vs a renderer),
so the only difference between them is where s comes from.

Occlusion is a real box in the scene, not a mask applied to the answer: the
marker is physically covered, the detector misses it, and the controller sees
fewer features. Collapse is the same target flattening as the numpy study.

    python3 mj_cases.py
"""

import numpy as np

from ibvs_core import (make_pose, rot_z, rot_x, run_ibvs, transform_points,
                       project)
from partitioned import run_partitioned
from truncated import run_truncated
from switched import run_switched, calibrate_area
from mj_scene import RenderedFeatureSource, ring, intrinsics

MARKER_M = 0.030
Z_STAR = 0.60
TAU = 1e-3
WINDOW = (30, 90)

P_RING = ring()
# The ORIGINAL numpy scenario geometry, so the failure modes actually occur.
# An earlier rescaled version (Z=0.30) agreed numpy-vs-rendered but produced
# none of the phenomena: classic showed 1.1x on dropout instead of 28.7x.
CS = make_pose(t=np.array([0.0, 0.0, Z_STAR]))
CI = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))
CS_RET = make_pose(t=np.array([0.0, 0.0, 0.8]))
CI_RET = make_pose(R=rot_z(np.pi)) @ CS_RET


def occluder_boxes(P_o, idx, pad=1.25):
    h = MARKER_M * pad / 2.0
    return [(P_o[i, 0], P_o[i, 1], h, h) for i in idx]


class WindowedFeatures:
    """Clear scene outside the window, occluded scene inside it."""

    def __init__(self, P_o, occlude_idx=(), window=WINDOW, marker_m=MARKER_M):
        self.clear = RenderedFeatureSource(P_o, marker_m=marker_m)
        self.occl = None
        if len(occlude_idx):
            self.occl = RenderedFeatureSource(
                P_o, occluders=occluder_boxes(P_o, occlude_idx),
                marker_m=marker_m)
        self.a, self.b = window
        self.n_calls = 0
        self.n_missed = 0
        self.expected = np.ones(P_o.shape[0], dtype=bool)
        self.occl_idx = list(occlude_idx)

    def __call__(self, k, cTo):
        src = self.clear
        expect = self.expected.copy()
        if self.occl is not None and self.a <= k <= self.b:
            src = self.occl
            expect[self.occl_idx] = False
        s, det = src.features(cTo)
        self.n_calls += 1
        # a detection failure is a marker that SHOULD have been visible and
        # was not. Counted separately from occlusion so the two never blur.
        self.n_missed += int((expect & ~det).sum())
        return s, det

    def close(self):
        self.clear.close()
        if self.occl is not None:
            self.occl.close()


def spike(lg, a=WINDOW[0], b=WINDOW[1]):
    return float(np.linalg.norm(lg["v"][a:b + 1], axis=1).max()
                 / np.linalg.norm(lg["v"][a - 1]))


def controllers(area_thr):
    return [
        ("Classic IBVS", lambda **k: run_ibvs(**k)),
        ("Partitioned (2001)", lambda **k: run_partitioned(**k)),
        ("Adaptive-rank trunc.",
         lambda **k: run_truncated(rel_tau=TAU, **k)),
        ("Both combined", lambda **k: run_partitioned(rel_tau=TAU, **k)),
        ("Switched (area guard)",
         lambda **k: run_switched(rel_tau=TAU, rule="area_guard",
                                  guard_thresh=area_thr, **k)),
    ]


def main():
    # Threshold calibrated on the HEALTHY target at the rendered working
    # distance. sigma carries viewing distance, so the numpy threshold (poses
    # 0.5-1.0 m) does not transfer to a 0.3 m scene.
    area_thr = calibrate_area(P_RING, z_range=(0.5, 1.0), lateral=0.1)
    print("area-guard threshold, healthy 1st pct over 4000 poses in "
          "Z=[0.5,1.0]: %.4e\n" % area_thr)

    cases = [
        ("retreat 180", P_RING, CI_RET, CS_RET, (), 1.0, "converge"),
        ("dropout 3of6", P_RING, CI, CS, (0, 1, 2), 1.0, "spike"),
        ("two features", P_RING, CI, CS, (1, 2, 4, 5), 1.0, "spike"),
        # The collapsed case is ABSENT by measurement, not by omission: see
        # mj_limits.py. Reproducing its verdict needs collapse c <= 0.05, and
        # the smallest renderable collapse with six decodable ArUco markers at
        # this working distance is c = 0.35 even at 2560 px wide.
    ]

    for cname, P_o, ci, cs, occ_idx, _, measure in cases:
        print("=== %s  (judged on %s) ===" % (cname, measure))
        print("%-24s %14s %14s %10s" % ("controller", "numpy", "rendered",
                                        "agree?"))
        minf = 2 if len(occ_idx) == 4 else 3
        for name, run in controllers(area_thr):
            steps = 900
            lg_np = run(P_o=P_o, cTo_init=ci, cTo_star=cs, lam=0.5,
                        steps=steps, min_features=minf,
                        visible_mask_fn=_mask(occ_idx, P_o.shape[0]))
            feats = WindowedFeatures(P_o, occ_idx)
            try:
                lg_mj = run(P_o=P_o, cTo_init=ci, cTo_star=cs, lam=0.5,
                            steps=steps, min_features=minf, feature_fn=feats)
            finally:
                feats.close()
            if measure == "spike":
                a, b = spike(lg_np), spike(lg_mj)
                agree = (a < 2.0) == (b < 2.0)
                print("%-24s %13.1fx %13.1fx %10s"
                      % (name, a, b, "yes" if agree else "NO"))
            else:
                a, b = lg_np["err"][-1], lg_mj["err"][-1]
                ca, cb = a < 1e-4, b < 5e-3
                print("%-24s %14s %14s %10s"
                      % (name, "conv" if ca else "%.1e" % a,
                         "%.1e" % b, "yes" if ca == cb else "NO"))
        print()


def _mask(occ_idx, n):
    if not len(occ_idx):
        return None

    def f(k, s):
        vis = np.ones(n, dtype=bool)
        if WINDOW[0] <= k <= WINDOW[1]:
            vis[list(occ_idx)] = False
        return vis
    return f


if __name__ == "__main__":
    main()
