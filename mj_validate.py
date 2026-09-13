"""Gate for the rendered feature source: does it agree with project()?

Nothing downstream is believed until this passes. A perception result and a
port bug are indistinguishable otherwise.

    python3 mj_validate.py
"""

import numpy as np

from ibvs_core import (make_pose, rot_z, rot_x, rot_y, transform_points,
                       project)
from mj_scene import RenderedFeatureSource, ring, intrinsics, WIDTH, HEIGHT

rng = np.random.default_rng(0)
P_o = ring()
src = RenderedFeatureSource(P_o)
fx, fy, cx, cy = intrinsics()

print("Rendered feature source vs project()")
print("  resolution %dx%d, fovy 45 deg, fx = %.2f px" % (WIDTH, HEIGHT, fx))
print("  marker centre = mean of 4 detected ArUco corners\n")

poses = []
for _ in range(60):
    R = (rot_z(rng.uniform(-np.pi, np.pi))
         @ rot_x(rng.uniform(-0.25, 0.25))
         @ rot_y(rng.uniform(-0.25, 0.25)))
    t = np.array([rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05),
                  rng.uniform(0.45, 0.95)])
    poses.append(make_pose(R=R, t=t))

err_px, n_missing, n_pose_full = [], 0, 0
for cTo in poses:
    s_true = project(transform_points(cTo, P_o))
    s_det, vis = src.features(cTo)
    n_missing += int((~vis).sum())
    if vis.all():
        n_pose_full += 1
    if vis.any():
        d = (s_det[vis] - s_true[vis]) * np.array([fx, fy])
        err_px.extend(np.linalg.norm(d, axis=1).tolist())

err_px = np.array(err_px)
print("  poses sampled            : %d" % len(poses))
print("  poses with all 6 detected: %d" % n_pose_full)
print("  markers missed           : %d of %d" % (n_missing, 6 * len(poses)))
print("  feature agreement with project():")
print("     mean   %.4f px" % err_px.mean())
print("     median %.4f px" % np.median(err_px))
print("     p95    %.4f px" % np.percentile(err_px, 95))
print("     max    %.4f px" % err_px.max())
print("     mean in normalised units: %.3e" % (err_px.mean() / fx))

ok = err_px.mean() < 1.0 and n_missing == 0
print("\n  %s: rendered features track project() to %.3f px mean"
      % ("PASS" if ok else "FAIL", err_px.mean()))
raise SystemExit(0 if ok else 1)
