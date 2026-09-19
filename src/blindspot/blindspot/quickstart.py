"""Calibrates the shipped ring target and shows the guard firing on a
degenerate view and staying quiet on a healthy one.

    ros2 run blindspot quickstart
"""

import numpy as np

from blindspot import FeatureGuard, intrinsics_from_fov, pixels_to_normalised
from blindspot.calibrate import calibrate


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def view(points, cTo):
    P_c = (cTo @ np.hstack([points, np.ones((len(points), 1))]).T).T[:, :3]
    return np.stack([P_c[:, 0] / P_c[:, 2], P_c[:, 1] / P_c[:, 2]], axis=1)


goal = np.eye(4)
goal[2, 3] = 0.6                      # 0.6 m straight on

print("Calibrating on the KNOWN-GOOD ring target (600 poses for speed)...")
cal = calibrate(ring(), goal, n_poses=600, percentile=1.0, seed=0)
guard = FeatureGuard(cal)
print("thresholds by visible count: %s\n"
      % {n: "%.3e" % v for n, v in sorted(cal.thresholds.items())})

pose = np.eye(4)
pose[:3, 3] = [0.02, -0.01, 0.62]

print("1. healthy view, all 6 features")
print("   ", guard.evaluate(view(ring(), pose)))

print("2. occluded, only 2 features survive")
print("   ", guard.evaluate(view(ring(), pose)[[0, 3]]))

print("3. target geometry collapsed toward a line, still all 6 features")
flat = ring()
flat[:, 1] *= 0.02
print("   ", guard.evaluate(view(flat, pose)))

print("\n4. same thing from pixel coordinates (1920x1440, 45 deg vertical FOV)")
fx, fy, cx, cy = intrinsics_from_fov(1920, 1440, 45.0)
uv = view(ring(), pose) * np.array([fx, fy]) + np.array([cx, cy])
print("   principal point is (W-1)/2 = %.1f, not W/2 = %.1f" % (cx, 1920 / 2))
print("   ", guard.evaluate(pixels_to_normalised(uv, fx, fy, cx, cy)))

print("\n5. the API refuses to run without a calibration")
try:
    FeatureGuard(None)
except ValueError as e:
    print("   ValueError:", str(e).splitlines()[0])

print("\nUse it in your own loop:")
print("    if guard.partition_ok(s_visible):  v = my_partitioned_control(...)")
print("    else:                              v = my_plain_control(...)")
