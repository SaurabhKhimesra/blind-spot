"""Calibrate guard thresholds on a KNOWN-GOOD target.

    python -m blindspot.calibrate --geometry target.json \\
        --goal-pose pose.json --poses 4000 --percentile 1 -o my_target.json

--geometry  JSON: {"points": [[x, y, z], ...]} in the target frame, metres.
--goal-pose JSON: {"cTo": [[4x4]]} or {"R": [[3x3]], "t": [x, y, z]}, the
            transform taking target-frame points into the camera frame at the
            desired pose. Its standoff sets the sampling envelope.

The target you point this at must be HEALTHY. Calibrating on a degenerate one
makes the degeneracy the norm: the threshold comes out below the degenerate
target's everyday value, the guard never fires, and nothing looks wrong.
"""

import argparse
import json
import sys

import numpy as np

from .guard import Calibration, polygon_sigma, SCHEMA


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def load_pose(d):
    if "cTo" in d:
        return np.asarray(d["cTo"], dtype=float)
    T = np.eye(4)
    T[:3, :3] = np.asarray(d.get("R", np.eye(3)), dtype=float)
    T[:3, 3] = np.asarray(d["t"], dtype=float)
    return T


def calibrate(points, goal_pose, n_poses=4000, percentile=1.0, seed=0,
              z_range=None, lateral=None, tilt=0.3):
    """Per-feature-count thresholds from healthy poses around the goal.

    Poses are drawn as
        translation : x, y ~ U(-lateral, lateral),  z ~ U(*z_range)
        rotation    : Rz ~ U(-pi, pi), then Rx, Ry ~ U(-tilt, tilt)
    with defaults scaled off the goal standoff. Poses putting any feature
    behind the camera are rejected and redrawn. For each count N the visible
    subset is drawn at random too, because a subset's statistic is bounded
    above by the full set's.
    """
    points = np.asarray(points, dtype=float)
    n_pts = points.shape[0]
    if n_pts < 3:
        raise ValueError("need at least 3 target points, got %d" % n_pts)
    Zg = float(goal_pose[2, 3])
    if Zg <= 0:
        raise ValueError("goal pose has non-positive standoff (%.3f)" % Zg)
    z_range = z_range or (0.55 * Zg, 1.6 * Zg)
    lateral = lateral if lateral is not None else 0.17 * Zg

    rng = np.random.default_rng(seed)
    P_h = np.hstack([points, np.ones((n_pts, 1))])
    thresholds, samples = {}, {}
    for n_vis in range(3, n_pts + 1):
        vals = []
        while len(vals) < n_poses:
            R = (_rot_z(rng.uniform(-np.pi, np.pi))
                 @ _rot_x(rng.uniform(-tilt, tilt))
                 @ _rot_y(rng.uniform(-tilt, tilt)))
            t = np.array([rng.uniform(-lateral, lateral),
                          rng.uniform(-lateral, lateral),
                          rng.uniform(*z_range)])
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R, t
            P_c = (T @ P_h.T).T[:, :3]
            if np.any(P_c[:, 2] <= 1e-6):
                continue
            s = np.stack([P_c[:, 0] / P_c[:, 2], P_c[:, 1] / P_c[:, 2]],
                         axis=1)
            if n_vis < n_pts:
                idx = np.sort(rng.choice(n_pts, n_vis, replace=False))
                s = s[idx]
            vals.append(polygon_sigma(s))
        thresholds[n_vis] = float(np.percentile(np.array(vals), percentile))
        samples[n_vis] = len(vals)
    # Below three features there is no polygon, so the guard always fires.
    thresholds[2] = 0.0
    meta = dict(n_points=n_pts, n_poses=n_poses, percentile=percentile,
                seed=seed, z_range=list(z_range), lateral=lateral, tilt=tilt,
                units="normalised image coordinates (x=X/Z, y=Y/Z, y down)",
                goal_standoff_m=Zg, samples_per_count=samples)
    return Calibration(thresholds, meta)


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m blindspot.calibrate",
        description="Calibrate guard thresholds on a KNOWN-GOOD target.")
    ap.add_argument("--geometry", required=True,
                    help='JSON {"points": [[x,y,z], ...]}, metres')
    ap.add_argument("--goal-pose", required=True,
                    help='JSON {"cTo": 4x4} or {"R": 3x3, "t": [x,y,z]}')
    ap.add_argument("--poses", type=int, default=4000)
    ap.add_argument("--percentile", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args(argv)

    with open(a.geometry) as f:
        pts = json.load(f)["points"]
    with open(a.goal_pose) as f:
        goal = load_pose(json.load(f))

    cal = calibrate(pts, goal, a.poses, a.percentile, a.seed)
    with open(a.out, "w") as f:
        json.dump(cal.to_dict(), f, indent=2)
        f.write("\n")
    print("wrote %s (%s)" % (a.out, SCHEMA))
    print("thresholds by visible feature count:")
    for n in sorted(cal.thresholds):
        print("   N=%-3d %.6e" % (n, cal.thresholds[n]))
    print("\nThis is only valid if the target you calibrated on was healthy.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
