"""Checks for the packaged guard. The regression checks cover the findings;
these cover the API contract, in particular the things the design
deliberately forbids.

    ros2 run blindspot api_checks
"""

import json
import os
import subprocess
import sys
import tempfile

import numpy as np

from blindspot import (Calibration, FeatureGuard, intrinsics_from_fov,
                       pixels_to_normalised, polygon_sigma)
from blindspot.calibrate import calibrate, load_pose
from blindspot.study.partitioned import polygon_sigma as reference_sigma

PASS, FAIL = "  ok  ", " FAIL "
_r = []


def check(name, cond, detail=""):
    _r.append(bool(cond))
    print("[%s] %s %s" % (PASS if cond else FAIL, name, detail))


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def view(points, cTo):
    P_c = (cTo @ np.hstack([points, np.ones((len(points), 1))]).T).T[:, :3]
    return np.stack([P_c[:, 0] / P_c[:, 2], P_c[:, 1] / P_c[:, 2]], axis=1)


goal = np.eye(4)
goal[2, 3] = 0.6
cal = calibrate(ring(), goal, n_poses=400, seed=0)
guard = FeatureGuard(cal)
pose = np.eye(4)
pose[:3, 3] = [0.02, -0.01, 0.62]

print("--- rule 1: no default threshold ---")
try:
    FeatureGuard(None)
    check("FeatureGuard(None) is refused", False)
except ValueError as e:
    msg = str(e)
    check("FeatureGuard(None) is refused", True)
    check("  and the error explains the in-situ trap",
          "in situ" in msg and "KNOWN-GOOD" in msg)
    check("  and names the calibrate command",
          "ros2 run blindspot calibrate" in msg)
try:
    FeatureGuard({"3": 0.01})
    check("a bare dict is refused", False)
except TypeError:
    check("a bare dict is refused", True)

print("\n--- rule 3: the guard has no truncation tolerance ---")
import blindspot.guard as _g                                   # noqa: E402
src = open(_g.__file__).read()
check("guard.py never mentions rel_tau", "rel_tau" not in src)
check("FeatureGuard takes only a calibration",
      list(FeatureGuard.__init__.__code__.co_varnames[:2]) == ["self",
                                                               "calibration"])
check("Calibration stores no tau", not any(
    "tau" in str(k).lower() for k in cal.metadata))

print("\n--- rule 4: units ---")
fx, fy, cx, cy = intrinsics_from_fov(1920, 1440, 45.0)
check("principal point is (W-1)/2, not W/2", cx == 959.5 and cy == 719.5,
      "(cx=%.1f, cy=%.1f)" % (cx, cy))
s_true = view(ring(), pose)
uv = s_true * np.array([fx, fy]) + np.array([cx, cy])
back = pixels_to_normalised(uv, fx, fy, cx, cy)
check("pixels round-trip to normalised", np.allclose(back, s_true, atol=1e-12))
try:
    pixels_to_normalised(uv, fx)
    check("cx/cy are required, not guessed", False)
except ValueError:
    check("cx/cy are required, not guessed", True)

print("\n--- the signal matches the verified implementation ---")
check("polygon_sigma agrees with partitioned.polygon_sigma",
      abs(polygon_sigma(s_true) - reference_sigma(s_true)) < 1e-15)

print("\n--- decisions ---")
check("healthy view: partition ok", guard.partition_ok(s_true))
flat = ring()
flat[:, 1] *= 0.02
check("collapsed target, all 6 features: partition dropped",
      not guard.partition_ok(view(flat, pose)),
      "(margin %.2fx)" % guard.evaluate(view(flat, pose)).margin)
check("two features: partition dropped",
      not guard.partition_ok(s_true[[0, 3]]))
r = guard.evaluate(s_true)
check("evaluate exposes signal, threshold, margin, count, decision",
      r.signal > 0 and r.threshold > 0 and r.margin > 1
      and r.n_features == 6 and r.decision is True, "(%s)" % r)
check("thresholds are conditioned on the feature count",
      cal.threshold_for(3) < cal.threshold_for(6),
      "(N=3 %.3e < N=6 %.3e)"
      % (cal.threshold_for(3), cal.threshold_for(6)))
try:
    guard.evaluate(np.zeros((6, 3)))
    check("a wrong-shaped array is refused", False)
except ValueError:
    check("a wrong-shaped array is refused", True)

print("\n--- round-trip through a calibration file ---")
with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "cal.json")
    with open(p, "w") as f:
        json.dump(cal.to_dict(), f)
    g2 = FeatureGuard.load(p)
    check("load() reproduces the same decisions",
          g2.evaluate(s_true).signal == r.signal
          and g2.calibration.thresholds == cal.thresholds)
    bad = os.path.join(d, "bad.json")
    with open(bad, "w") as f:
        json.dump({"thresholds": {"6": 1.0}}, f)
    try:
        FeatureGuard.load(bad)
        check("a file without the schema tag is refused", False)
    except ValueError:
        check("a file without the schema tag is refused", True)

    print("\n--- the calibrate command runs ---")
    geo, gp, out = (os.path.join(d, x) for x in
                    ("geo.json", "goal.json", "out.json"))
    with open(geo, "w") as f:
        json.dump({"points": ring().tolist()}, f)
    with open(gp, "w") as f:
        json.dump({"cTo": goal.tolist()}, f)
    cp = subprocess.run(
        [sys.executable, "-m", "blindspot.calibrate", "--geometry", geo,
         "--goal-pose", gp, "--poses", "200", "-o", out],
        capture_output=True, text=True,
        cwd=tempfile.gettempdir())
    check("the calibrate command exits 0", cp.returncode == 0,
          cp.stderr.strip().splitlines()[-1] if cp.returncode else "")
    check("  and writes a loadable calibration",
          os.path.exists(out) and FeatureGuard.load(out) is not None)
    check("  and warns that the target must have been healthy",
          "healthy" in cp.stdout)

print("\n--- rule 2: controllers are out of the core ---")
import blindspot                                               # noqa: E402
check("blindspot exports no controller",
      not any(n.startswith("run_") for n in dir(blindspot)))
from blindspot import reference                                # noqa: E402
check("blindspot.reference has them, labelled as examples",
      all(hasattr(reference, n) for n in
          ("run_ibvs", "run_partitioned", "run_truncated", "run_switched")))
check("  and its docstring says they are not the product",
      "EXAMPLES, not the product" in reference.__doc__)

print("\n--- ROS bridge: ordering, units, message shapes ---")
from blindspot.ros_bridge import (detection_center, detection_label,  # noqa: E402
                                  features_to_normalised,
                                  intrinsics_from_camera_info,
                                  order_features, ordering_is_suspect)

sq = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
bow = sq[[0, 2, 1, 3]]
check("a sane ordering is not flagged", not ordering_is_suspect(sq))
check("a self-crossing ordering IS flagged", ordering_is_suspect(bow),
      "(shoelace area collapses, which would fire the guard for the wrong "
      "reason)")
o_id, _, how = order_features(bow, ids=[0, 2, 1, 3])
check("ordering by detection id repairs it",
      how == "id" and not ordering_is_suspect(o_id))
o_ang, _, how2 = order_features(bow, ids=None)
check("without ids it falls back to an angular sort",
      how2 == "angular" and not ordering_is_suspect(o_ang))

fx_i, fy_i, cx_i, cy_i, src_i = intrinsics_from_camera_info(
    [1738.2, 0, 959.5, 0, 1738.2, 719.5, 0, 0, 1], 1920, 1440)
check("a calibrated camera_info is used as-is",
      src_i == "camera_info" and cx_i == 959.5 and fx_i == 1738.2)
_, _, cx_u, cy_u, src_u = intrinsics_from_camera_info([0] * 9, 1920, 1440)
check("an uncalibrated one reports so and offers (W-1)/2",
      src_u == "uncalibrated" and cx_u == 959.5 and cy_u == 719.5)

s_ros, _, _ = features_to_normalised(uv, fx, fy, cx, cy,
                                     ids=list(range(6)))
check("bridge reproduces the guard decision from pixels",
      guard.evaluate(s_ros).decision == guard.evaluate(s_true).decision
      and abs(guard.evaluate(s_ros).signal
              - guard.evaluate(s_true).signal) < 1e-12)


class _P:
    x, y = 12.0, 34.0


class _C4:
    position = _P()


class _B4:
    center = _C4()


class _D4:
    bbox = _B4()
    id = "3"


class _C3:
    x, y = 5.0, 6.0


class _B3:
    center = _C3()


class _H:
    class_id = "7"


class _R:
    hypothesis = _H()


class _D3:
    bbox = _B3()
    id = ""
    results = [_R()]


check("reads vision_msgs 4.x Detection2D (Pose2D.position)",
      detection_center(_D4()) == (12.0, 34.0) and detection_label(_D4()) == "3")
check("reads the older shape and the hypothesis class_id",
      detection_center(_D3()) == (5.0, 6.0) and detection_label(_D3()) == "7")

print("\n%d/%d passed" % (sum(_r), len(_r)))
raise SystemExit(0 if all(_r) else 1)
