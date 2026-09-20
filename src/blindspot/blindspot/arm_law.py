"""The eye-in-hand control law and perception shared by the arm nodes.

No ROS imports and no OpenCV: the Gazebo nodes in blindspot_arm import it,
and the regression checks import the same module, so the checks exercise the
exact law the arms run rather than a transcription of it. The marker detector
lives in blindspot_arm/detect.py.
"""

import numpy as np

R_TARGET = 0.05          # fiducial ring radius on the panel, metres
Z_DESIRED = 0.26         # standoff the servo drives to
LAM = 0.6
TAU = 1e-3
DT = 0.1

# Intrinsics come from the sensor definition in the URDF, not from
# camera_info. The gz topic /left_wrist_camera_info exists but nothing arrives
# on the ROS side, and waiting on it silently stalled the whole servo loop:
# step() returned early, so the HUD read 0/6 features and neither arm moved.
# Principal point is (W-1)/2, the convention used throughout this repo.
IMG_W, IMG_H, HFOV = 640, 480, 1.0472
_FX = (IMG_W / 2.0) / np.tan(HFOV / 2.0)
K_DEFAULT = (_FX, _FX, (IMG_W - 1) / 2.0, (IMG_H - 1) / 2.0)

def order_by_angle(pts):
    c = pts.mean(axis=0)
    return pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]


def best_cyclic_match(s, s_star):
    """Rotate s_star to the cyclic shift that actually pairs with s.

    Ordering six identical dots by angle fixes their ORDER but not where the
    order starts, so a rotated hexagon can pair every dot with the wrong
    target. The arm then chases a bogus goal: measured, one cell converged to
    |e| 0.090 while its twin diverged to 0.530 running identical code. Picking
    the shift with the smallest residual removes the ambiguity.
    """
    best, best_err = s_star, np.inf
    for k in range(len(s_star)):
        cand = np.roll(s_star, k, axis=0)
        err = np.linalg.norm(s - cand)
        if err < best_err:
            best, best_err = cand, err
    return best


def interaction(s, Z):
    L = np.zeros((2 * len(s), 6))
    for i, (x, y) in enumerate(s):
        L[2 * i] = [-1 / Z, 0, x / Z, x * y, -(1 + x * x), y]
        L[2 * i + 1] = [0, -1 / Z, y / Z, 1 + y * y, -x * y, -x]
    return L


def pinv_trunc(L, tau):
    U, S, Vt = np.linalg.svd(L, full_matrices=False)
    keep = S > tau * S[0]
    Si = np.where(keep, 1.0 / np.where(S > 0, S, 1.0), 0.0)
    return (Vt.T * Si) @ U.T


def poly_sigma(s):
    x, y = s[:, 0], s[:, 1]
    return float(np.sqrt(max(0.5 * abs(np.dot(x, np.roll(y, -1))
                                       - np.dot(y, np.roll(x, -1))), 1e-12)))


def ibvs_twist(s, s_star, use_partition):
    """Camera twist. Partitioned when the partition is safe, plain otherwise."""
    e = (s - s_star).reshape(-1)
    L = interaction(s, Z_DESIRED)
    if use_partition and len(s) >= 3:
        sig, sig_star = poly_sigma(s), poly_sigma(s_star)
        vz = 0.6 * np.log(sig_star / max(sig, 1e-9))
        al = np.arctan2(s[1, 1] - s[0, 1], s[1, 0] - s[0, 0])
        al_s = np.arctan2(s_star[1, 1] - s_star[0, 1],
                          s_star[1, 0] - s_star[0, 0])
        wz = 0.6 * ((al - al_s + np.pi) % (2 * np.pi) - np.pi)
        xy, z = [0, 1, 3, 4], [2, 5]
        v_xy = -pinv_trunc(L[:, xy], TAU) @ (LAM * e + L[:, z] @ [vz, wz])
        v = np.zeros(6)
        v[xy], v[2], v[5] = v_xy, vz, wz
        return v
    return -LAM * pinv_trunc(L, TAU) @ e
