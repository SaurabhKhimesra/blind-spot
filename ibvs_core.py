"""
Core IBVS simulation: virtual perspective camera, 4-point target, classic
image-based visual servo control law, and a set of candidate health monitors.

Pure numpy. No simulator required.

Conventions
-----------
Points are expressed in the camera frame. Perspective projection is
    x = X/Z,  y = Y/Z        (normalised image coordinates, metres)

Camera velocity is the 6-vector v = [vx, vy, vz, wx, wy, wz] expressed in
the camera frame. A point P in the camera frame then moves as
    Pdot = -v_lin - w x P
which is the standard eye-in-hand kinematics.

The interaction matrix for one point feature (x, y) at depth Z is
    Lx = [-1/Z,     0,  x/Z,     x*y, -(1+x^2),   y]
    Ly = [    0, -1/Z,  y/Z, (1+y^2),     -x*y,  -x]
(Chaumette & Hutchinson, Visual servo control part I, 2006)
"""

import numpy as np


# ----------------------------------------------------------------------
# SE(3) utilities
# ----------------------------------------------------------------------

def skew(w):
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy],
                     [wz, 0.0, -wx],
                     [-wy, wx, 0.0]])


def se3_exp(xi):
    """Exponential map se(3) -> SE(3). xi = [v(3), w(3)]."""
    v = np.asarray(xi[:3], dtype=float)
    w = np.asarray(xi[3:], dtype=float)
    theta = np.linalg.norm(w)
    T = np.eye(4)
    if theta < 1e-12:
        T[:3, :3] = np.eye(3)
        T[:3, 3] = v
        return T
    W = skew(w / theta)
    th = theta
    R = np.eye(3) + np.sin(th) * W + (1.0 - np.cos(th)) * (W @ W)
    V = (np.eye(3)
         + ((1.0 - np.cos(th)) / th) * W
         + ((th - np.sin(th)) / th) * (W @ W))
    T[:3, :3] = R
    T[:3, 3] = V @ (v / theta) * theta / theta * theta  # = V @ v  (kept explicit)
    T[:3, 3] = V @ v
    return T


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def make_pose(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


# ----------------------------------------------------------------------
# Camera and features
# ----------------------------------------------------------------------

def transform_points(cTo, P_o):
    """Express object points (N,3) given in the object frame in the camera frame."""
    P_h = np.hstack([P_o, np.ones((P_o.shape[0], 1))])
    return (cTo @ P_h.T).T[:, :3]


def project(P_c):
    """Perspective projection to normalised image coords. Returns (N,2)."""
    Z = P_c[:, 2]
    return np.stack([P_c[:, 0] / Z, P_c[:, 1] / Z], axis=1)


def interaction_matrix(s, Z):
    """Stacked interaction matrix for N point features.

    s : (N,2) normalised image coordinates
    Z : (N,)  depths in the camera frame
    returns (2N, 6)
    """
    N = s.shape[0]
    L = np.zeros((2 * N, 6))
    for i in range(N):
        x, y = s[i]
        Zi = Z[i]
        L[2 * i] = [-1.0 / Zi, 0.0, x / Zi, x * y, -(1.0 + x * x), y]
        L[2 * i + 1] = [0.0, -1.0 / Zi, y / Zi, 1.0 + y * y, -x * y, -x]
    return L


# ----------------------------------------------------------------------
# Health monitors: candidate signals for "the servo has gone blind"
# ----------------------------------------------------------------------

def sigma_6(L):
    """The 6th singular value of L, with the spectrum padded to length 6.

    This is the health metric. L is (2N,6), so it has min(2N,6) singular
    values. Padding to 6 with zeros means that when the camera has fewer
    independent image constraints than degrees of freedom, sigma_6 is
    EXACTLY zero - which is the correct reading, because those DOF are
    unobservable rather than merely ill-conditioned.

    Note what the naive S[-1] does instead: for N=2 it returns the 4th
    singular value, a healthy-looking number, because it silently compares
    matrices of different shapes and cannot see a null space at all.
    """
    S = np.linalg.svd(L, compute_uv=False)
    padded = np.zeros(6)
    padded[:S.shape[0]] = S
    return float(padded.min())


def monitors(L, e, v, dt):
    """Return a dict of candidate health signals for the current step.

    L : (2N,6) interaction matrix
    e : (2N,)  feature error  s - s*
    v : (6,)   commanded camera velocity
    """
    U, S, Vt = np.linalg.svd(L, full_matrices=False)

    sigma_min = S[-1]          # shape dependent, kept only for continuity
    s6 = sigma_6(L)            # the health metric: 0 on a real null space
    sigma_max = S[0]
    cond = sigma_max / max(sigma_min, 1e-16)

    # How much of the task error lies in directions the camera can actually
    # shrink quickly. e_hat projected onto left singular vectors weighted by
    # the inverse singular values tells you the control effort needed.
    e_norm = np.linalg.norm(e)
    if e_norm > 1e-12:
        coeffs = U.T @ (e / e_norm)          # (6,) components of unit error
        # effort = || L^+ e_hat ||: large when error sits in weak directions
        effort = np.linalg.norm(coeffs / np.maximum(S, 1e-16))
        # fraction of unit error energy in the weakest direction
        weak_frac = float(coeffs[-1] ** 2)
    else:
        effort = 0.0
        weak_frac = 0.0

    # Predicted first-order error decay vs actual commanded motion.
    # edot = L v. cos_align near -1 means the command drives error straight down.
    edot = L @ v
    if e_norm > 1e-12 and np.linalg.norm(edot) > 1e-12:
        cos_align = float(edot @ e / (np.linalg.norm(edot) * e_norm))
    else:
        cos_align = 0.0

    # Retreat indicator: optical-axis translation relative to total command.
    v_norm = np.linalg.norm(v)
    retreat = float(v[2] / v_norm) if v_norm > 1e-12 else 0.0

    return dict(sigma_min=float(sigma_min),
                sigma_6=s6,
                sigma_max=float(sigma_max),
                cond=float(cond),
                effort=float(effort),
                weak_frac=weak_frac,
                cos_align=cos_align,
                retreat=retreat)


# ----------------------------------------------------------------------
# IBVS loop
# ----------------------------------------------------------------------

def run_ibvs(P_o, cTo_init, cTo_star, lam=0.5, dt=0.033, steps=900,
             depth_mode="true", visible_mask_fn=None, max_range=50.0,
             min_features=3, feature_fn=None):
    """Run a classic IBVS servo and log everything.

    depth_mode : 'true'      use the true current depth in L  (L at current)
                 'desired'   use the depth at the desired pose (L*)
                 'constant'  use the desired-pose depth of point 0 for all
    visible_mask_fn : optional fn(step, s) -> boolean array of visible features,
                      used to simulate occlusion.
    min_features : if fewer than this many features are visible, fall back to
                   using all of them. 3 is the default because 3 points give
                   6 equations for 6 DOF. Set to 2 to study the two-feature
                   regime, where the system is genuinely underdetermined.
    """
    P_star_c = transform_points(cTo_star, P_o)
    s_star = project(P_star_c)
    Z_star = P_star_c[:, 2]

    cTo = cTo_init.copy()
    log = {k: [] for k in ["t", "err", "v", "sigma_min", "sigma_6", "cond",
                           "effort", "weak_frac", "cos_align", "retreat",
                           "cam_pos", "n_vis", "s"]}

    for k in range(steps):
        P_c = transform_points(cTo, P_o)
        Z = P_c[:, 2]
        if np.any(Z <= 1e-3):                 # target behind/at the camera
            break
        if feature_fn is None:
            s = project(P_c)
            detected = np.ones(P_o.shape[0], dtype=bool)
        else:
            # rendered-camera feature source: same control law, same cTo,
            # only the origin of s changes. Depth stays ground-truth so that
            # exactly one variable moves.
            s, detected = feature_fn(k, cTo)

        vis = np.ones(P_o.shape[0], dtype=bool)
        if visible_mask_fn is not None:
            vis = visible_mask_fn(k, s)
        vis = vis & detected
        if vis.sum() < min_features:
            # fall back to everything AVAILABLE, which with a real detector is
            # what was detected - never to all features, whose undetected
            # entries are NaN.
            vis = detected.copy()
        if vis.sum() < min_features:          # floor on usable features
            vis = np.ones(P_o.shape[0], dtype=bool) & detected
        if vis.sum() < 2:
            # Target lost: fewer than two features available, so no control
            # law can be formed. With a real detector this is a physical
            # outcome (the target left the frame or became too small to
            # detect), not an error to paper over. The numpy path never
            # reaches here because `detected` is all-ones there.
            break

        # scored over DETECTED features only: an undetected marker has NaN
        # coordinates and would poison the norm. `detected` is all-ones on
        # the numpy path, so this is identical there.
        e_full = (s - s_star)[detected].reshape(-1)
        e = (s[vis] - s_star[vis]).reshape(-1)

        if depth_mode == "true":
            Zu = Z[vis]
            Lu = interaction_matrix(s[vis], Zu)
        elif depth_mode == "desired":
            Lu = interaction_matrix(s_star[vis], Z_star[vis])
        elif depth_mode == "constant":
            Zc = np.full(vis.sum(), Z_star[0])
            Lu = interaction_matrix(s[vis], Zc)
        else:
            raise ValueError(depth_mode)

        v = -lam * np.linalg.pinv(Lu) @ e

        m = monitors(Lu, e, v, dt)

        cam_pos = -cTo[:3, :3].T @ cTo[:3, 3]   # camera position in object frame
        log["t"].append(k * dt)
        log["err"].append(float(np.linalg.norm(e_full)))
        log["v"].append(v.copy())
        log["cam_pos"].append(cam_pos.copy())
        log["n_vis"].append(int(vis.sum()))
        log["s"].append(s.copy())
        for key in ["sigma_min", "sigma_6", "cond", "effort", "weak_frac",
                    "cos_align", "retreat"]:
            log[key].append(m[key])

        # integrate: camera moves with v, so object pose in camera frame
        # transforms by exp(-v dt)
        cTo = se3_exp(-v * dt) @ cTo

        if np.linalg.norm(cam_pos) > max_range:
            break

    for k in log:
        log[k] = np.array(log[k])
    return log


# ----------------------------------------------------------------------
# Scenarios
# ----------------------------------------------------------------------

def square_target(half=0.1):
    """Four coplanar points forming a square in the object frame (z=0)."""
    return np.array([[-half, -half, 0.0],
                     [half, -half, 0.0],
                     [half, half, 0.0],
                     [-half, half, 0.0]])


def scenario_camera_retreat(angle_deg=180.0, Zd=0.8):
    """The classic IBVS failure (Chaumette 1998).

    Desired pose: camera straight on, distance Zd from the target plane.
    Initial pose: identical, but rotated about the optical axis by angle_deg.
    With L computed at the current features, the servo translates backwards
    along the optical axis instead of simply rotating.
    """
    P_o = square_target()
    cTo_star = make_pose(R=np.eye(3), t=np.array([0.0, 0.0, Zd]))
    a = np.deg2rad(angle_deg)
    cTo_init = make_pose(R=rot_z(a), t=np.array([0.0, 0.0, Zd]))
    return P_o, cTo_init, cTo_star


def scenario_final_approach(Zd=0.05, Z0=0.35, lateral=0.02, tilt_deg=4.0):
    """Insertion-like final approach: converge to a very short standoff.

    Near the end the target nearly fills the view, depth becomes weakly
    observable, and the interaction matrix conditioning degrades.
    """
    P_o = square_target(half=0.01)          # small target, 2 cm across
    cTo_star = make_pose(R=np.eye(3), t=np.array([0.0, 0.0, Zd]))
    R0 = rot_x(np.deg2rad(tilt_deg)) @ rot_y(np.deg2rad(tilt_deg))
    cTo_init = make_pose(R=R0, t=np.array([lateral, -lateral, Z0]))
    return P_o, cTo_init, cTo_star
