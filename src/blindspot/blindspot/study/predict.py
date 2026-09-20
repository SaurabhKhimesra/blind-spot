"""Forward prediction of the guard signal: how far ahead can it be trusted?

s_next = s + L v dt predicts the degeneracy the CAMERA causes, not the
degeneracy the WORLD causes. It should see a collapse the camera is driving
itself into, and it should be blind to an occluder arriving, because that is
exogenous. That contrast is the result, so it is measured, not assumed.

Depth is propagated too, from the same kinematics: for a point in the camera
frame, Pdot = -v - w x P, so
    Zdot = -vz - Z (wx y - wy x)
Holding Z fixed instead is cruder and is measured alongside.

A library of functions: the forward-prediction section of the regression
checks exercises it.
"""

import numpy as np

from blindspot.study.ibvs_core import (interaction_matrix, transform_points, project,
                                       se3_exp)


def roll_forward(s, Z, v, dt, K, fixed_depth=False):
    """Iterate s_{j+1} = s_j + L(s_j, Z_j) v dt, K times, v held constant."""
    s, Z = s.copy().astype(float), Z.copy().astype(float)
    for _ in range(K):
        if np.any(Z <= 1e-3) or not np.all(np.isfinite(s)):
            return None, None          # rolled through the camera plane
        L = interaction_matrix(s, Z)
        s = s + (L @ v).reshape(-1, 2) * dt
        if not fixed_depth:
            x, y = s[:, 0], s[:, 1]
            Zdot = -v[2] - Z * (v[3] * y - v[4] * x)
            Z = Z + Zdot * dt
    if not np.all(np.isfinite(s)):
        return None, None
    return s, Z


def truth(cTo, P_o, v, dt, K):
    """Where the features actually are after K steps of the same v."""
    c = cTo.copy()
    for _ in range(K):
        c = se3_exp(-v * dt) @ c
    P_c = transform_points(c, P_o)
    return project(P_c), P_c[:, 2]


def steps_to_fire(s, Z, v, dt, thresh, horizon=20, sigma_fn=None):
    """First K within the horizon at which the guard statistic would trip.

    Returns 0 if it is already below threshold, an int 1..horizon if the
    current heading drives it below within the horizon, or None if not.
    The horizon is the measured trust limit: at |v| > 1.5 the predicted
    statistic is within 4.9% (median) / 9.7% (p95) of the threshold at K=20,
    against a guard margin of 13-62%, so 20 steps is usable and 50 is not.
    """
    from blindspot.study.partitioned import polygon_sigma
    f = sigma_fn or polygon_sigma
    if f(s) < thresh:
        return 0
    sk, Zk = s.copy().astype(float), Z.copy().astype(float)
    for K in range(1, horizon + 1):
        sk, Zk = roll_forward(sk, Zk, v, dt, 1)
        if sk is None:
            return None
        if f(sk) < thresh:
            return K
    return None
