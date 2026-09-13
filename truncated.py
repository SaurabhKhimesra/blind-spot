"""Gate test for the successor idea: adaptive-rank IBVS.

Partitioned IBVS (2001) buys robustness by removing two FIXED axes (vz, wz)
from the pseudo-inverse and driving them from hand-designed image features
(polygon area, line angle). That works, but it only works for the two DOF
somebody found robust substitute features for, and the choice was fixed once
and never adapts.

The generalisation: don't remove a named axis, remove the worst-conditioned
DIRECTION, picked from the data at every step. That is a truncated SVD
pseudo-inverse with an adaptive rank:

    L = U S V^T,   L+_tau = V diag(1/s_i if s_i > tau*s_max else 0) U^T

No substitute features required, nothing hand-designed per DOF, and it adapts
to whatever degeneracy actually shows up rather than the one anticipated in
2001.

The question this file answers: does it match partitioned IBVS on robustness
without partitioned's hand-built machinery, and what does it cost in
convergence?
"""

import numpy as np

from ibvs_core import (se3_exp, transform_points, project, interaction_matrix)


def pinv_truncated(L, rel_tau):
    """Pseudo-inverse with singular directions below rel_tau*s_max dropped.

    Returns the inverse and the retained rank.
    """
    U, S, Vt = np.linalg.svd(L, full_matrices=False)
    keep = S > rel_tau * S[0]
    if not keep.any():
        keep[0] = True
    S_inv = np.where(keep, 1.0 / np.where(S > 0, S, 1.0), 0.0)
    return (Vt.T * S_inv) @ U.T, int(keep.sum())


def run_truncated(P_o, cTo_init, cTo_star, rel_tau=1e-2, lam=0.5, dt=0.033,
                  steps=1500, visible_mask_fn=None, max_range=50.0,
                  min_features=3, feature_fn=None):
    P_star_c = transform_points(cTo_star, P_o)
    s_star_all = project(P_star_c)

    cTo = cTo_init.copy()
    log = {k: [] for k in ["t", "err", "v", "cam_pos", "n_vis", "rank", "cond"]}

    for k in range(steps):
        P_c = transform_points(cTo, P_o)
        Z = P_c[:, 2]
        if np.any(Z <= 1e-3):
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
        if vis.sum() < min_features:
            vis = np.ones(P_o.shape[0], dtype=bool) & detected
        if vis.sum() < 2:
            # Target lost: fewer than two features available, so no control
            # law can be formed. With a real detector this is a physical
            # outcome (the target left the frame or became too small to
            # detect), not an error to paper over. The numpy path never
            # reaches here because `detected` is all-ones there.
            break

        e = (s[vis] - s_star_all[vis]).reshape(-1)
        L = interaction_matrix(s[vis], Z[vis])

        Linv, rank = pinv_truncated(L, rel_tau)
        v = -lam * Linv @ e

        S = np.linalg.svd(L, compute_uv=False)
        cam_pos = -cTo[:3, :3].T @ cTo[:3, 3]

        log["t"].append(k * dt)
        log["err"].append(
            float(np.linalg.norm((s - s_star_all)[detected].reshape(-1))))
        log["v"].append(v.copy())
        log["cam_pos"].append(cam_pos.copy())
        log["n_vis"].append(int(vis.sum()))
        log["rank"].append(rank)
        log["cond"].append(float(S[0] / max(S[-1], 1e-16)))

        cTo = se3_exp(-v * dt) @ cTo
        if np.linalg.norm(cam_pos) > max_range:
            break

    for key in log:
        log[key] = np.array(log[key])
    return log
