"""Rung 3: does the published fix for camera retreat also survive feature loss?

Partitioned IBVS (Corke & Hutchinson, "A new partitioned approach to
image-based visual servo control", IEEE T-RA 2001) removes camera retreat by
taking the two optical-axis degrees of freedom OUT of the pseudo-inverse and
driving them from direct image measurements instead:

    vz  from  sigma = sqrt(area of the feature polygon)
    wz  from  alpha = orientation of a line between two features

The remaining four DOF come from the reduced interaction matrix. Because vz
is no longer produced by inverting L, the retreat pathology disappears.

The question here is whether that helps when features go MISSING, which is a
different failure: not the structure of the control law, but the absence of
information. Implemented fairly - the desired-value references sigma* and
alpha* are recomputed from the same visible subset, which is what a competent
engineer would do - so this is not a strawman.
"""

import numpy as np

from blindspot.study.ibvs_core import (se3_exp, transform_points, project,
                                       interaction_matrix)
from blindspot.study.truncated import pinv_truncated

Z_COLS = [2, 5]           # vz, wz
XY_COLS = [0, 1, 3, 4]    # vx, vy, wx, wy


def polygon_sigma(s):
    """sqrt of polygon area in the image, the Corke-Hutchinson scale feature."""
    x, y = s[:, 0], s[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    return np.sqrt(max(area, 1e-12))


def line_alpha(s, i=0, j=1):
    """Orientation of the line between two DESIGNATED features.

    Corke & Hutchinson use a specific feature pair, not a data-dependent
    choice. Selecting the pair by a property of the current image (e.g. the
    longest line) makes alpha jump whenever the selection changes, which on a
    symmetric target happens constantly and breaks the controller. The pair
    must be fixed and the same in the current and desired images.
    """
    return np.arctan2(s[j, 1] - s[i, 1], s[j, 0] - s[i, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def run_partitioned(P_o, cTo_init, cTo_star, lam=0.5, lam_z=0.6, lam_a=0.6,
                    dt=0.033, steps=1500, visible_mask_fn=None, max_range=50.0,
                    rel_tau=None, min_features=3, feature_fn=None):
    P_star_c = transform_points(cTo_star, P_o)
    s_star_all = project(P_star_c)

    cTo = cTo_init.copy()
    log = {k: [] for k in ["t", "err", "v", "cam_pos", "n_vis",
                           "cond_xy", "sigma_min_xy", "vz", "wz", "rank_xy"]}

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

        sv, Zv = s[vis], Z[vis]
        s_star_v = s_star_all[vis]

        e = (sv - s_star_v).reshape(-1)
        L = interaction_matrix(sv, Zv)

        # --- the two partitioned DOF, driven by direct image measurements ---
        sig, sig_star = polygon_sigma(sv), polygon_sigma(s_star_v)
        vz = lam_z * np.log(sig_star / sig)

        al, al_star = line_alpha(sv), line_alpha(s_star_v)
        # verified numerically: d(alpha)/d(wz) = -1 exactly, so the
        # sign here is POSITIVE. Getting this backwards makes the
        # controller rotate away from the goal.
        wz = lam_a * wrap(al - al_star)

        # --- remaining four DOF from the reduced interaction matrix ---
        L_z = L[:, Z_COLS]
        L_xy = L[:, XY_COLS]
        # sdot = L_xy v_xy + L_z v_z, and we want sdot = -lam*e, so
        #   v_xy = -pinv(L_xy) (lam*e + L_z v_z)
        # The coupling term is ADDED: the xy DOF must cancel the image motion
        # that the partitioned z DOF are about to cause.
        v_z = np.array([vz, wz])
        if rel_tau is None:
            L_xy_inv = np.linalg.pinv(L_xy)
            rank_xy = min(L_xy.shape)
        else:
            # adaptive rank on the REDUCED matrix: the 2001 partition handles
            # the retreat coupling, truncation handles information loss.
            L_xy_inv, rank_xy = pinv_truncated(L_xy, rel_tau)
        v_xy = -L_xy_inv @ (lam * e + L_z @ v_z)

        v = np.zeros(6)
        v[XY_COLS] = v_xy
        v[2], v[5] = vz, wz

        S = np.linalg.svd(L_xy, compute_uv=False)
        e_full = (s - s_star_all)[detected].reshape(-1)
        cam_pos = -cTo[:3, :3].T @ cTo[:3, 3]

        log["t"].append(k * dt)
        log["err"].append(float(np.linalg.norm(e_full)))
        log["v"].append(v.copy())
        log["cam_pos"].append(cam_pos.copy())
        log["n_vis"].append(int(vis.sum()))
        log["cond_xy"].append(float(S[0] / max(S[-1], 1e-16)))
        log["sigma_min_xy"].append(float(S[-1]))
        log["vz"].append(float(vz))
        log["wz"].append(float(wz))
        log["rank_xy"].append(int(rank_xy))

        cTo = se3_exp(-v * dt) @ cTo
        if np.linalg.norm(cam_pos) > max_range:
            break

    for key in log:
        log[key] = np.array(log[key])
    return log
