"""Task 1: make the partition a RUNTIME decision instead of a fixed one.

Established by the four fixed controllers, and locked by
blindspot/checks/regression.py:

  - Camera retreat is a COUPLING failure. The 2001 partition fixes it,
    truncation does nothing for it.
  - Feature loss is a CONDITIONING failure. Truncation fixes it, and the
    fixed partition fails catastrophically (288x) on two features, because
    two points make the reduced 4x4 inversion exactly determined and brittle
    while classic IBVS stays underdetermined and is protected by the
    minimum-norm solution.

No fixed choice wins everywhere: combined still spikes 11.6x on two features
where truncation alone manages 0.9x. So switch per step between

    'partition'  the combined scheme  (2001 partition + truncated reduced inverse)
    'truncate'   truncation only on the full interaction matrix

The decision must be made from the interaction matrix spectrum, because image
error is provably useless as a health signal here (alignment sits at exactly
-1.0 throughout the retreat failure).

Two rules are implemented.

`count` - the obvious first try, and the one to beat. Keep the
    partition while enough features remain to leave the reduced inversion
    overdetermined. L_xy is 2N x 4, so that means N >= 3.

`rank_margin` - the spectral rule, and the better one. The reduced solve
    inverts L_xy for 4 unknowns (vx, vy, wx, wy). Keep the partition only
    while the image supplies strictly more independent constraints than those
    4 unknowns, measured as numerical rank at tolerance tau:

        use partition  <=>  rank_tau_switch(L) > len(XY_COLS)   (i.e. > 4)

    If rank_tau(L) <= 4 every usable direction the camera can observe is
    consumed by the 4 xy unknowns. The two partitioned DOF then inject image
    motion (L_z @ v_z) that the reduced solve must cancel exactly, with no
    redundancy left to absorb it, and that is the 288x regime. Above 4 there
    is slack and the partition is safe.

    NOTE: the margin is measured against the COLUMN COUNT, not against
    rank_tau(L_xy). An earlier version of this rule compared the two ranks
    and was wrong: on two features L_xy is 4x4 but numerically rank 3 at
    tau=1e-3, so rank_tau(L_xy) < rank_tau(L) held and the partition stayed
    on in exactly the case it must be dropped. The finite-difference checks
    (blindspot/checks/fd_check.py) caught that.

    The rank test uses its OWN threshold tau_switch, not the truncation tau.
    They ask different questions: truncation asks "is this direction too weak
    to invert through", the switch asks "does this direction exist at all".
    Conflating them is not safe here. Measured on the clean 6-point ring the
    spectrum is

        sigma/sigma_max = [1, 0.99, 0.041, 0.037, 0.00136, 0.00136]

    so the two weakly observable rotational directions sit at 1.36e-3. Sharing
    tau = 1e-3 leaves a margin of only 1.36x: at tau = 3e-3 the clean run reads
    rank 4, the partition is dropped permanently, and the retreat fix is lost
    (measured: 3.90 m, no convergence). With tau_switch = 1e-6 the same test
    has ~1400x of margin, and tau can then be swept for truncation alone
    without destroying the switch. See tau_sweep.py.

`sigma6` - the rule that actually uses the spectrum. Drop the partition when
    the 6th singular value of L, with the spectrum padded to length 6, falls
    below a threshold calibrated from healthy poses:

        use partition  <=>  sigma_6(L) > thresh

    Padding is what makes this work. L is 2N x 6, so with N=2 it has only 4
    singular values and sigma_6 is EXACTLY 0: the two missing DOF are
    unobservable, not merely weak. The rank-based rules above could not see
    that, which is why they collapsed into feature counting.

    Unlike `count` this also fires on a geometric degeneracy at FULL feature
    count - a target collapsed toward a line - which counting cannot detect
    even in principle.

    thresh must be calibrated per target, by sampling sigma_6 over healthy
    poses (see calibrate_sigma6). It is NOT a global constant: measured
    healthy 1st percentile is 3.9e-3 for the 6-point ring but 1.8e-2 for the
    4-point square, a factor of 4.5, because sigma_6 carries the scale of the
    target and the viewing distance.

    This strictly generalises `count`: with N=2 both rules drop the partition,
    but `rank_margin` also drops it when features are plentiful and a
    direction has gone numerically blind anyway - a case `count` cannot see,
    because counting features says nothing about whether they are informative.
"""

import numpy as np

from blindspot.study.ibvs_core import (se3_exp, transform_points, project, interaction_matrix,
                                       sigma_6, make_pose, rot_z, rot_x, rot_y)
from blindspot.study.partitioned import (Z_COLS, XY_COLS, polygon_sigma, line_alpha, wrap)
from blindspot.study.truncated import pinv_truncated


def numerical_rank(L, rel_tau):
    """Number of singular values above rel_tau * sigma_max."""
    S = np.linalg.svd(L, compute_uv=False)
    return int((S > rel_tau * S[0]).sum())


def _healthy_poses(P_o, n, seed, z_range, lateral, tilt, n_visible=None):
    """Yield n random healthy camera poses viewing P_o, all features in front.

    If n_visible is given, also draw a random subset of that many features at
    each pose, so the distribution is conditioned on the feature count. This
    matters: singular-value interlacing means removing rows from L cannot
    increase any singular value, so a 3-feature sigma_6 is bounded above by
    the 6-feature one. Calibrating at N=6 and operating at N=3 pushes the
    operating point toward the threshold by construction.
    """
    rng = np.random.default_rng(seed)
    got = 0
    while got < n:
        R = (rot_z(rng.uniform(-np.pi, np.pi))
             @ rot_x(rng.uniform(-tilt, tilt))
             @ rot_y(rng.uniform(-tilt, tilt)))
        tvec = np.array([rng.uniform(-lateral, lateral),
                         rng.uniform(-lateral, lateral),
                         rng.uniform(*z_range)])
        P_c = transform_points(make_pose(R=R, t=tvec), P_o)
        if np.any(P_c[:, 2] <= 1e-3):
            continue
        if n_visible is not None and n_visible < P_o.shape[0]:
            idx = np.sort(rng.choice(P_o.shape[0], n_visible, replace=False))
            P_c = P_c[idx]
        got += 1
        yield P_c


def calibrate_area(P_o, pct=1.0, n=4000, seed=0, z_range=(0.5, 1.0),
                   lateral=0.1, tilt=0.3, n_visible=None):
    """Threshold for the area_guard rule: sigma = sqrt(polygon area).

    Calibrated exactly like calibrate_sigma6 - same pose distribution, same
    percentile, same per-target requirement - so the two rules differ only in
    WHICH quantity they test, never in how the threshold was obtained.
    """
    vals = [polygon_sigma(project(P_c))
            for P_c in _healthy_poses(P_o, n, seed, z_range, lateral,
                                      tilt, n_visible)]
    return float(np.percentile(np.array(vals), pct))


def calibrate_alpha(P_o, pct=1.0, n=4000, seed=0, z_range=(0.5, 1.0),
                    lateral=0.1, tilt=0.3, n_visible=None):
    """Threshold for the alpha_guard rule: the image baseline |s1 - s0|.

    alpha = atan2 of the line between the two DESIGNATED features. Its noise
    scales as 1/baseline, so the feature is untrustworthy when that pair
    projects close together, which is a different failure from the polygon
    collapsing. Calibrated identically.
    """
    vals = [np.linalg.norm(project(P_c)[1] - project(P_c)[0])
            for P_c in _healthy_poses(P_o, n, seed, z_range, lateral,
                                      tilt, n_visible)]
    return float(np.percentile(np.array(vals), pct))


def calibrate_sigma6(P_o, pct=1.0, n=4000, seed=0,
                     z_range=(0.5, 1.0), lateral=0.1, tilt=0.3,
                     n_visible=None):
    """Threshold for the sigma6 rule, from a HEALTHY pose distribution.

    Samples n random camera poses viewing the healthy target P_o with every
    feature visible, and returns the `pct` percentile of sigma_6. Poses are
    drawn as
        translation : x, y ~ U(-lateral, lateral),  z ~ U(*z_range)
        rotation    : Rz ~ U(-pi, pi), then Rx, Ry ~ U(-tilt, tilt)
    Poses that put any feature behind the camera are rejected and redrawn.

    The percentile is a false-positive budget: at pct=1 the rule gives up the
    partition on 1% of healthy poses, which costs almost nothing because the
    fallback (truncation only) is itself a competent controller.
    """
    vals = [sigma_6(interaction_matrix(project(P_c), P_c[:, 2]))
            for P_c in _healthy_poses(P_o, n, seed, z_range, lateral,
                                      tilt, n_visible)]
    return float(np.percentile(np.array(vals), pct))


def calibrate_sigma6_table(P_o, pct=1.0, **kw):
    """sigma6 thresholds conditioned on the number of visible features.

    Returns {N: threshold} for N = 2 .. len(P_o). Fixes the thin-margin
    problem: with one N=6 threshold the dropout case clears it by only 1.19x,
    because the operating point is an N=3 value. Conditioning on N restores
    the margin to ~47x. At N=2 the healthy threshold is exactly 0.0, and
    `sigma_6 > 0.0` is False, so the partition is still correctly dropped.
    """
    return {N: calibrate_sigma6(P_o, pct=pct, n_visible=N, **kw)
            for N in range(2, P_o.shape[0] + 1)}


def use_partition(L, L_xy, tau_switch, rule, sigma6_thresh=None,
                  sv=None, guard_thresh=None):
    """Decide whether the partition is safe this step.

    The spectral rules look only at L. The guard rules look only at the
    partition's own substitute features and never at the spectrum.
    """
    if rule == "area_guard":
        # Is sqrt(polygon area) still in its healthy range?
        return polygon_sigma(sv) > guard_thresh
    if rule == "alpha_guard":
        # Is the designated feature pair still far enough apart to give a
        # trustworthy line angle?
        return np.linalg.norm(sv[1] - sv[0]) > guard_thresh
    if rule == "sigma6_n":
        # sigma6_thresh is a {n_visible: threshold} table
        n_vis = L.shape[0] // 2
        return sigma_6(L) > sigma6_thresh[n_vis]
    if rule == "sigma6":
        if sigma6_thresh is None:
            raise ValueError("rule='sigma6' needs sigma6_thresh "
                             "(see calibrate_sigma6)")
        return sigma_6(L) > sigma6_thresh
    if rule == "count":
        # L_xy is 2N x 4; overdetermined means 2N > 4, i.e. N >= 3.
        return L_xy.shape[0] > L_xy.shape[1]
    if rule == "rank_margin":
        # strictly more usable independent constraints than xy unknowns
        return numerical_rank(L, tau_switch) > L_xy.shape[1]
    if rule == "always":            # == the fixed combined controller
        return True
    if rule == "never":             # == truncation only
        return False
    raise ValueError(rule)


def run_switched(P_o, cTo_init, cTo_star, lam=0.5, lam_z=0.6, lam_a=0.6,
                 dt=0.033, steps=1500, visible_mask_fn=None, max_range=50.0,
                 rel_tau=1e-3, rule="rank_margin", min_features=3,
                 tau_switch=1e-6, sigma6_thresh=None, guard_thresh=None,
                 feature_fn=None, hysteresis=0):
    P_star_c = transform_points(cTo_star, P_o)
    s_star_all = project(P_star_c)

    cTo = cTo_init.copy()
    _hyst = {"state": True, "count": 0}
    log = {k: [] for k in ["t", "err", "v", "cam_pos", "n_vis", "partition",
                           "rank_L", "rank_xy", "cond", "sigma_6"]}

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
        L_xy = L[:, XY_COLS]

        want = use_partition(L, L_xy, tau_switch, rule, sigma6_thresh,
                             sv=sv, guard_thresh=guard_thresh)
        if hysteresis <= 0:
            on = want
        else:
            # hold-off: the candidate state must persist `hysteresis` steps
            # before it is adopted. Measured to be unnecessary (the guard
            # does not chatter) and to cost a delayed re-engagement, so the
            # default is 0. See the hysteresis section of the regression
            # checks.
            if want == _hyst["state"]:
                _hyst["count"] = 0
            else:
                _hyst["count"] += 1
                if _hyst["count"] >= hysteresis:
                    _hyst["state"] = want
                    _hyst["count"] = 0
            on = _hyst["state"]

        if on:
            # --- combined scheme: 2001 partition, truncated reduced inverse ---
            sig, sig_star = polygon_sigma(sv), polygon_sigma(s_star_v)
            vz = lam_z * np.log(sig_star / sig)
            al, al_star = line_alpha(sv), line_alpha(s_star_v)
            wz = lam_a * wrap(al - al_star)          # sign verified: d(alpha)/d(wz) = -1

            v_z = np.array([vz, wz])
            L_xy_inv, rank_xy = pinv_truncated(L_xy, rel_tau)
            # coupling term ADDED: xy must cancel the image motion the
            # partitioned z DOF are about to cause.
            v_xy = -L_xy_inv @ (lam * e + L[:, Z_COLS] @ v_z)

            v = np.zeros(6)
            v[XY_COLS] = v_xy
            v[2], v[5] = vz, wz
            rank_L = numerical_rank(L, tau_switch)
        else:
            # --- truncation only, full interaction matrix ---
            Linv, _ = pinv_truncated(L, rel_tau)
            v = -lam * Linv @ e
            rank_L = numerical_rank(L, tau_switch)
            rank_xy = numerical_rank(L_xy, rel_tau)

        S = np.linalg.svd(L, compute_uv=False)
        cam_pos = -cTo[:3, :3].T @ cTo[:3, 3]

        log["t"].append(k * dt)
        log["err"].append(
            float(np.linalg.norm((s - s_star_all)[detected].reshape(-1))))
        log["v"].append(v.copy())
        log["cam_pos"].append(cam_pos.copy())
        log["n_vis"].append(int(vis.sum()))
        log["partition"].append(bool(on))
        log["rank_L"].append(int(rank_L))
        log["rank_xy"].append(int(rank_xy))
        log["cond"].append(float(S[0] / max(S[-1], 1e-16)))
        log["sigma_6"].append(sigma_6(L))

        cTo = se3_exp(-v * dt) @ cTo
        if np.linalg.norm(cam_pos) > max_range:
            break

    for key in log:
        log[key] = np.array(log[key])
    return log
