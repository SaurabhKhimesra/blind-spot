"""Does detector noise drown the health signal?

The monitor reads sigma_min of the interaction matrix, built from MEASURED
feature positions. Detector error perturbs those positions, so it perturbs
sigma_min. The question is whether that perturbation is small compared with
the change caused by a genuine degeneracy.

We compare two configurations:
    healthy    - 6 features visible, wide target
    degenerate - 3 of 6 features visible (the occlusion case)

and ask: at what detector noise level do the two become indistinguishable?

Noise is specified in PIXELS and converted to normalised image coordinates
using a focal length, so the numbers are comparable to real detectors:
    ArUco / AprilTag corner      ~0.1 - 0.3 px
    good Harris / FAST corner    ~0.2 - 0.5 px
    blurred or low-light feature ~1 - 3 px
"""

import numpy as np
from blindspot.study.ibvs_core import (square_target, make_pose, rot_z, rot_x,
                                       transform_points, project, interaction_matrix)

F_PX = 600.0          # focal length, typical 640x480 webcam
RNG = np.random.default_rng(7)


def hex_target(r=0.05):
    a = np.linspace(0, 2 * np.pi, 7)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(6)], axis=1)


def health(s, Z, s_ref, Z_ref):
    """Normalised health: sigma_min(L) / sigma_min(L_ref)."""
    L = interaction_matrix(s, Z)
    L_ref = interaction_matrix(s_ref, Z_ref)
    return (np.linalg.svd(L, compute_uv=False)[-1]
            / np.linalg.svd(L_ref, compute_uv=False)[-1])


def sample_health(vis, sigma_px, n_trials, bias_px=0.0, outlier_rate=0.0,
                  outlier_px=8.0):
    """Distribution of the health reading under detector error."""
    P_o = hex_target()
    cTo = make_pose(R=rot_z(0.3) @ rot_x(0.12),
                    t=np.array([0.05, -0.04, 0.7]))
    P_c = transform_points(cTo, P_o)
    s_true, Z = project(P_c), P_c[:, 2]

    # reference is the full healthy set at this pose
    s_ref, Z_ref = s_true, Z

    sd = sigma_px / F_PX
    bd = bias_px / F_PX
    od = outlier_px / F_PX

    out = np.empty(n_trials)
    for i in range(n_trials):
        s = s_true.copy()
        s = s + RNG.normal(0.0, sd, size=s.shape)
        if bd:                       # systematic offset, same every frame
            s = s + bd
        if outlier_rate:
            hit = RNG.random(s.shape[0]) < outlier_rate
            s[hit] += RNG.normal(0.0, od, size=(hit.sum(), 2))
        out[i] = health(s[vis], Z[vis], s_ref, Z_ref)
    return out


def separation(a, b):
    """Discriminability d' between two health distributions."""
    pooled = np.sqrt(0.5 * (a.var() + b.var()))
    if pooled < 1e-15:
        return np.inf
    return abs(a.mean() - b.mean()) / pooled


ALL = np.ones(6, dtype=bool)
OCC = np.array([False, False, False, True, True, True])

print("Health reading: sigma_min(L) / sigma_min(L*), 4000 trials per cell")
print("healthy = 6 features, degenerate = 3 of 6 occluded")
print()
print("  noise    healthy h        degenerate h      d'      verdict")
print("  (px)     mean +- sd       mean +- sd")
print("  " + "-" * 66)

N = 4000
for sig in [0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0]:
    h_ok = sample_health(ALL, sig, N)
    h_bad = sample_health(OCC, sig, N)
    d = separation(h_ok, h_bad)
    verdict = ("clear" if d > 5 else
               "usable" if d > 2 else
               "marginal" if d > 1 else "DROWNED")
    print("  %5.2f    %.3f +- %.3f    %.3f +- %.3f    %7s  %s"
          % (sig, h_ok.mean(), h_ok.std(), h_bad.mean(), h_bad.std(),
             ("%.1f" % d) if np.isfinite(d) else "inf", verdict))

print()
print("Nastier error models, held at 0.5 px base noise")
print("  " + "-" * 66)
cases = [("zero-mean only", dict()),
         ("+ 1 px systematic bias", dict(bias_px=1.0)),
         ("+ 5% outliers at 8 px", dict(outlier_rate=0.05)),
         ("+ 20% outliers at 8 px", dict(outlier_rate=0.20)),
         ("+ bias and 20% outliers", dict(bias_px=1.0, outlier_rate=0.20))]
for name, kw in cases:
    h_ok = sample_health(ALL, 0.5, N, **kw)
    h_bad = sample_health(OCC, 0.5, N, **kw)
    d = separation(h_ok, h_bad)
    print("  %-26s d' = %6.1f   healthy sd %.4f  degen sd %.4f"
          % (name, d, h_ok.std(), h_bad.std()))

print()
print("Temporal filtering: does averaging the health signal help?")
print("  (0.5 px noise + 20% outliers, k frames averaged)")
print("  " + "-" * 66)
for k in [1, 3, 5, 10, 20]:
    a = np.array([sample_health(ALL, 0.5, k, outlier_rate=0.20).mean()
                  for _ in range(1500)])
    b = np.array([sample_health(OCC, 0.5, k, outlier_rate=0.20).mean()
                  for _ in range(1500)])
    print("  k = %2d frames    d' = %6.1f" % (k, separation(a, b)))
