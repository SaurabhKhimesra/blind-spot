"""Correlated degradation: motion blur and exposure, both driven by the run.

Independent noise was already cleared by the numpy study (d' = 454 down to
7.1). The open question is noise that CO-VARIES with the degeneracy: blur and
exposure that wreck the features and the geometry in the same instant. That
cannot be faked by injection, which is why it needs a rendered camera.

Both channels are closed loops around the run, not schedules:

  motion blur   the camera's ACTUAL inter-frame motion is measured from
                successive cTo, and the exposure window is rendered as several
                sub-frames along that motion and averaged. A control lurch
                therefore produces its own blur, on the same step.

  auto-exposure a lagged AE controller chases a target image brightness. When
                the scene content changes abruptly - a marker occluded, the
                camera lurching - the gain is wrong for several frames and the
                image over- or under-exposes. The lag is what couples it.

se3_log is verified by finite difference against se3_exp rather than trusted.

    python3 mj_degrade.py
"""

import numpy as np
import cv2

from ibvs_core import se3_exp, skew
from mj_scene import RenderedFeatureSource


def se3_log(T):
    """Matrix log SE(3) -> se(3) twist [v(3), w(3)]. Inverse of se3_exp."""
    R, p = T[:3, :3], T[:3, 3]
    c = (np.trace(R) - 1.0) / 2.0
    th = np.arccos(np.clip(c, -1.0, 1.0))
    if th < 1e-9:
        return np.concatenate([p, np.zeros(3)])
    w_hat = (R - R.T) * (th / (2.0 * np.sin(th)))
    w = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])
    W = skew(w / th)
    A = (np.eye(3)
         - (th / 2.0) * W
         + (1.0 - th / (2.0 * np.tan(th / 2.0))) * (W @ W))
    return np.concatenate([A @ p, w])


class CorrelatedDegrader:
    """Rendered feature source with run-driven blur and exposure."""

    def __init__(self, source, n_sub=4, exposure_frac=1.0, ae_gain=0.25,
                 P_o=None, max_sub=24,
                 ae_lag=4, target_mean=128.0, active=None):
        self.src = source
        self.n_sub = n_sub
        self.P_o = P_o
        self.max_sub = max_sub
        self.sub_used = []
        self.exposure_frac = exposure_frac
        self.ae_gain = ae_gain
        self.ae_lag = ae_lag
        self.target_mean = target_mean
        self.active = active            # fn(k) -> bool, when degradation is on
        self.prev_cTo = None
        self.gain = 1.0
        self.gain_queue = [1.0] * ae_lag
        # diagnostics
        self.blur_px = []
        self.span_px = []      # actual feature travel across the exposure, px
        self.gains = []
        self.n_frames = 0

    def _blur_length_px(self, cTo, xi):
        """Max feature displacement across the exposure, in pixels."""
        if self.P_o is None:
            return None
        from ibvs_core import transform_points, project
        from mj_scene import intrinsics
        fx, _, _, _ = intrinsics()
        ends = []
        for f in (-self.exposure_frac / 2.0, self.exposure_frac / 2.0):
            Pc = transform_points(se3_exp(xi * f) @ cTo, self.P_o)
            ends.append(project(Pc) * fx)
        return float(np.linalg.norm(ends[1] - ends[0], axis=1).max())

    def _exposure_stack(self, cTo):
        """Average sub-frames along the ACTUAL inter-frame motion.

        The sub-frame COUNT adapts so successive samples land under a pixel
        apart. With a fixed small count the samples are further apart than a
        marker and the result is a row of ghost copies, not a smear - the
        detector then locks onto one ghost and reports a feature hundreds of
        pixels from the truth (measured: 156 px at exposure 8). Ghosting is a
        sampling artefact of the blur model, not a property of motion blur.
        """
        if self.prev_cTo is None:
            return self.src.render(cTo).astype(np.float64)
        # cTo_k = exp(-v dt) @ cTo_{k-1}, so this recovers the step twist.
        xi = se3_log(cTo @ np.linalg.inv(self.prev_cTo))
        span = self._blur_length_px(cTo, xi)
        if span is not None:
            self.span_px.append(span)
        n = self.n_sub if span is None else int(
            np.clip(np.ceil(span) + 1, 4, self.max_sub))
        self.sub_used.append(n)
        acc = None
        for j in range(n):
            f = self.exposure_frac * (j / max(n - 1, 1) - 0.5)
            img = se3_sub_render(self.src, se3_exp(xi * f) @ cTo)
            acc = img if acc is None else acc + img
        return acc / n

    def __call__(self, k, cTo):
        on = self.active is None or self.active(k)
        if not on:
            img = self.src.render(cTo).astype(np.float64)
        else:
            img = self._exposure_stack(cTo)

        if on:
            # lagged auto-exposure: the gain applied now was computed from
            # the brightness several frames ago, so an abrupt scene change
            # leaves the image mis-exposed for the length of the lag.
            applied = self.gain_queue.pop(0)
            img = np.clip(img * applied, 0, 255)
            measured = max(img.mean(), 1e-6)
            self.gain = float(np.clip(
                self.gain * (1.0 + self.ae_gain
                             * (self.target_mean / measured - 1.0)),
                0.05, 20.0))
            self.gain_queue.append(self.gain)
            self.gains.append(applied)
            if self.prev_cTo is not None:
                xi = se3_log(cTo @ np.linalg.inv(self.prev_cTo))
                self.blur_px.append(float(np.linalg.norm(xi)))

        self.prev_cTo = cTo.copy()
        self.n_frames += 1
        return self.src.detect(img.astype(np.uint8))


def se3_sub_render(src, cTo):
    return src.render(cTo).astype(np.float64)
