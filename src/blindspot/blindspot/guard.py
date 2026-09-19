"""The guard: a per-step answer to "is the partition safe right now".

Nothing in this module knows about a controller. It takes the visible feature
positions and returns a decision plus the numbers behind it.

Two things are deliberately made impossible here.

  * There is NO default threshold. A guard cannot be constructed without a
    calibration produced from a KNOWN-GOOD target. Calibrating on whatever the
    camera happens to be looking at makes the degeneracy the norm and silently
    disables the guard - measured, the in-situ threshold came out 7x lower,
    never fired, and the run floored while still looking like it worked.

  * This module has no truncation tolerance. The guard threshold and the
    pseudo-inverse tolerance tau are different quantities answering different
    questions, and sharing one value between them was a real bug that left a
    1.36x margin before the retreat fix broke. tau belongs to whatever
    controller you run, not to the guard.
"""

import json
from dataclasses import dataclass

import numpy as np

SCHEMA = "blindspot/guard-calibration/1"


def polygon_sigma(s):
    """sqrt of the polygon area of the feature set, in normalised units.

    An aggregate over every visible point, so independent per-point detector
    noise largely cancels inside it: measured, 2 px of per-point noise moves
    this by well under 1% of a typical threshold.
    """
    s = np.asarray(s, dtype=float)
    x, y = s[:, 0], s[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    return float(np.sqrt(max(area, 0.0)))


@dataclass(frozen=True)
class GuardReading:
    """Everything behind one decision. Log this, not just the bool."""
    signal: float
    threshold: float
    margin: float          # signal / threshold; < 1 means the guard fires
    n_features: int
    decision: bool         # True = the partition is safe to use

    def __str__(self):
        return ("signal=%.4e threshold=%.4e margin=%.2fx n=%d partition=%s"
                % (self.signal, self.threshold, self.margin, self.n_features,
                   "ok" if self.decision else "DROP"))


class Calibration:
    """Per-feature-count thresholds measured on a known-good target."""

    def __init__(self, thresholds, metadata=None):
        if not thresholds:
            raise ValueError("a calibration needs at least one threshold")
        self.thresholds = {int(k): float(v) for k, v in thresholds.items()}
        self.metadata = dict(metadata or {})

    def threshold_for(self, n):
        """Threshold for n visible features.

        Conditioned on the count because singular-value interlacing means a
        subset's statistic is bounded above by the full set's: calibrating at
        N=6 and operating at N=3 left a 1.19x margin where conditioning gives
        45.9x.
        """
        if n in self.thresholds:
            return self.thresholds[n]
        known = sorted(self.thresholds)
        if n > known[-1]:
            return self.thresholds[known[-1]]
        below = [k for k in known if k < n]
        if below:
            return self.thresholds[below[-1]]
        return self.thresholds[known[0]]

    def to_dict(self):
        return {"schema": SCHEMA,
                "thresholds": {str(k): v for k, v in
                               sorted(self.thresholds.items())},
                "metadata": self.metadata}

    @classmethod
    def from_dict(cls, d):
        if d.get("schema") != SCHEMA:
            raise ValueError("not a blindspot calibration file (schema=%r)"
                             % d.get("schema"))
        return cls(d["thresholds"], d.get("metadata"))


class FeatureGuard:
    """Decides, per step, whether the partitioned control law is safe.

        guard = FeatureGuard.load("my_target.json")
        if guard.partition_ok(s_visible):
            v = my_partitioned_control(...)
        else:
            v = my_plain_control(...)

    `s_visible` is an (N, 2) array of the CURRENTLY VISIBLE features in
    normalised image coordinates - see blindspot.units.
    """

    def __init__(self, calibration):
        if calibration is None:
            raise ValueError(
                "FeatureGuard requires a calibration and ships no default "
                "threshold.\n"
                "The threshold carries the scale of your target and viewing "
                "distance (measured: 3.9e-03 for one target, 1.7e-02 for "
                "another), so no global constant exists.\n"
                "Calibrate on a KNOWN-GOOD target:\n"
                "    ros2 run blindspot calibrate --geometry target.json "
                "--goal-pose pose.json -o my_target.json\n"
                "then FeatureGuard.load('my_target.json').\n"
                "Do NOT calibrate on the target you are currently servoing "
                "to unless you have verified it is healthy: calibrating in "
                "situ on a degenerate target makes the degeneracy the norm "
                "and silently disables the guard.")
        if not isinstance(calibration, Calibration):
            raise TypeError("expected a Calibration, got %r"
                            % type(calibration).__name__)
        self.calibration = calibration

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(Calibration.from_dict(json.load(f)))

    def evaluate(self, s_visible):
        """Full reading: signal, threshold, margin, count, decision."""
        s = np.asarray(s_visible, dtype=float)
        if s.ndim != 2 or s.shape[1] != 2:
            raise ValueError("expected an (N, 2) array of normalised image "
                             "coordinates, got shape %r" % (s.shape,))
        n = s.shape[0]
        thr = self.calibration.threshold_for(n)
        if n < 3:
            # fewer than three points have no polygon at all, so there is
            # nothing for the partition's area feature to stand on.
            return GuardReading(0.0, thr, 0.0, n, False)
        sig = polygon_sigma(s)
        return GuardReading(sig, thr, sig / thr if thr > 0 else np.inf, n,
                            bool(sig > thr))

    def partition_ok(self, s_visible):
        """True if the partitioned control law is safe this step."""
        return self.evaluate(s_visible).decision
