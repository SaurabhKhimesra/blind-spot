"""The example target the demos run against, and its calibration.

The guard ships no default threshold, on purpose: a threshold carries the
target's scale and working distance. So the demos do what a real deployment
does at commissioning time - calibrate on a KNOWN-GOOD target - using the
six-point ring the whole study is built around, 0.05 m radius, 0.6 m standoff.

Pass calibration:=/path.json to a launch file to use your own instead.
"""

import json
import os
import tempfile

import numpy as np

from blindspot.calibrate import calibrate

RING_RADIUS = 0.05
STANDOFF = 0.6


def ring(r=RING_RADIUS, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def ensure_calibration(path=""):
    """Return `path` if given, otherwise calibrate the example ring and return
    where that calibration was written."""
    if path:
        return path
    goal = np.eye(4)
    goal[2, 3] = STANDOFF
    cal = calibrate(ring(), goal, n_poses=4000, percentile=1.0, seed=0)
    out = os.path.join(tempfile.gettempdir(), "blindspot_example_ring.json")
    with open(out, "w") as f:
        json.dump(cal.to_dict(), f, indent=2)
    return out
