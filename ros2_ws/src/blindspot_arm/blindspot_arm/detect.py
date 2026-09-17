"""Blob detection for the panel markers. The only part that needs OpenCV.

Kept out of law.py so the control law stays numpy-only and can be checked by
tests.py, which the repo promises runs on numpy, scipy and matplotlib alone.
"""

import cv2
import numpy as np


def detect_dots(rgb):
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    _, th = cv2.threshold(g, 90, 255, cv2.THRESH_BINARY_INV)
    n, _, stats, cent = cv2.connectedComponentsWithStats(th, 8)
    cand = [(int(s[4]), c) for s, c in zip(stats[1:], cent[1:])
            if 25 < s[4] < 4000]
    if not cand:
        return np.zeros((0, 2))
    # The panel carries exactly six dots. Rendering artefacts and the post
    # edge can add spurious components, so keep the six largest: a 7th blob
    # otherwise breaks correspondence against the six desired features.
    # The six dots are identical circles, so pick the six most SIMILAR
    # blobs, not the six largest. Taking the largest lets the robot's own
    # dark wrist - which is in frame at close standoff - masquerade as a
    # fiducial and displace a real dot. Offline the control law converges to
    # |e| 0.0008 on ground-truth features, so divergence in the sim was
    # perception, not control.
    if len(cand) < 6:
        return np.array([c for _, c in cand], dtype=float)
    cand.sort(key=lambda x: x[0])
    best, spread = None, np.inf
    for i in range(len(cand) - 5):
        win = cand[i:i + 6]
        sp = win[-1][0] / max(win[0][0], 1)
        if sp < spread:
            best, spread = win, sp
    return np.array([c for _, c in best], dtype=float)
