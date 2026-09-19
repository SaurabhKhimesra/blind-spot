"""Image coordinate conventions.

The guard works in NORMALISED image coordinates: x = X/Z, y = Y/Z, with the
camera looking along +z, x to the right and y DOWN. That is the computer
vision convention, not the OpenGL one.

Detectors report pixels, so convert before calling the guard.

The principal point is (W-1)/2, not W/2. A renderer maps normalised device
coordinates onto a CONTINUOUS pixel range [0, W], so the optical axis lands at
continuous coordinate W/2; OpenCV indexes pixel CENTRES, which sit half a
pixel lower. Using W/2 shows up as a systematic half-pixel bias in every
feature - see the "Five bugs" section of the README, where it hid behind a
corner-refinement method with a compensating offset.
"""

import numpy as np


def intrinsics_from_fov(width, height, fovy_deg):
    """(fx, fy, cx, cy) for a camera specified by vertical field of view."""
    fy = (height / 2.0) / np.tan(np.deg2rad(fovy_deg) / 2.0)
    return fy, fy, (width - 1) / 2.0, (height - 1) / 2.0


def pixels_to_normalised(uv, fx, fy=None, cx=None, cy=None):
    """Pixel coordinates -> normalised image coordinates.

    uv : (N, 2) pixel coordinates, origin at the centre of the top-left pixel
         (the OpenCV convention).
    fx, fy, cx, cy : intrinsics. Pass cx = (W-1)/2 and cy = (H-1)/2 unless you
         have a calibrated principal point.
    """
    uv = np.asarray(uv, dtype=float)
    fy = fx if fy is None else fy
    if cx is None or cy is None:
        raise ValueError("cx and cy are required; for an ideal camera use "
                         "(W-1)/2 and (H-1)/2, not W/2 and H/2")
    return np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy], axis=1)
