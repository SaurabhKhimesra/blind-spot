"""Turning detector output into something the guard can score.

No ROS imports here on purpose: this is the part that can be wrong, so it is
testable without a ROS installation. The node in blindspot_ros/ is a thin
wrapper around it.

Two things this exists to get right.

ORDERING. The guard signal is a shoelace polygon area, which depends on the
ORDER of the points. A detector reports whatever order it happens to find, and
an order that crosses itself gives a small area - firing the guard for a
reason that has nothing to do with the geometry. Points are therefore ordered
by detection id, which reproduces the order the calibration used. Without ids
we fall back to an angular sort about the centroid, which is correct for a
convex arrangement and is reported so it can be seen in the diagnostic.

UNITS. The guard works in normalised image coordinates. Use the principal
point from CameraInfo when the camera is actually calibrated; only fall back
to (W-1)/2 when it is not, and say which was used. Note (W-1)/2, not W/2:
OpenGL maps NDC onto a continuous pixel range so the axis lands at W/2, while
OpenCV indexes pixel centres half a pixel lower.
"""

import numpy as np

from .units import pixels_to_normalised


def detection_center(det):
    """(x, y) pixel centre of a Detection2D, across vision_msgs versions.

    vision_msgs 4.x uses BoundingBox2D.center = vision_msgs/Pose2D, which has
    a `.position` with x and y. Earlier versions used geometry_msgs/Pose2D,
    with x and y directly on the centre. Duck-typed so both work and so this
    is testable without a ROS installation.
    """
    c = det.bbox.center
    pos = getattr(c, "position", None)
    if pos is not None:
        return float(pos.x), float(pos.y)
    return float(c.x), float(c.y)


def detection_label(det):
    """Stable id for a Detection2D, or None if the detector reports none.

    Prefers Detection2D.id, falls back to the first hypothesis class_id
    (vision_msgs 4.x) or result id (earlier).
    """
    did = getattr(det, "id", "")
    if did:
        return did
    for r in getattr(det, "results", []) or []:
        hyp = getattr(r, "hypothesis", None)
        cid = (getattr(hyp, "class_id", None) if hyp is not None
               else getattr(r, "id", None))
        if cid not in (None, ""):
            return cid
    return None


def angular_order(uv):
    """Indices that sort points counter-clockwise about their centroid."""
    uv = np.asarray(uv, dtype=float)
    c = uv.mean(axis=0)
    return np.argsort(np.arctan2(uv[:, 1] - c[1], uv[:, 0] - c[0]))


def order_features(uv, ids=None):
    """Order detections so the shoelace area means what it should.

    Returns (ordered_uv, ordered_ids, how) where `how` is "id" or "angular".
    """
    uv = np.asarray(uv, dtype=float)
    if ids is not None and len(ids) == len(uv):
        keys = []
        for i in ids:
            try:
                keys.append((0, float(i)))
            except (TypeError, ValueError):
                keys.append((1, str(i)))
        try:
            idx = np.array(sorted(range(len(uv)), key=lambda k: keys[k]))
        except TypeError:                     # mixed unorderable keys
            idx = np.array(sorted(range(len(uv)), key=lambda k: str(ids[k])))
        return uv[idx], [ids[i] for i in idx], "id"
    idx = angular_order(uv)
    return uv[idx], None, "angular"


def intrinsics_from_camera_info(K, width, height):
    """(fx, fy, cx, cy, source) from a CameraInfo K matrix.

    K is the row-major 9-vector [fx 0 cx; 0 fy cy; 0 0 1]. An uncalibrated
    camera publishes zeros, in which case there is nothing to use and the
    caller must supply a field of view instead.
    """
    K = np.asarray(K, dtype=float).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if fx > 0 and fy > 0:
        return float(fx), float(fy), float(cx), float(cy), "camera_info"
    return None, None, (width - 1) / 2.0, (height - 1) / 2.0, "uncalibrated"


def features_to_normalised(uv, fx, fy, cx, cy, ids=None):
    """Detector pixels -> ordered normalised coordinates for the guard."""
    ordered, ordered_ids, how = order_features(uv, ids)
    return pixels_to_normalised(ordered, fx, fy, cx, cy), ordered_ids, how


def ordering_is_suspect(uv, tol=0.98):
    """True if the given order encloses much less area than an angular sort.

    A cheap self-check: for points in a sane order the two agree. If they do
    not, the order crosses itself and the area - and therefore the guard
    decision - is not measuring the geometry.

    The reference is the polygon of the angular sort about the centroid, which
    is the convex hull only when the points are in convex position; for a
    concave arrangement it under-reports, so this errs towards staying quiet.
    """
    uv = np.asarray(uv, dtype=float)
    if uv.shape[0] < 3:
        return False

    def shoelace(p):
        x, y = p[:, 0], p[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    hull = uv[angular_order(uv)]
    a_hull = shoelace(hull)
    if a_hull <= 0:
        return False
    return shoelace(uv) < tol * a_hull
