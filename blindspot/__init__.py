"""blindspot - a per-step answer to "is the partition safe right now".

    from blindspot import FeatureGuard

    guard = FeatureGuard.load("my_target.json")
    if guard.partition_ok(s_visible):
        v = my_partitioned_control(...)
    else:
        v = my_plain_control(...)

Features are normalised image coordinates (see blindspot.units). Reference
controllers live in blindspot.reference and are examples, not the product.
"""

from .guard import (Calibration, FeatureGuard, GuardReading,  # noqa: F401
                    polygon_sigma)
from .units import intrinsics_from_fov, pixels_to_normalised  # noqa: F401

__all__ = ["FeatureGuard", "GuardReading", "Calibration", "polygon_sigma",
           "pixels_to_normalised", "intrinsics_from_fov"]
__version__ = "0.1.0"
