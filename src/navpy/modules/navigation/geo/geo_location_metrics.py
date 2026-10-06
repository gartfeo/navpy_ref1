"""Distance and bearing metrics for geographic locations."""

from __future__ import annotations

import math

import pymap3d

from navpy.modules.common.models.location import Location


def calculate_distance(
    current_loc: Location | None,
    target_loc: Location | None,
) -> float | None:
    if target_loc is None or current_loc is None:
        return None
    surface_distance = current_loc.distance_to(target_loc)
    height_diff = target_loc.alt - current_loc.alt
    return round(math.sqrt(surface_distance**2 + height_diff**2), 1)


def calculate_bearing(
    current_loc: Location,
    target_loc: Location,
) -> float:
    azimuth, _, _ = pymap3d.geodetic2aer(
        current_loc.lat,
        current_loc.lng,
        current_loc.alt,
        target_loc.lat,
        target_loc.lng,
        target_loc.alt,
    )
    return round(azimuth, 1)
