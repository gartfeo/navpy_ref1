"""Distance and bearing metrics for geographic locations."""

from __future__ import annotations

import math

import pymap3d

from navpy.modules.common.models.location import Location


def calculate_distance(
    current_loc: Location | None,
    poi_loc: Location | None,
) -> float | None:
    if poi_loc is None or current_loc is None:
        return None
    surface_distance = current_loc.distance_to(poi_loc)
    height_diff = poi_loc.alt - current_loc.alt
    return round(math.sqrt(surface_distance**2 + height_diff**2), 1)


def calculate_bearing(
    current_loc: Location,
    poi_loc: Location,
) -> float:
    azimuth, _, _ = pymap3d.geodetic2aer(
        current_loc.lat,
        current_loc.lng,
        current_loc.alt,
        poi_loc.lat,
        poi_loc.lng,
        poi_loc.alt,
    )
    return round(azimuth, 1)
