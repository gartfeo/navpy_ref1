"""Pure geometry for post-command closest-approach diagnostics."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import pymap3d

from navpy.logger.navigation_snap_types import ClosestPointComponents
from navpy.modules.common.models.location import Location


def is_valid_location(location: Optional[Location]) -> bool:
    return location is not None and all(
        math.isfinite(value)
        for value in (location.lat, location.lng, location.alt)
    )


def calc_h_v_dist(
    current: Optional[Location],
    target: Optional[Location],
) -> tuple[float, float]:
    """Return infinite miss when either diagnostic position is unavailable."""
    if not is_valid_location(current) or not is_valid_location(target):
        return float("inf"), float("inf")
    north, east, down = pymap3d.geodetic2ned(
        target.lat,
        target.lng,
        target.alt,
        current.lat,
        current.lng,
        current.alt,
    )
    return math.hypot(north, east), abs(down)


def calc_distance(
    first: Optional[Location],
    second: Optional[Location],
) -> float:
    """Return infinite distance when either diagnostic position is absent."""
    if not is_valid_location(first) or not is_valid_location(second):
        return float("inf")
    north, east, down = pymap3d.geodetic2ned(
        second.lat,
        second.lng,
        second.alt,
        first.lat,
        first.lng,
        first.alt,
    )
    return math.sqrt(north * north + east * east + down * down)


def _require_segment_locations(
    c0: Location,
    c1: Location,
    target: Location,
) -> None:
    if not all(is_valid_location(location) for location in (c0, c1, target)):
        raise ValueError("closest-point geometry requires finite locations")


def closest_point_components_on_segment(
    c0: Location,
    c1: Location,
    target: Location,
) -> ClosestPointComponents:
    """Find the closest path point and its track-frame miss components."""
    _require_segment_locations(c0, c1, target)
    n1, e1, d1 = pymap3d.geodetic2ned(
        c1.lat, c1.lng, c1.alt, c0.lat, c0.lng, c0.alt,
    )
    nt, et, dt = pymap3d.geodetic2ned(
        target.lat, target.lng, target.alt, c0.lat, c0.lng, c0.alt,
    )
    segment = np.array([n1, e1, d1], dtype=float)
    target_from_start = np.array([nt, et, dt], dtype=float)
    denominator = float(np.dot(segment, segment))
    if denominator < 1e-6:
        closest = np.zeros(3)
    else:
        fraction = float(np.dot(target_from_start, segment) / denominator)
        closest = max(0.0, min(1.0, fraction)) * segment

    residual = target_from_start - closest
    north, east, down = residual.tolist()
    h_dist = math.hypot(north, east)
    v_dist = abs(down)
    slant = math.hypot(h_dist, down)
    ned_unit = (
        residual / slant
        if slant > 1e-9
        else np.array([0.0, 0.0, 0.0])
    )
    horizontal_track = np.array([n1, e1], dtype=float)
    track_norm = float(np.linalg.norm(horizontal_track))
    track_unit = (
        horizontal_track / track_norm
        if track_norm > 1e-9
        else np.array([1.0, 0.0])
    )
    lateral_unit = np.array([-track_unit[1], track_unit[0]])
    horizontal_residual = np.array([north, east], dtype=float)
    longitudinal_signed = float(np.dot(horizontal_residual, track_unit))
    lateral_signed = float(np.dot(horizontal_residual, lateral_unit))
    vertical_signed = float(down)

    closest_north, closest_east, closest_down = closest.tolist()
    latitude, longitude, altitude = pymap3d.ned2geodetic(
        closest_north,
        closest_east,
        closest_down,
        c0.lat,
        c0.lng,
        c0.alt,
    )
    return ClosestPointComponents(
        c_star=Location(latitude, longitude, altitude, is_absolute=True),
        h_dist=h_dist,
        v_dist=v_dist,
        slant=slant,
        ned_unit=ned_unit,
        lateral=abs(lateral_signed),
        longitudinal=abs(longitudinal_signed),
        vertical=abs(vertical_signed),
        lateral_signed=lateral_signed,
        longitudinal_signed=longitudinal_signed,
        vertical_signed=vertical_signed,
    )


def closest_on_segment(
    c0: Location,
    c1: Location,
    target: Location,
) -> tuple[Location, float, float, float, np.ndarray]:
    components = closest_point_components_on_segment(c0, c1, target)
    return (
        components.c_star,
        components.h_dist,
        components.v_dist,
        components.slant,
        components.ned_unit,
    )


def closest_horizontal_on_segment(
    c0: Location,
    c1: Location,
    target: Location,
) -> tuple[float, float]:
    """Return minimum horizontal miss and vertical miss at that point."""
    _require_segment_locations(c0, c1, target)
    n1, e1, d1 = pymap3d.geodetic2ned(
        c1.lat, c1.lng, c1.alt, c0.lat, c0.lng, c0.alt,
    )
    nt, et, dt = pymap3d.geodetic2ned(
        target.lat, target.lng, target.alt, c0.lat, c0.lng, c0.alt,
    )
    segment_horizontal = np.array([n1, e1], dtype=float)
    target_horizontal = np.array([nt, et], dtype=float)
    denominator = float(np.dot(segment_horizontal, segment_horizontal))
    if denominator < 1e-6:
        fraction = 0.0
    else:
        fraction = float(np.dot(target_horizontal, segment_horizontal) / denominator)
        fraction = max(0.0, min(1.0, fraction))

    residual_north = nt - fraction * n1
    residual_east = et - fraction * e1
    residual_down = dt - fraction * d1
    return math.hypot(residual_north, residual_east), abs(residual_down)


__all__ = [
    "calc_distance",
    "calc_h_v_dist",
    "closest_horizontal_on_segment",
    "closest_on_segment",
    "closest_point_components_on_segment",
    "is_valid_location",
]
