"""Geometry of the sim dock contact test: flat dock frame and path bow.

The contact sensor runs on the telemetry reader thread, so it first checks
each sample in a flat local frame at the dock (a few float operations) and
runs the exact geodetic test only within reach of the net.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.core.kernel.location import Location
from navpy.core.kernel.math_utils import GRAVITY_MSS
from navpy.sim.world.simulation_object import SimulationObject

# Steepest bank the chord-bow bound assumes (ArduPlane ROLL_LIMIT_DEG
# default). The flown path between two samples can bow away from their chord
# by at most the sagitta of a turn at this bank, so contact needs
# closest distance + that bow < radius.
SIM_DOCK_CONTACT_BANK_LIMIT_DEG = 45.0

# WGS84, for the dock's local radii of curvature.
_WGS84_A_M = 6_378_137.0
_WGS84_E2 = 6.694_379_990_14e-3
# Smallest WGS84 radius of curvature (the meridional radius at the equator).
_WGS84_MIN_RADIUS_M = _WGS84_A_M * (1.0 - _WGS84_E2)
# Rounding allowance on top of the flat-earth bound (float and 1e-7 deg
# telemetry quantisation are both far below it).
SIM_DOCK_CONTACT_FLAT_EARTH_FLOOR_M = 1e-3


def chord_bow_bound_m(chord_m: float, speed_mps: float) -> float:
    """Largest distance the flown path can stray from the sample chord.

    At ``speed_mps`` and no steeper than the bank limit, the aircraft turns
    on a radius of at least v^2 / (g tan(bank)); over the chord its path
    bows by at most that circle's sagitta. A radius shorter than half the
    chord (not flyable) bounds the bow by half the chord.
    """
    half_chord_m = chord_m / 2.0
    radius_m = speed_mps ** 2 / (
        GRAVITY_MSS * math.tan(math.radians(SIM_DOCK_CONTACT_BANK_LIMIT_DEG))
    )
    if radius_m < half_chord_m:
        return half_chord_m
    return radius_m - math.sqrt(radius_m ** 2 - half_chord_m ** 2)


def flat_earth_error_bound_m(
    reach_m: float,
    abs_tan_lat: float,
    abs_height_m: float,
) -> float:
    """How far the flat local frame can disagree with the exact geometry.

    The coarse filter works in a flat frame at the dock, scaled by WGS84's
    local surface radii of curvature, so first-order scale error cancels on
    the ellipsoid. Points at height h above it sit on radii larger by h, so
    their horizontal distances scale by up to |h| / R: at most
    reach * |h| / R_min, with ``abs_height_m`` the largest |h| involved. What
    remains for points within ``reach_m`` of each other near the dock is
    second order: earth curvature drops the surface by reach^2 / (2R), and
    the east scale changes with latitude by reach * tan(lat) / R per metre.
    Both stay below reach^2 * (1 + |tan lat|) / R_min. A rounding floor is
    added on top.
    """
    return (
        reach_m * (reach_m * (1.0 + abs_tan_lat) + abs_height_m) / _WGS84_MIN_RADIUS_M
        + SIM_DOCK_CONTACT_FLAT_EARTH_FLOOR_M
    )


@dataclass(frozen=True)
class DockFrame:
    """The armed dock's flat local frame, computed once per dock object."""

    dock: SimulationObject
    lat_deg: float
    lon_deg: float
    alt_m: float
    north_m_per_deg: float
    east_m_per_deg: float
    abs_tan_lat: float

    @classmethod
    def at(cls, dock: SimulationObject) -> "DockFrame":
        lat = math.radians(dock.g_loc.lat)
        sin2 = math.sin(lat) ** 2
        meridional_m = _WGS84_A_M * (1.0 - _WGS84_E2) / (1.0 - _WGS84_E2 * sin2) ** 1.5
        normal_m = _WGS84_A_M / math.sqrt(1.0 - _WGS84_E2 * sin2)
        return cls(
            dock,
            dock.g_loc.lat,
            dock.g_loc.lng,
            dock.g_loc.alt,
            math.radians(meridional_m),
            math.radians(normal_m * math.cos(lat)),
            abs(math.tan(lat)),
        )

    def local(self, location: Location) -> tuple[float, float, float]:
        return (
            (location.lat - self.lat_deg) * self.north_m_per_deg,
            (location.lng - self.lon_deg) * self.east_m_per_deg,
            location.alt - self.alt_m,
        )


def flat_segment_distance(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float:
    """Distance from the frame origin (the dock) to the flat segment."""
    dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
    length2 = dx * dx + dy * dy + dz * dz
    fraction = 0.0
    if length2 > 0.0:
        fraction = -(start[0] * dx + start[1] * dy + start[2] * dz) / length2
        fraction = min(1.0, max(0.0, fraction))
    return math.sqrt(
        (start[0] + fraction * dx) ** 2
        + (start[1] + fraction * dy) ** 2
        + (start[2] + fraction * dz) ** 2
    )


__all__ = [
    "DockFrame",
    "SIM_DOCK_CONTACT_BANK_LIMIT_DEG",
    "SIM_DOCK_CONTACT_FLAT_EARTH_FLOOR_M",
    "chord_bow_bound_m",
    "flat_earth_error_bound_m",
    "flat_segment_distance",
]
