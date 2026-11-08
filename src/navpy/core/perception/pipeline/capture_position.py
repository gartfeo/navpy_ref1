"""The camera's position at a frame's capture instant (HITL step 22).

The latest GLOBAL_POSITION_INT fix is moved along its own NED ground
velocity to the capture instant. Both stamps are on the autopilot's boot
clock (the fix's ``time_boot_ms`` and the capture-time ATTITUDE
interpolation), so link delay does not enter the shift. Acquisition
evidence only: the guidance command path never reads it.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Protocol

import pymap3d

from navpy.core.kernel.location import Location

# Longest shift trusted from one fix: a few fix periods at the 10 Hz or
# faster position stream; longer means a stale fix, not a delayed frame.
MAX_CAPTURE_POSITION_SHIFT_S = 0.5


class PositionSampleView(Protocol):
    lat: float
    lon: float
    alt: float
    time_boot_s: float
    vn: float
    ve: float
    vd: float


def capture_location(
    sample: PositionSampleView | None,
    capture_boot_s: float | None,
) -> Location | None:
    """Absolute camera location at ``capture_boot_s``, or None."""
    if sample is None or capture_boot_s is None:
        return None
    try:
        values = tuple(
            float(getattr(sample, name))
            for name in ("lat", "lon", "alt", "time_boot_s", "vn", "ve", "vd")
        )
        capture_s = float(capture_boot_s)
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values + (capture_s,)):
        return None
    lat, lon, alt, fix_s, vn, ve, vd = values
    shift_s = capture_s - fix_s
    if abs(shift_s) > MAX_CAPTURE_POSITION_SHIFT_S:
        return None
    latitude, longitude, altitude = pymap3d.ned2geodetic(
        vn * shift_s, ve * shift_s, vd * shift_s, lat, lon, alt,
    )
    return Location(
        float(latitude), float(longitude), float(altitude), is_absolute=True,
    )


def read_capture_location(
    reader: Callable[[], PositionSampleView | None] | None,
    capture_boot_s: float | None,
) -> Location | None:
    """``capture_location`` from a live reader; None when it has none or fails."""
    if reader is None:
        return None
    try:
        sample = reader()
    except Exception:
        return None
    return capture_location(sample, capture_boot_s)


__all__ = [
    "MAX_CAPTURE_POSITION_SHIFT_S",
    "PositionSampleView",
    "capture_location",
    "read_capture_location",
]
