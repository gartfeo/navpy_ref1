"""Orbit-established gate before confirm and NAV (HITL step 25).

The orbit counts as established once the aircraft has stayed within the radial tolerance of the
circle commanded on the own-detected centre for the dwell, on the navigation decision clock. It
reads only the aircraft's stamped GPS fix (``vehicle.position_sample``: latitude, longitude and the
autopilot's boot time; GPS is allowed before NAV, owner ruling) and the horizontal distance to that
centre: no heading, no altitude, no ground speed and no truth.

A fix counts only while it is new: the autopilot's boot time must advance, so a cached position
cannot hold the circle. The dwell restarts when the aircraft is outside the band, the fix is
missing or not finite, the boot time goes back, two fixes are more than
``ORBIT_MAX_OBSERVATION_GAP_S`` apart in boot time, no new fix has come for that long on the
decision clock, or two observations are that far apart.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from navpy.core.kernel.location import Location
from navpy.core.skills.confirm.acquisition_gate_settings import AcquisitionGateSettings

# The navigation loop observes every tick (10 Hz or faster) and ArduPilot streams GLOBAL_POSITION_INT
# at several Hz; a second without either is not "staying" on the circle, it is not looking.
ORBIT_MAX_OBSERVATION_GAP_S = 1.0


@dataclass(frozen=True)
class OrbitCommand:
    """The circle commanded on the own-detected centre."""

    centre: Location
    radius_m: float


def _finite(*values: Any) -> bool:
    return all(
        isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
        for value in values
    )


class OrbitEstablished:
    """Track the dwell on the commanded circle."""

    def __init__(
        self,
        settings: AcquisitionGateSettings,
        current_fix: Callable[[], Any],
        now_s: Callable[[], float],
    ) -> None:
        self._settings = settings
        self._current_fix = current_fix
        self._now_s = now_s
        self.reset()

    def reset(self) -> None:
        self._command: OrbitCommand | None = None
        self._last_s: float | None = None
        self._since_s: float | None = None
        self._fix_boot_s: float | None = None
        self._fix_seen_s: float | None = None
        self._on_circle = False

    @property
    def commanded(self) -> bool:
        return self._command is not None

    def start(self, command: OrbitCommand) -> None:
        self.reset()
        self._command = command

    def established(self) -> bool:
        """Observe the aircraft now and say whether it has held the circle for the dwell."""
        command = self._command
        if command is None:
            return False
        now_s = float(self._now_s())
        if self._last_s is not None and not 0.0 <= now_s - self._last_s <= ORBIT_MAX_OBSERVATION_GAP_S:
            self._since_s = None
        self._last_s = now_s
        if not self._take_fix(command, now_s):
            self._since_s = None
            return False
        if self._since_s is None:
            self._since_s = now_s
        return now_s - self._since_s >= self._settings.orbit_dwell_s

    def _take_fix(self, command: OrbitCommand, now_s: float) -> bool:
        """Take this tick's fix; True while the newest fix is recent and on the circle."""
        fix = self._current_fix()
        lat, lon, boot_s = (getattr(fix, name, None) for name in ("lat", "lon", "time_boot_s"))
        if fix is None or not _finite(lat, lon, boot_s):
            self._fix_boot_s = self._fix_seen_s = None
            return False
        last_boot_s = self._fix_boot_s
        if last_boot_s is None or boot_s > last_boot_s:
            if last_boot_s is not None and boot_s - last_boot_s > ORBIT_MAX_OBSERVATION_GAP_S:
                self._since_s = None
            self._fix_boot_s, self._fix_seen_s = float(boot_s), now_s
            self._on_circle = self._radial_error_m(command, lat, lon) <= self._settings.orbit_radial_tolerance_m
        elif boot_s < last_boot_s:  # the autopilot restarted its clock: no fix history to trust
            self._fix_boot_s = self._fix_seen_s = None
            return False
        if self._fix_seen_s is None or now_s - self._fix_seen_s > ORBIT_MAX_OBSERVATION_GAP_S:
            return False  # the same fix for too long: a cached position, not the aircraft now
        return self._on_circle

    @staticmethod
    def _radial_error_m(command: OrbitCommand, lat: float, lon: float) -> float:
        # Horizontal only (haversine on latitude and longitude), so no altitude reference enters.
        centre = command.centre
        if not _finite(centre.lat, centre.lng):
            return math.inf
        here = Location(float(lat), float(lon), 0.0, is_absolute=centre.is_absolute)
        return abs(here.distance_to(centre) - command.radius_m)


__all__ = ["ORBIT_MAX_OBSERVATION_GAP_S", "OrbitCommand", "OrbitEstablished"]
