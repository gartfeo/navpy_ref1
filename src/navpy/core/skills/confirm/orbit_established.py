"""Orbit-established gate before confirm and NAV (HITL step 25).

The orbit counts as established once the aircraft has stayed within the radial tolerance of the
circle commanded on the own-detected centre for the dwell, on the navigation decision clock. It
reads only the aircraft's home-relative GPS position (allowed before NAV, owner ruling) and the
horizontal distance to that centre: no heading, no altitude, no ground speed and no truth. Outside
the band, a missing or non-finite position, or a gap between observations over
``ORBIT_MAX_OBSERVATION_GAP_S`` restarts the dwell.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel.location import Location
from navpy.core.skills.confirm.acquisition_gate_settings import AcquisitionGateSettings

# The navigation loop observes every tick (10 Hz or faster); a second without one is not "staying" on the
# circle, it is not looking.
ORBIT_MAX_OBSERVATION_GAP_S = 1.0


@dataclass(frozen=True)
class OrbitCommand:
    """The circle commanded on the own-detected centre (home-relative)."""

    centre: Location
    radius_m: float


class OrbitEstablished:
    """Track the dwell on the commanded circle."""

    def __init__(
        self,
        settings: AcquisitionGateSettings,
        current_relative: Callable[[], Location | None],
        now_s: Callable[[], float],
    ) -> None:
        self._settings = settings
        self._current_relative = current_relative
        self._now_s = now_s
        self.reset()

    def reset(self) -> None:
        self._command: OrbitCommand | None = None
        self._last_s: float | None = None
        self._since_s: float | None = None

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
        if not self._on_circle(command):
            self._since_s = None
            return False
        if self._since_s is None:
            self._since_s = now_s
        return now_s - self._since_s >= self._settings.orbit_dwell_s

    def _on_circle(self, command: OrbitCommand) -> bool:
        location = self._current_relative()
        if location is None:
            return False
        # Horizontal only (haversine on latitude and longitude), so the altitude reference of either
        # side does not enter.
        values = (location.lat, location.lng, command.centre.lat, command.centre.lng)
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            return False
        radial_error_m = abs(location.distance_to(command.centre) - command.radius_m)
        return radial_error_m <= self._settings.orbit_radial_tolerance_m


__all__ = ["ORBIT_MAX_OBSERVATION_GAP_S", "OrbitCommand", "OrbitEstablished"]
