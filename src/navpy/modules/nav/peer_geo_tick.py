"""Non-terminal peer geo pre-acquisition tick."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.nav.nav_state import NavigationTaskState, GeoHoldState
from navpy.modules.nav.peer_geo import PeerGeoAcquisition


@dataclass(frozen=True)
class DetectPointingPorts:
    is_detection_armed: Callable[[], bool]
    prepare_geo_acquisition: Callable[[Location, Attitude, int, float], bool]
    update_geo: Callable[[Location, Attitude], None]


class PeerGeoTick:
    """Update peer geo pointing only while visual detection is unarmed."""

    def __init__(
        self,
        navigation_task: NavigationTaskState,
        geo_hold: GeoHoldState,
        pointing: DetectPointingPorts,
        acquisition: PeerGeoAcquisition,
        terminal_active: Callable[[], bool],
        current_location: Callable[[], Location | None],
        current_attitude: Callable[[], Attitude | None],
    ) -> None:
        self._navigation_task = navigation_task
        self._geo_hold = geo_hold
        self._pointing = pointing
        self._acquisition = acquisition
        self._terminal_active = terminal_active
        self._current_location = current_location
        self._current_attitude = current_attitude

    def update(self) -> None:
        if (
            self._terminal_active()
            or not self._navigation_task.peer_navigation
            or self._geo_hold.target_location is None
            or self._pointing.is_detection_armed()
        ):
            return
        location = self._current_location()
        attitude = self._current_attitude()
        if location is None or attitude is None:
            return
        state = self._acquisition.snapshot(location, attitude)
        if state is not None:
            self._acquisition.log(state)
            self._pointing.prepare_geo_acquisition(
                location,
                attitude,
                state.class_id,
                state.min_pixels,
            )
        self._pointing.update_geo(location, attitude)


__all__ = ["DetectPointingPorts", "PeerGeoTick"]
