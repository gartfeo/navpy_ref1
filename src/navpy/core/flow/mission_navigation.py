"""Default delivery hub mission navigation and pass classification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.location import Location
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.flow.nav_state import NavigationTaskState
from navpy.core.skills.geo_approach.pass_tracker import LegacyPassTracker


@dataclass(frozen=True)
class MissionPassPorts:
    final_approach_active: Callable[[], bool]
    poi_passed_override: Callable[[], Optional[bool]]
    last_visual_bearing_deg: Callable[[], Optional[float]]
    current_absolute: Callable[[], Optional[Location]]
    locked_poi_distance: Callable[[], Optional[float]]


class MissionPassPolicy:
    """Classify mission waypoint and POI-pass progress."""

    def __init__(
        self,
        ports: MissionPassPorts,
        args: NavArgs,
        navigation_task: NavigationTaskState,
        pass_tracker: LegacyPassTracker,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._args = args
        self._navigation_task = navigation_task
        self._pass_tracker = pass_tracker
        self._logger = logger

    def passed_detection_waypoint(self, next_waypoint: int) -> bool:
        return next_waypoint > self._args.min_wp

    def passed_poi(self) -> bool:
        navigation_passed = self._ports.poi_passed_override()
        if self._ports.final_approach_active():
            return navigation_passed is True
        if isinstance(navigation_passed, bool):
            return navigation_passed
        poi = self._navigation_task.navigation_poi_location
        if poi is not None:
            current = self._ports.current_absolute()
            distance = (
                None
                if current is None
                else GeoRefCalc.calculate_distance(current, poi)
            )
        else:
            distance = self._ports.locked_poi_distance()
        if distance is None:
            return False
        observation = self._pass_tracker.observe(distance)
        if observation.entered_close_basin:
            self._logger.info(
                f"PASS GATE: entered coordinate basin dist={distance:.1f}m "
                f"poi={poi}",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
        return observation.passed

    def has_completion_evidence(self) -> bool:
        if self._ports.final_approach_active():
            return self._ports.poi_passed_override() is True
        return self._pass_tracker.has_crossing_evidence(
            self._ports.last_visual_bearing_deg()
        )

    @property
    def close_observed(self) -> bool:
        return self._pass_tracker.close_observed

    def reset(self) -> None:
        self._pass_tracker.reset()


__all__ = [
    "MissionPassPolicy",
    "MissionPassPorts",
]
