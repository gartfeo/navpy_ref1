"""Legacy ORBIT preparation for a locally detected target."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.nav_state import NavigationTaskState
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class SelfDetectedApproachPorts:
    terminal_active: Callable[[], bool]
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]
    current_relative: Callable[[], Optional[Location]]
    plan_orbit: Callable[..., tuple]
    absolute_location: Callable[[Optional[Location]], Optional[Location]]
    save_loiter_radius: Callable[[], bool]
    request_guided: Callable[[], None]
    loiter_target: Callable[[Location, float, Optional[float]], None]


class SelfDetectedApproach:
    """Prepare legacy target-centered ORBIT geometry for a local target."""

    def __init__(
        self,
        ports: SelfDetectedApproachPorts,
        navigation_task: NavigationTaskState,
        approach_kind: ApproachKind,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._approach_kind = approach_kind
        self._logger = logger

    def prepare(self, target: DetectedObject) -> None:
        if self._ports.terminal_active():
            return
        if self._navigation_task.orbit_radius_m > 0:
            return
        if self._approach_kind != ApproachKind.ORBIT:
            return
        target_location = self.target_location(target)
        if target_location is None:
            return
        plan, loiter_alt = self._ports.plan_orbit(
            target_location,
            target.classification.class_id,
            self._ports.current_relative(),
        )
        if not plan.orbit_radius or plan.orbit_radius <= 0:
            return
        self._navigation_task.navigation_target_location = (
            self._ports.absolute_location(target_location)
        )
        self._navigation_task.orbit_radius_m = plan.orbit_radius
        self._navigation_task.orbit_approach_alt_rel_m = loiter_alt
        if not self._ports.save_loiter_radius():
            self._logger.warning(
                "ORBIT host-detect: WP_LOITER_RAD save failed; orbit radius "
                "will not be restored when the navigation task ends",
                key="nav",
            )
        self._ports.request_guided()
        self._ports.loiter_target(
            target_location,
            self._navigation_task.orbit_radius_m,
            loiter_alt,
        )
        self._logger.info(
            f"Self-detect orbit: target={target_location} "
            f"orbit_r={self._navigation_task.orbit_radius_m:.0f}m",
            key="nav",
        )

    def target_location(self, target: DetectedObject) -> Optional[Location]:
        if (
            self._ports.detector_is_simulation()
            and target.geo.truth_target_location is not None
        ):
            return target.geo.truth_target_location
        return self._ports.ground_location(target, allow_fallback=False)


__all__ = ["SelfDetectedApproach", "SelfDetectedApproachPorts"]
