"""Circle a locally detected POI while it waits for confirmation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.location import Location
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.flow.nav_state import NavigationTaskState
from navpy.core.ports.detect_data import DetectedObject

# The waiting circle takes the POI to sit on the terrain: a few metres of real
# height move the circle centre by a few metres, well inside the circle.
WAITING_CIRCLE_REFERENCE_HEIGHT_M = 0.0


@dataclass(frozen=True)
class SelfDetectedApproachPorts:
    final_approach_active: Callable[[], bool]
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]
    current_relative: Callable[[], Optional[Location]]
    plan_orbit: Callable[..., tuple]
    absolute_location: Callable[[Optional[Location]], Optional[Location]]
    save_loiter_radius: Callable[[], bool]
    request_guided: Callable[[], None]
    loiter_poi: Callable[[Location, float, Optional[float]], None]


class SelfDetectedApproach:
    """Command a POI-centred ORBIT for a local POI until it is confirmed."""

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

    def prepare(self, poi: DetectedObject) -> None:
        if self._navigation_task.orbit_radius_m > 0:
            return
        if self._approach_kind != ApproachKind.ORBIT:
            return
        # Circle the POI until it is confirmed (owner, 2026-10-07), so it
        # stays inside the turn and in view. Under the vision law only the
        # camera's ground estimate centres the circle, and it is not stored
        # where the approach could read it. No sim truth: not the POI's
        # position and not its height (the POI is taken to sit on the ground).
        final_approach = self._ports.final_approach_active()
        poi_location = (
            self._ports.ground_location(
                poi,
                allow_fallback=False,
                reference_height_m=WAITING_CIRCLE_REFERENCE_HEIGHT_M,
            )
            if final_approach
            else self.poi_location(poi)
        )
        if poi_location is None:
            if final_approach:
                self._logger.warning(
                    "Self-detect orbit skipped: no ground estimate for the POI",
                    key="nav",
                )
            return
        plan, loiter_alt = self._ports.plan_orbit(
            poi_location,
            poi.classification.class_id,
            self._ports.current_relative(),
        )
        if not plan.orbit_radius or plan.orbit_radius <= 0:
            return
        if not final_approach:
            self._navigation_task.navigation_poi_location = (
                self._ports.absolute_location(poi_location)
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
        self._ports.loiter_poi(
            poi_location,
            self._navigation_task.orbit_radius_m,
            loiter_alt,
        )
        self._logger.info(
            f"Self-detect orbit: poi={poi_location} "
            f"orbit_r={self._navigation_task.orbit_radius_m:.0f}m",
            key="nav",
        )

    def poi_location(self, poi: DetectedObject) -> Optional[Location]:
        if (
            self._ports.detector_is_simulation()
            and poi.geo.truth_poi_location is not None
        ):
            return poi.geo.truth_poi_location
        return self._ports.ground_location(poi, allow_fallback=False)


__all__ = ["SelfDetectedApproach", "SelfDetectedApproachPorts"]
