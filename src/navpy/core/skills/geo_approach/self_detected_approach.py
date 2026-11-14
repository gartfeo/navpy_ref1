"""Circle a locally detected POI while it waits for confirmation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.location import Location
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.flow.nav_state import NavigationTaskState
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.skills.geo_approach.board_orbit_centre import BoardOrbitCentre
from navpy.core.skills.confirm.orbit_established import OrbitCommand

# The waiting circle takes the POI to sit on the terrain: a few metres of real
# height move the circle centre by a few metres, well inside the circle.
WAITING_CIRCLE_REFERENCE_HEIGHT_M = 0.0


@dataclass(frozen=True)
class SelfDetectedApproachPorts:
    guidance_active: Callable[[], bool]
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]
    current_relative: Callable[[], Optional[Location]]
    plan_orbit: Callable[..., tuple]
    absolute_location: Callable[[Optional[Location]], Optional[Location]]
    save_loiter_radius: Callable[[], bool]
    request_guided: Callable[[], None]
    loiter_poi: Callable[[Location, float, Optional[float]], None]
    needs_own_orbit: Callable[[DetectedObject], bool]
    # The own board pose's centre (HITL step 22); None without a board pose.
    board_centre: Callable[[DetectedObject], Optional[BoardOrbitCentre]]


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

    def prepare(self, poi: DetectedObject) -> Optional[OrbitCommand]:
        """Command the waiting ORBIT; the circle it commanded, or None when it commanded none."""
        if self._navigation_task.orbit_radius_m > 0:
            if not self._ports.needs_own_orbit(poi):
                return None
            # The mission module says this dock needs its own circle: the
            # circle already flown would take it out of view.
            self._navigation_task.orbit_radius_m = 0.0
        if self._approach_kind != ApproachKind.ORBIT:
            return None
        # Circle the POI until it is confirmed (owner, 2026-10-07), so it
        # stays inside the turn and in view. Under the vision law only the
        # camera's own estimate centres the circle, and it is not stored
        # where the approach could read it. No sim truth: not the POI's
        # position and not its height.
        guidance = self._ports.guidance_active()
        poi_location = (
            self._vision_centre(poi)
            if guidance
            else self.poi_location(poi)
        )
        if poi_location is None:
            if guidance:
                self._logger.warning(
                    "Self-detect orbit skipped: no ground estimate for the POI",
                    key="nav",
                )
            return None
        plan, loiter_alt = self._ports.plan_orbit(
            poi_location,
            poi.classification.class_id,
            self._ports.current_relative(),
        )
        if not plan.orbit_radius or plan.orbit_radius <= 0:
            return None
        if not guidance:
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
        return OrbitCommand(poi_location, self._navigation_task.orbit_radius_m)

    def _vision_centre(self, poi: DetectedObject) -> Optional[Location]:
        """The board pose's centre; else the ground ray at height zero."""
        board = self._ports.board_centre(poi)
        if board is not None and board.location is not None:
            return board.location
        if board is not None:
            # A refused fit falls back to the ground ray, as before step 22.
            self._logger.warning(
                f"Self-detect orbit: board pose refused ({board.reason}); "
                "using the ground ray",
                key="nav",
            )
        return self._ports.ground_location(
            poi,
            allow_fallback=False,
            reference_height_m=WAITING_CIRCLE_REFERENCE_HEIGHT_M,
        )

    def poi_location(self, poi: DetectedObject) -> Optional[Location]:
        if (
            self._ports.detector_is_simulation()
            and poi.geo.truth_poi_location is not None
        ):
            return poi.geo.truth_poi_location
        return self._ports.ground_location(poi, allow_fallback=False)


__all__ = ["SelfDetectedApproach", "SelfDetectedApproachPorts"]
