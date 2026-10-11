"""Default delivery hub (DDH) fallback mission navigation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.nav.approach_planner import ApproachPlanner
from navpy.modules.nav.mission_catalog import MissionCatalog
from navpy.modules.nav.nav_state import NavigationTaskState
from navpy.modules.nav.vehicle_navigation import VehicleNavigationCommands
from navpy.modules.vision.vision_class_profile import DOCK_DETECT_CLASS_ID


@dataclass(frozen=True)
class FallbackMissionPorts:
    next_waypoint: Callable[[], int]
    mission_item_count: Callable[[], int]
    current_relative: Callable[[], Optional[Location]]
    set_sim_poi: Callable[..., None]
    detector_is_simulation: Callable[[], bool]


class FallbackMissionNavigation:
    """Route the aircraft to the appended default delivery hub."""

    def __init__(
        self,
        ports: FallbackMissionPorts,
        catalog: MissionCatalog,
        navigation_task: NavigationTaskState,
        approach_planner: ApproachPlanner,
        commands: VehicleNavigationCommands,
        approach_kind: ApproachKind,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._catalog = catalog
        self._navigation_task = navigation_task
        self._approach_planner = approach_planner
        self._commands = commands
        self._approach_kind = approach_kind
        self._logger = logger

    def should_nav_to_fallback(self) -> bool:
        if (
            self._catalog.default_delivery_hub is None
            or self._catalog.default_delivery_hub_active
        ):
            return False
        return self._ports.next_waypoint() >= self._ports.mission_item_count() - 1

    def setup(self) -> None:
        poi = self._catalog.default_delivery_hub
        self._catalog.mark_fallback_active()
        self._navigation_task.navigation_poi_location = (
            self._commands.absolute_location(poi)
        )
        if self._ports.detector_is_simulation():
            self._ports.set_sim_poi(
                self._ports.mission_item_count() - 1,
                poi,
                location_type=self._catalog.default_delivery_hub_type,
            )
        plan, loiter_alt = self.plan_orbit_approach(
            poi,
            DOCK_DETECT_CLASS_ID,
            self._ports.current_relative(),
        )
        self._navigation_task.peer_approach_distance_m = plan.offset_distance
        self._navigation_task.orbit_radius_m = plan.orbit_radius or 0.0
        self._navigation_task.orbit_approach_alt_rel_m = loiter_alt
        if plan.orbit_radius and not self._commands.save_loiter_radius():
            self._logger.warning(
                f"Default delivery hub {plan.kind.value}: WP_LOITER_RAD save failed; "
                "radius will not be restored when the navigation task ends",
                key="nav",
            )
        self._commands.request_guided()
        self._commands.dispatch_approach(plan, loiter_alt_rel=loiter_alt)
        self._logger.info(
            f"Default delivery hub navigation: {poi} "
            f"({plan.kind.value}, offset={plan.offset_distance:.0f}m"
            f"{f', orbit_r={plan.orbit_radius:.0f}m' if plan.orbit_radius else ''})",
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def plan_orbit_approach(
        self,
        poi: Location,
        class_id: int,
        drone_location: Optional[Location],
    ) -> tuple[ApproachPlan, Optional[float]]:
        return self._approach_planner.plan(
            poi,
            class_id,
            drone_location,
            approach_kind=self._approach_kind,
            approach_alt_rel=self._catalog.scan_altitude_rel,
        )
