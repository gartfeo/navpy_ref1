"""Default delivery hub mission navigation and pass classification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.nav.approach_planner import ApproachPlanner
from navpy.modules.nav.mission_catalog import MissionCatalog
from navpy.modules.nav.nav_state import NavigationTaskState
from navpy.modules.nav.pass_tracker import LegacyPassTracker
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
    "FallbackMissionNavigation",
    "FallbackMissionPorts",
    "MissionPassPolicy",
    "MissionPassPorts",
]
