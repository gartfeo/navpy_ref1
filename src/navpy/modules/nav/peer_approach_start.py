"""Peer assignment preparation and transactional approach dispatch."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.nav.nav_state import NavigationTaskState, GeoHoldState
from navpy.modules.nav.peer_geo import PeerGeoTracker


class PeerSimulationPoiSetup:
    """Seed a simulator POI only for a simulated detector session."""

    def __init__(
        self,
        is_simulation: Callable[[], bool],
        set_sim_poi: Callable[[int, Location], None],
        mission_item_count: Callable[[], int],
    ) -> None:
        self._is_simulation = is_simulation
        self._set_sim_poi = set_sim_poi
        self._mission_item_count = mission_item_count

    def apply(self, poi: Location) -> None:
        if self._is_simulation():
            self._set_sim_poi(
                self._mission_item_count() - 2,
                Location(poi.lat, poi.lng, 0),
            )


@dataclass(frozen=True)
class PeerAssignmentPorts:
    selected_poi: Callable[[], TaskAssignMsgData | None]
    absolute_location: Callable[[Location | None], Location | None]


class PeerAssignmentSetup:
    """Translate one accepted task into peer navigation state."""

    def __init__(
        self,
        ports: PeerAssignmentPorts,
        navigation_task: NavigationTaskState,
        geo_hold: GeoHoldState,
        simulation: PeerSimulationPoiSetup,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._geo_hold = geo_hold
        self._simulation = simulation

    def prepare(self) -> tuple[TaskAssignMsgData, Location] | None:
        selected = self._ports.selected_poi()
        if selected is None:
            return None
        self._navigation_task.peer_navigation = True
        self._navigation_task.peer_poi_location = None
        self._geo_hold.poi_location = None
        self._geo_hold.acquisition_log_bucket = None
        location = selected.location
        navigation_location = Location(
            location.lat,
            location.lng,
            location.alt,
            is_absolute=True,
        )
        self._navigation_task.navigation_poi_location = (
            self._ports.absolute_location(navigation_location)
        )
        self._simulation.apply(navigation_location)
        return selected, navigation_location


@dataclass(frozen=True)
class PeerApproachPorts:
    current_absolute: Callable[[], Location | None]
    plan_orbit: Callable[..., tuple[ApproachPlan, float | None]]
    request_guided: Callable[[], None]
    dispatch_approach: Callable[..., None]
    save_loiter_radius: Callable[[], bool]


class PeerApproachStarter:
    """Plan and dispatch one peer approach with geo-arm rollback."""

    def __init__(
        self,
        ports: PeerApproachPorts,
        navigation_task: NavigationTaskState,
        geo_tracker: PeerGeoTracker,
        final_approach_active: Callable[[], bool],
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._geo_tracker = geo_tracker
        self._final_approach_active = final_approach_active
        self._logger = logger

    def start(
        self,
        selected: TaskAssignMsgData,
        navigation_location: Location,
    ) -> None:
        plan, loiter_alt = self._ports.plan_orbit(
            navigation_location,
            selected.class_id,
            self._ports.current_absolute(),
        )
        self._navigation_task.orbit_approach_alt_rel_m = loiter_alt
        self._navigation_task.peer_approach_distance_m = plan.offset_distance
        self._navigation_task.orbit_radius_m = plan.orbit_radius or 0.0
        if plan.orbit_radius and not self._ports.save_loiter_radius():
            self._logger.warning(
                f"Peer-nav {plan.kind.value}: WP_LOITER_RAD save failed; "
                "radius will not be restored when the navigation task ends",
                key="nav",
            )
        self._ports.request_guided()
        geo_armed = False
        if plan.kind == ApproachKind.ORBIT and not self._final_approach_active():
            geo_armed = self._geo_tracker.start(navigation_location)
            if geo_armed:
                geo_armed = self._geo_tracker.prime()
        try:
            self._ports.dispatch_approach(plan, loiter_alt_rel=loiter_alt)
        except Exception:
            if geo_armed:
                self._geo_tracker.stop()
            raise
        orbit = (
            f", orbit_r={plan.orbit_radius:.0f}m"
            if plan.orbit_radius
            else ""
        )
        self._logger.info(
            f"Peer navigation started ({plan.kind.value}, "
            f"offset={plan.offset_distance:.0f}m{orbit}).",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = [
    "PeerApproachPorts",
    "PeerApproachStarter",
    "PeerAssignmentPorts",
    "PeerAssignmentSetup",
    "PeerSimulationPoiSetup",
]
