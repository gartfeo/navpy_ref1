"""Mission rewrite operations used by wind-aligned legacy navigation."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from geopy.distance import geodesic
from geopy.point import Point
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_LOITER_TO_ALT,
    MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAVLink_mission_item_message,
)
from pymavlink.mavextra import wrap_360

from navpy.args.mission_planner_args import MissionPlannerArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.flight_mode import FlightMode


class MissionApproachVehicle(Protocol):
    target_system: int
    mission_items_count: int

    def get_param_or_default(self, name: str, default: float) -> float: ...
    def get_mission_item_location(self, sequence: int) -> Location: ...
    def get_mission_item(
        self,
        sequence: int,
    ) -> MAVLink_mission_item_message: ...
    def update_mission_item(
        self,
        sequence: int,
        command: MAVLink_mission_item_message,
    ) -> None: ...
    def upload_mission(self) -> bool: ...
    def restart_mission(self, mission_index: int = 0) -> None: ...
    def set_mode(self, flight_mode: FlightMode) -> bool: ...


class MissionApproachWriter:
    """Build and upload loiter or directional approach waypoints."""

    def __init__(self, logger: ILogger, args: MissionPlannerArgs) -> None:
        self._logger = logger
        self._args = args

    def plan_loiter(
        self,
        vehicle: MissionApproachVehicle,
        wind_bearing: float,
        current: Location,
        target: Location,
        distance: float,
    ) -> bool:
        start, wind_sign = self._start_location(
            current,
            distance,
            target,
            wind_bearing,
        )
        if self._args.plan_loiter_wind_direction:
            loiter = start
        else:
            radius = self._args.loiter_coefficient * vehicle.get_param_or_default(
                "WP_LOITER_RAD",
                90.0,
            )
            loiter = geodesic(meters=wind_sign * radius).destination(
                point=(start.latitude, start.longitude),
                bearing=wrap_360(wind_bearing - 90),
            )
        altitude_location = loiter if self._args.alt_loc_same else start
        altitude = vehicle.get_mission_item_location(1).alt
        waypoints = (
            self._waypoint(vehicle, MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 50.0),
            self._waypoint(
                vehicle,
                MAV_CMD_NAV_LOITER_TO_ALT,
                altitude_location.latitude,
                altitude_location.longitude,
                altitude,
            ),
            self._waypoint(
                vehicle,
                MAV_CMD_NAV_LOITER_UNLIM,
                loiter.latitude,
                loiter.longitude,
                altitude,
            ),
            vehicle.get_mission_item(vehicle.mission_items_count - 1),
        )
        self._upload(vehicle, waypoints, restart_index=1)
        self._logger.info(
            f"start_location: {start.latitude}, {start.longitude}"
        )
        self._logger.info(
            f"loiter_location: {loiter.latitude}, {loiter.longitude}"
        )
        return True

    def plan_direction(
        self,
        vehicle: MissionApproachVehicle,
        bearing: float,
        current: Location,
        target: Location,
        distance: float,
    ) -> bool:
        start, wind_sign = self._start_location(
            current,
            distance,
            target,
            bearing,
        )
        altitude = vehicle.get_mission_item_location(1).alt
        radius = vehicle.get_param_or_default("WP_LOITER_RAD", 90.0)
        direction = geodesic(meters=wind_sign * radius).destination(
            point=(start.latitude, start.longitude),
            bearing=wrap_360(bearing - 180),
        )
        detect = geodesic(
            meters=wind_sign * self._args.dir_distance,
        ).destination(
            point=(start.latitude, start.longitude),
            bearing=wrap_360(bearing - 180),
        )
        waypoints = (
            self._waypoint(
                vehicle,
                MAV_CMD_NAV_LOITER_TO_ALT,
                start.latitude,
                start.longitude,
                altitude,
            ),
            self._waypoint(
                vehicle,
                MAV_CMD_NAV_WAYPOINT,
                direction.latitude,
                direction.longitude,
                altitude,
            ),
            self._waypoint(
                vehicle,
                MAV_CMD_NAV_WAYPOINT,
                detect.latitude,
                detect.longitude,
                altitude,
            ),
            vehicle.get_mission_item(vehicle.mission_items_count - 1),
        )
        self._upload(vehicle, waypoints, restart_index=0)
        self._logger.info(
            f"start_location: {start.latitude}, {start.longitude}"
        )
        self._logger.info(
            f"loiter_location: {direction.latitude}, {direction.longitude}"
        )
        self._logger.info(
            f"detect_location: {detect.latitude}, {detect.longitude}"
        )
        return True

    def _start_location(
        self,
        current: Location,
        distance: float,
        target: Location,
        wind_bearing: float,
    ) -> tuple[Point, int]:
        if self._args.wind_dir is not None:
            wind_sign = 1 if self._args.wind_dir is True else -1
            return (
                geodesic(meters=wind_sign * distance).destination(
                    point=(target.lat, target.lng),
                    bearing=wind_bearing,
                ),
                wind_sign,
            )

        opposite = geodesic(meters=-distance).destination(
            point=(target.lat, target.lng),
            bearing=wind_bearing,
        )
        aligned = geodesic(meters=distance).destination(
            point=(target.lat, target.lng),
            bearing=wind_bearing,
        )
        opposite_distance = geodesic().measure(
            (current.lat, current.lng),
            (opposite.latitude, opposite.longitude),
        )
        aligned_distance = geodesic().measure(
            (current.lat, current.lng),
            (aligned.latitude, aligned.longitude),
        )
        return (opposite, -1) if opposite_distance < aligned_distance else (aligned, 1)

    @staticmethod
    def _waypoint(
        vehicle: MissionApproachVehicle,
        command: int,
        lat: float,
        lng: float,
        altitude: float,
    ) -> MAVLink_mission_item_message:
        return MAVLink_mission_item_message(
            vehicle.target_system,
            0,
            0,
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command,
            0,
            0,
            15 if command == MAV_CMD_NAV_TAKEOFF else 0,
            0,
            0,
            0,
            lat * 1e7,
            lng * 1e7,
            altitude,
        )

    @staticmethod
    def _upload(
        vehicle: MissionApproachVehicle,
        waypoints: Sequence[MAVLink_mission_item_message],
        *,
        restart_index: int,
    ) -> None:
        for index, waypoint in enumerate(waypoints):
            vehicle.update_mission_item(index, waypoint)
        vehicle.upload_mission()
        vehicle.restart_mission(restart_index)
        vehicle.set_mode(FlightMode.AUTO)
