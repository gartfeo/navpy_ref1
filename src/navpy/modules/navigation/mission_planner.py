"""Wind-aware delivery-approach mission planning policy.

This legacy waypoint planner is not a complete moving-recipient rendezvous
planner: authorization, uncertainty, separation and handover need validation.
"""

from __future__ import annotations

from geopy.distance import geodesic
from pymavlink.mavextra import wrap_180

from navpy.args.mission_planner_args import MissionPlannerArgs
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.mission_approach_writer import MissionApproachWriter
from navpy.modules.navigation.peer_poi_mission_writer import PeerPoiMissionWriter
from navpy.modules.navigation.wind_feed_forward import WindFeedForwardCalculator
from navpy.modules.vehicle.vehicle_interface import IVehicle


class MissionPlanner:
    """Choose whether and how legacy mission navigation should be rewritten."""

    def __init__(self, logger, args: MissionPlannerArgs) -> None:
        self._logger = logger
        self._args = args
        self._approach_writer = MissionApproachWriter(logger, args)
        self._peer_writer = PeerPoiMissionWriter()
        self._feed_forward = WindFeedForwardCalculator(args.min_air_speed)

    def adjust_nav_bearing(self, vehicle: IVehicle, wind_bearing):
        poi_index = vehicle.mission_items_count - 1
        poi = vehicle.get_mission_item_location(poi_index)
        current = vehicle.location(True)
        if wind_bearing is None:
            loiter_alt_index = vehicle.mission_items_count - 3
            current = vehicle.get_mission_item_location(loiter_alt_index)
            return self.adjust_nav(
                vehicle,
                vehicle.attitude.yaw,
                current,
                poi,
            )
        return self.plan_loiter(
            vehicle,
            wind_bearing,
            current,
            poi,
            self._args.custom_start_distance,
        )

    def adjust_nav(
        self,
        vehicle: IVehicle,
        target_bearing,
        current: Location,
        poi: Location,
    ):
        if not self._args.mp_enable:
            self._logger.info("Mission Planner disabled")
            return None

        wind = vehicle.wind
        if wind is None:
            self._logger.info("Wind is not available")
            return None
        if wind.speed < self._args.min_air_speed:
            self._logger.info(
                f"Wind planning skipped speed is too low: {wind.speed} m/s"
            )
            return False
        if wind.direction is None:
            self._logger.info("Wind bearing is not available")
            return None

        bearing_difference = abs(wrap_180(wind.direction - target_bearing))
        if bearing_difference < self._args.aligned_delta and (
            self._args.wind_dir is None or self._args.wind_dir is False
        ):
            self._logger.info(
                "Wind planning skipped. Opposite wind bearing: "
                f"{bearing_difference} deg."
            )
            return True
        if bearing_difference > 180 - self._args.aligned_delta and (
            self._args.wind_dir is None or self._args.wind_dir is True
        ):
            self._logger.info(
                "Wind planning skipped. Aligned with wind bearing: "
                f"{bearing_difference} deg."
            )
            return True

        self._logger.info(
            f"Mission Planner: Wind: {wind.direction} deg, {wind.speed} m/s/"
        )
        distance = self._args.custom_start_distance
        if distance is None:
            distance = geodesic().measure(
                (current.lat, current.lng),
                (poi.lat, poi.lng),
            ) * 1000

        if self._args.plan_loiter:
            return self.plan_loiter(
                vehicle,
                wind.direction,
                current,
                poi,
                distance,
            )
        return self.plan_dir(
            vehicle,
            wind.direction,
            current,
            poi,
            distance,
        )

    def plan_loiter(
        self,
        vehicle: IVehicle,
        wind_bearing,
        current: Location,
        poi: Location,
        distance,
    ) -> bool:
        return self._approach_writer.plan_loiter(
            vehicle,
            wind_bearing,
            current,
            poi,
            distance,
        )

    def plan_dir(
        self,
        vehicle: IVehicle,
        bearing,
        current: Location,
        poi: Location,
        distance,
    ) -> bool:
        return self._approach_writer.plan_direction(
            vehicle,
            bearing,
            current,
            poi,
            distance,
        )

    def calculate_ff_coefficients(
        self,
        vehicle: IVehicle,
    ) -> tuple[float, float]:
        return self._feed_forward.calculate(vehicle)

    def peer_poi(self, vehicle: IVehicle, peer_poi: Location) -> None:
        self._peer_writer.plan(vehicle, peer_poi)
