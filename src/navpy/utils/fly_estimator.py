from __future__ import annotations

import math
from typing import Optional, Protocol

from navpy.modules.common.models.location import Location
from navpy.modules.common.models.wind import Wind
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc

# Constants
DRONE_MAX_FLIGHT_TIME_SEC = 60 * 60  # Maximum flight time in seconds (1 hour)
DRONE_SAFETY_MARGIN = 0.1  # 10% safety margin
BATTERY_FULL_LEVEL = 100  # Battery level at 100%
MIN_BATTERY_THRESHOLD = 20  # Minimum battery level to consider for flight (%)


class FlightEstimateTelemetry(Protocol):
    def location(self, is_relative: bool) -> Location | None: ...

    @property
    def ground_speed(self) -> float | None: ...

    @property
    def wind(self) -> Wind: ...

    @property
    def battery_level(self) -> float | None: ...


class FlyEstimator:
    """
    Estimates the time required for a vehicle to fly to a POI location,
    considering various factors like battery level, wind conditions, and vehicle specifications.
    """

    @staticmethod
    def time_to_fly(
            vehicle: FlightEstimateTelemetry,
            poi_location: Location
    ) -> Optional[float]:
        """
        Calculates the estimated time in minutes for the vehicle to fly to the POI location.

        :param vehicle: The vehicle instance implementing IDrone interface.
        :param poi_location: The POI location to which the vehicle needs to fly.
        :return: Estimated time in minutes, or None if the vehicle cannot make the flight safely.
        """
        vehicle_location = vehicle.location(False)
        if not vehicle_location or not poi_location:
            return -1

        # Calculate the straight-line distance to the POI in meters
        distance_m = GeoRefCalc.calculate_distance(vehicle_location, poi_location)

        # Get the vehicle's airspeed in m/s (without wind)
        airspeed_m_s = vehicle.ground_speed or 0.0
        if airspeed_m_s <= 0:
            return -1  # Cannot calculate time without valid airspeed

        # Get wind data from the vehicle
        wind_speed_m_s = vehicle.wind.speed or 0.0  # Wind speed in m/s
        wind_direction_deg = vehicle.wind.direction or 0.0  # Wind direction in degrees FROM which the wind is coming

        # Calculate the heading from the vehicle to the POI
        heading_to_poi_deg = GeoRefCalc.calculate_bearing(vehicle_location, poi_location)

        # Adjust airspeed for wind to get ground speed towards the POI
        adjusted_ground_speed_m_s = FlyEstimator.calculate_ground_speed(
            airspeed_m_s,
            heading_to_poi_deg,
            wind_speed_m_s,
            wind_direction_deg
        )
        if adjusted_ground_speed_m_s <= 0:
            return -1  # Vehicle cannot make progress towards the POI

        # Estimate the time to fly to the POI (in seconds)
        poi_time_to_fly_sec = distance_m / adjusted_ground_speed_m_s

        # Get the vehicle's battery level (%)
        battery_level = vehicle.battery_level

        if battery_level is None or battery_level < MIN_BATTERY_THRESHOLD:
            if battery_level is None or battery_level == 0:
                # the battery level is invalid or not available
                # calculate the estimated remaining flight time based on flight duration
                # TODO - add a more accurate way to estimate the remaining flight time
                remaining_time_to_fly_sec = DRONE_MAX_FLIGHT_TIME_SEC
            else:
                return -1  # Not enough battery to consider flying
        else:
            # Estimate remaining flight time based on battery level (in seconds)
            remaining_time_to_fly_sec = (battery_level / BATTERY_FULL_LEVEL) * DRONE_MAX_FLIGHT_TIME_SEC

        # Include safety margin
        remaining_time_with_margin_sec = remaining_time_to_fly_sec * (1 - DRONE_SAFETY_MARGIN)

        # Check if the vehicle can reach the POI location safely
        if poi_time_to_fly_sec > remaining_time_with_margin_sec:
            return -1  # Not enough battery to reach the POI safely

        # Return the estimated time in minutes, rounded to two decimal places
        return round(poi_time_to_fly_sec / 60, 2)

    @staticmethod
    def calculate_ground_speed(
            airspeed_m_s: float,
            heading_deg: float,
            wind_speed_m_s: float,
            wind_direction_deg: float
    ) -> float:
        """
        Calculates the ground speed of the vehicle towards the POI, considering wind.

        :param airspeed_m_s: Vehicle's airspeed in m/s.
        :param heading_deg: Heading from the vehicle to the POI in degrees.
        :param wind_speed_m_s: Wind speed in m/s.
        :param wind_direction_deg: Wind direction in degrees FROM which the wind is coming.
        :return: Ground speed towards the POI in m/s.
        """
        # Convert degrees to radians
        heading_rad = math.radians(heading_deg)
        # Wind direction is the direction FROM which the wind is coming
        # So the wind is going TO wind_direction_deg + 180 degrees
        wind_direction_to_deg = (wind_direction_deg + 180) % 360
        wind_direction_rad = math.radians(wind_direction_to_deg)

        # Wind velocity components
        wind_velocity_x = wind_speed_m_s * math.sin(wind_direction_rad)
        wind_velocity_y = wind_speed_m_s * math.cos(wind_direction_rad)

        # Vehicle's airspeed components (assuming heading is direction of motion)
        airspeed_x = airspeed_m_s * math.sin(heading_rad)
        airspeed_y = airspeed_m_s * math.cos(heading_rad)

        # Ground speed components
        ground_speed_x = airspeed_x + wind_velocity_x
        ground_speed_y = airspeed_y + wind_velocity_y

        # Calculate the ground speed in the direction towards the POI
        # Project ground speed vector onto the heading direction
        ground_speed_towards_poi = (
                ground_speed_x * math.sin(heading_rad) + ground_speed_y * math.cos(heading_rad)
        )

        return max(ground_speed_towards_poi, 0.0)  # Ground speed cannot be negative
