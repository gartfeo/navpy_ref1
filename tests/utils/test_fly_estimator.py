import unittest
from unittest.mock import Mock

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.utils.fly_estimator import FlyEstimator


class FlyEstimatorTest(unittest.TestCase):
    def setUp(self):
        # Mocking IDrone interface
        self.vehicle = Mock(spec=IVehicle)
        self.vehicle.location.return_value = Location(0.0, 0.0, 0.0)
        self.vehicle.battery_level = 80  # 80% battery

        # Target location 600 meters north from the vehicle's current location
        self.target_location = Location(0.0054, 0.0, 0.0)  # Approx 600 meters north

    def test_time_to_fly_no_wind(self):
        # Arrange
        self.vehicle.ground_speed = 10.0  # Airspeed in m/s
        self.vehicle.wind.speed = 0.0  # No wind
        self.vehicle.wind.direction = 0.0

        # Act
        estimated_time = FlyEstimator.time_to_fly(self.vehicle, self.target_location)

        # Assert
        self.assertIsNotNone(estimated_time)
        self.assertGreater(estimated_time, 0)
        # Expected time: distance / speed (600m / 10 m/s) = 60s => 1 minute
        self.assertAlmostEqual(estimated_time, 1.0, places=2)

    def test_time_to_fly_with_headwind(self):
        # Arrange
        self.vehicle.ground_speed = 10.0  # Airspeed in m/s
        self.vehicle.wind.speed = 5.0  # Wind speed in m/s
        self.vehicle.wind.direction = 180.0  # Wind coming from the North (0 degrees)

        # Act
        estimated_time = FlyEstimator.time_to_fly(self.vehicle, self.target_location)

        # Assert
        self.assertIsNotNone(estimated_time)
        # Wind is coming from 0 degrees (North), so going to 180 degrees (South)
        # Heading is towards 0 degrees (North)
        # The wind is opposing the vehicle's motion
        # Effective ground speed = Airspeed - Wind speed = 10 - 5 = 5 m/s
        self.assertAlmostEqual(estimated_time, 2, places=2)

    def test_time_to_fly_with_tailwind(self):
        # Arrange
        self.vehicle.ground_speed = 10.0  # Airspeed in m/s
        self.vehicle.wind.speed = 5.0  # Wind speed in m/s
        self.vehicle.wind.direction = 0.0  # Wind coming from the South (180 degrees)

        # Act
        estimated_time = FlyEstimator.time_to_fly(self.vehicle, self.target_location)

        # Assert
        self.assertIsNotNone(estimated_time)
        # Wind is coming from 180 degrees (South), so going to 0 degrees (North)
        # Heading is towards 0 degrees (North)
        # The wind is aiding the vehicle's motion
        # Effective ground speed = Airspeed + Wind speed = 10 + 5 = 15 m/s
        # Estimated time: 600m / 15 m/s = 40s => 0.67 minutes
        self.assertAlmostEqual(estimated_time, 0.67, places=2)

    def test_time_to_fly_with_crosswind(self):
        # Arrange
        self.vehicle.ground_speed = 10.0  # Airspeed in m/s
        self.vehicle.wind.speed = 5.0  # Wind speed in m/s
        self.vehicle.wind.direction = 90.0  # Wind coming from the East (90 degrees)

        # Act
        estimated_time = FlyEstimator.time_to_fly(self.vehicle, self.target_location)

        # Assert
        self.assertIsNotNone(estimated_time)
        # Wind is coming from 90 degrees (East), so going to 270 degrees (West)
        # Heading is towards 0 degrees (North)
        # Crosswind component doesn't affect ground speed towards target
        # Effective ground speed towards target remains approximately 10 m/s
        self.assertAlmostEqual(estimated_time, 1.0, places=2)
