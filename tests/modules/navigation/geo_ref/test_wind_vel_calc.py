import unittest
from unittest.mock import Mock, MagicMock

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.wind import Wind
from navpy.modules.navigation.mission_planner import MissionPlanner


def create_vehicle_mock(wind, air_speed, attitude):
    """Create a mock vehicle with the required properties for wind velocity calculation tests.
    
    Uses MagicMock to avoid having to implement all abstract methods of IVehicle.
    This is appropriate for unit tests that only need a subset of vehicle functionality.
    """
    vehicle = MagicMock()
    vehicle.wind = wind
    vehicle.air_speed = air_speed
    vehicle.attitude = attitude
    vehicle.target_system = 1
    vehicle.source_system = 1
    return vehicle


class WindVelCalcTestCase(unittest.TestCase):
    def __init__(self, tests=()):
        super().__init__(tests)
        # create mock of argparser
        mock_args = Mock()
        mock_args.min_air_speed = 0
        self._mp = MissionPlanner(None, mock_args)

    def test_calc_wind_velocity_should_be_more_than_min(self):
        mock_wind_instance = Wind(270, 5, 0)
        mock_attitude_instance = Attitude(-30, 40, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        args = Mock()
        args.min_air_speed = 6
        mp = MissionPlanner(None, args)

        wind_y, wind_p = mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_p, 0)
        self.assertEqual(wind_y, 0)

    def test_calc_wind_0_velocity_yaw_0(self):
        mock_wind_instance = Wind(0, 5, 0)
        mock_attitude_instance = Attitude(0, 0, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_0_velocity_yaw_90(self):
        mock_wind_instance = Wind(0, 5, 0)
        mock_attitude_instance = Attitude(0, 90, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, -0.25)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_0_velocity_yaw_270(self):
        mock_wind_instance = Wind(0, 5, 0)
        mock_attitude_instance = Attitude(0, 270, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0.25)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_90_velocity_yaw_0(self):
        mock_wind_instance = Wind(90, 5, 0)
        mock_attitude_instance = Attitude(0, 0, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0.25)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_270_velocity_yaw_0(self):
        mock_wind_instance = Wind(270, 5, 0)
        mock_attitude_instance = Attitude(0, 0, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, -0.25)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_90_velocity_yaw_90(self):
        mock_wind_instance = Wind(90, 5, 0)
        mock_attitude_instance = Attitude(0, 90, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_90_velocity_yaw_270(self):
        mock_wind_instance = Wind(90, 0.25, 0)
        mock_attitude_instance = Attitude(0, 270, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0)
        self.assertEqual(wind_p, 0)

    def test_calc_wind_0_velocity_pitch_90(self):
        mock_wind_instance = Wind(0, 5, 0)
        mock_attitude_instance = Attitude(90, 0, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0)
        self.assertEqual(wind_p, 0.25)

    def test_calc_wind_0_velocity_pitch_270(self):
        mock_wind_instance = Wind(0, 5, 0)
        mock_attitude_instance = Attitude(270, 0, 0)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertEqual(wind_y, 0)
        self.assertEqual(wind_p, -0.25)

    def test_calc_wind_0_velocity_pitch_90_roll_90(self):
        mock_wind_instance = Wind(0, 5, 0)
        mock_attitude_instance = Attitude(90, 0, 90)

        mock_vehicle = create_vehicle_mock(mock_wind_instance, 20, mock_attitude_instance)

        wind_y, wind_p = self._mp.calculate_ff_coefficients(mock_vehicle)

        # Assert the expected outcomes
        self.assertAlmostEqual(wind_y, 0.25)
        self.assertAlmostEqual(wind_p, 0)


if __name__ == '__main__':
    unittest.main()
