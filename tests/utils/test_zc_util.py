import unittest
from unittest.mock import MagicMock
import numpy as np

from navpy.modules.common.models.location import Location
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.zc_util import ZcUtil, DemData


class TestZcUtil(unittest.TestCase):
    def setUp(self):
        self.zc_util = ZcUtil()

    def test_invalid_coords(self):
        """
        (200, 200) => outside lat/lon bounds => immediate None
        """
        elev = self.zc_util.get_elevation((200.0, 200.0))
        self.assertIsNone(elev, "Should be None for invalid coords")

        loc = Location(200.0, 200.0, 100)
        att = Attitude(-90, 0, 0)
        result = self.zc_util.ray_to_terrain(loc, att)
        self.assertIsNone(result, "No intersection if invalid coords")

    def test_missing_tile_returns_none(self):
        """
        File missing, user says no => returns None
        """
        repository = MagicMock()
        repository.get_elevation.return_value = None
        zc_util = ZcUtil(tile_repository=repository)

        elev = zc_util.get_elevation((37.0, -123.0))
        self.assertIsNone(elev)

    def test_straight_down_intersection(self):
        """
        Insert a fully-prepared DemData mock, then verify a straight-down intersection => alt=0.0
        """
        # Create a fully-prepared mock DemData
        mock_dem = DemData.__new__(DemData)
        mock_dem.transform = MagicMock()
        mock_dem.dem_data = np.zeros((100, 100), dtype=np.float32)
        mock_dem.dem_interpolator = MagicMock(return_value=0.0)

        def mock_gps(lat, lon):
            return float(lon), float(lat)

        mock_dem.gps_to_dem_coords = MagicMock(side_effect=mock_gps)

        def mock_height(coord):
            return 0.0  # always zero terrain

        mock_dem.get_height = MagicMock(side_effect=mock_height)

        repository = MagicMock()
        repository.get_tile.return_value = mock_dem
        zc_util = ZcUtil(tile_repository=repository)
        location = Location(37.0, -123.0, 100.0)
        att = Attitude(pitch=-90, roll=0, yaw=0)

        result = zc_util.ray_to_terrain(location, att)

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.alt, 0.0, places=2, msg="Altitude should be 0.0")

    def test_non_vertical_coarse_to_fine_intersection(self):
        """
        Test a downward pitch, not purely vertical, that should intersect flat terrain at 1000 altitude.
        Vehicle altitude=1500 => intersection ~ some horizontal distance.
        """
        # Build a mock DEM that is always altitude=1000
        mock_dem = DemData.__new__(DemData)
        mock_dem.dem_data = np.zeros((100, 100), dtype=np.float32)
        mock_dem.transform = MagicMock()
        mock_dem.dem_interpolator = MagicMock(return_value=1000.0)

        def mock_gps_to_dem(lat, lon):
            # Dummy transform => just pass through
            return float(lon), float(lat)

        mock_dem.gps_to_dem_coords = MagicMock(side_effect=mock_gps_to_dem)

        def mock_get_height(coords):
            # Always 1000.0
            return 1000.0

        mock_dem.get_height = MagicMock(side_effect=mock_get_height)

        repository = MagicMock()
        repository.get_tile.return_value = mock_dem
        zc_util = ZcUtil(tile_repository=repository)

        # Vehicle at altitude 1500, so we have 500 m difference
        loc = Location(lat=40.0, lng=44.0, alt=1500.0)
        # Suppose pitch=-30 => mild downward => we create a DroneAttitude
        # For simplicity, let's define pitch=-30, roll=0, yaw=0 in your Euler scheme (ZYX).
        att = Attitude(pitch=-30, roll=0, yaw=0)

        result = zc_util.ray_to_terrain(loc, att)
        self.assertIsNotNone(result, "Should find intersection for downward angle")
        # The mock terrain is always 1000. If everything is correct, the intersection alt=1000.
        self.assertAlmostEqual(result.alt, 1000.0, places=1)

        # Print to see how far horizontally we traveled (not mandatory in real tests)
        print("Intersection Location =>", result)


if __name__ == "__main__":
    unittest.main()
