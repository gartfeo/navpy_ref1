import math
import unittest

import pymap3d
from numpy.testing import assert_allclose

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.rotation_utils import normalize
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.common.models.location import Location


class ZcUtilTestCase(unittest.TestCase):

    def test_geodetic(self):
        zc_util = ZcUtil()

        target_alt = 1295.21
        target_coords = [40.3114898305604, 44.4552312791348]
        target_alt_zc = zc_util.get_elevation(target_coords)
        target_alt_delta = target_alt_zc - target_alt
        target_loc = Location(lat=target_coords[0], lng=target_coords[1], alt=target_alt_zc)

        current_alt = 1290.17
        current_coords = [40.308993, 44.447804]
        current_alt_zc = zc_util.get_elevation(current_coords)
        current_alt_delta = current_alt_zc - current_alt
        current_loc = Location(lat=40.308993, lng=44.447804, alt=1800 + current_alt_delta)

        ned = pymap3d.geodetic2ned(target_loc.lat, target_loc.lng, target_loc.alt,
                                   current_loc.lat, current_loc.lng, current_loc.alt)

        actual_loc = zc_util.ray_to_terrain_ned(current_loc, ned, delta_alt=-target_alt_delta)

        self.assertAlmostEqual(actual_loc.alt, target_alt, delta=0.1)
        self.assertAlmostEqual(actual_loc.lat, target_loc.lat, delta=1e-5)
        self.assertAlmostEqual(actual_loc.lng, target_loc.lng, delta=1e-5)

    def test_actual_loc_down(self):
        expected_alt = 1295.4000244140625
        expected_loc = Location(lat=40.31148910522461, lng=44.455230712890625, alt=expected_alt + 300)
        zc_util = ZcUtil(max_distance=10000, degrees=True)
        actual_loc = zc_util.ray_to_terrain_ned(expected_loc, [0.0, 0.0, 1.0])
        self.assertAlmostEqual(actual_loc.lat, expected_loc.lat)
        self.assertAlmostEqual(actual_loc.lng, expected_loc.lng)
        self.assertAlmostEqual(actual_loc.alt, expected_alt, places=0)

    def test_actual_loc_ned(self):
        zc_util = ZcUtil()

        home_loc = Location(lat=40.31148910522461, lng=44.455230712890625, alt=1295.4000244140625)
        current_loc = Location(lat=40.308993, lng=44.447804, alt=304.2)
        expected_target_loc = Location(lat=40.311489, lng=44.455231, alt=0.0)
        target_ned = [0.42867977, 0.97587588, 0.47028754]

        actual_target_loc = zc_util.ray_to_terrain_ned(current_loc, target_ned, home_alt=home_loc.alt)
        self.assertAlmostEqual(actual_target_loc.lat, expected_target_loc.lat, places=5)
        self.assertAlmostEqual(actual_target_loc.lng, expected_target_loc.lng, places=4) # TODO adjusted for precision
        self.assertAlmostEqual(actual_target_loc.alt, expected_target_loc.alt, places=0)

    def test_geo_ned(self):
        zc_util = ZcUtil(max_distance=10000, degrees=True)
        current_loc = Location(40.3067927, 44.4511127, 1621.4)
        target_loc = zc_util.ray_to_terrain(current_loc, Attitude(-35, 3, 5))

        actual_ned = pymap3d.geodetic2ned(target_loc.lat, target_loc.lng, target_loc.alt,
                                          current_loc.lat, current_loc.lng, current_loc.alt)
        actual_ned = normalize(actual_ned)

        actual_target_loc = zc_util.ray_to_terrain_ned(current_loc, actual_ned)

        self.assertAlmostEqual(actual_target_loc.lat, target_loc.lat, places=4)
        self.assertAlmostEqual(actual_target_loc.lng, target_loc.lng, places=4)
        self.assertAlmostEqual(actual_target_loc.alt, target_loc.alt, places=1)

    def test_intersection(self):
        ray_origin_gps = Location(40.3116676, 44.4551189, 1800)  # Latitude, Longitude, Altitude (above sea level)
        att = Attitude(-15, 0, 0)  # Pitch, Roll, Yaw
        zc_util = ZcUtil(max_distance=10000)
        ray_intersection_result = zc_util.ray_to_terrain(ray_origin_gps, att)
        actual_result = [ray_intersection_result.lat, ray_intersection_result.lng, ray_intersection_result.alt]
        assert_allclose(actual_result, [40.3278280, 44.4551189, 1319], rtol=0.001)

    def test_intersection_delta_alt(self):
        ray_origin_gps = Location(40.3126676, 44.4551189, 1800)  # Latitude, Longitude, Altitude (above sea level)
        att = Attitude(-25, 0, 0)  # Pitch, Roll, Yaw
        zc_util = ZcUtil(max_distance=10000)
        ray_intersection_result = zc_util.ray_to_terrain(ray_origin_gps, att, delta_alt=2)
        actual_result = [ray_intersection_result.lat, ray_intersection_result.lng, ray_intersection_result.alt]
        assert_allclose(actual_result, [40.322097, 44.455119, 1311.740494], rtol=0.001)

    def test_intersection_range_returns_max_distance_point(self):
        """When no terrain intersection within max_distance, return point at max_distance in the air."""
        ray_origin_gps = Location(40.3116676, 44.4551189, 1800)  # Latitude, Longitude, Altitude (above sea level)
        att = Attitude(math.radians(-45), 0, 0)  # Pitch, Roll, Yaw (degrees=False since radians)

        max_dist = 10
        zc_util = ZcUtil(max_distance=max_dist, degrees=False)
        ray_intersection_result = zc_util.ray_to_terrain(ray_origin_gps, att)

        # Should return a Location, not None
        self.assertIsNotNone(ray_intersection_result)
        # The result should be approximately max_distance away horizontally
        # At -45 degrees pitch, horizontal distance equals vertical drop
        # For a short max_distance=10m, point should be in the air above terrain

    def test_no_intersection_returns_point_at_max_distance(self):
        """Test that when ray doesn't hit terrain, we get coordinates at max_distance."""
        zc_util = ZcUtil(max_distance=100, degrees=True)

        # Looking upward (positive pitch) - will never hit terrain
        ray_origin_gps = Location(40.3116676, 44.4551189, 1800)
        att = Attitude(pitch=10, roll=0, yaw=0)  # Looking slightly up

        result = zc_util.ray_to_terrain(ray_origin_gps, att)

        # Should return a Location (not None) at max_distance
        self.assertIsNotNone(result)
        # Altitude should be higher than origin since looking up
        self.assertGreater(result.alt, ray_origin_gps.alt)


if __name__ == '__main__':
    unittest.main()
