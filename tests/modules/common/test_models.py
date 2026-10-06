"""Tests for common model classes."""
import math
import unittest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.common.models.wind import Wind


class TestLocation(unittest.TestCase):
    """Tests for Location class."""

    def test_init_required_fields(self):
        """Location initializes with lat, lng, alt."""
        loc = Location(lat=40.7128, lng=-74.0060, alt=100.0)

        self.assertEqual(loc.lat, 40.7128)
        self.assertEqual(loc.lng, -74.0060)
        self.assertEqual(loc.alt, 100.0)

    def test_init_default_values(self):
        """Location has correct default values."""
        loc = Location(lat=40.0, lng=-74.0, alt=50.0)

        self.assertEqual(loc.heading, 0.0)
        self.assertFalse(loc.is_absolute)

    def test_init_custom_optional(self):
        """Location accepts custom optional fields."""
        loc = Location(
            lat=40.0, lng=-74.0, alt=50.0,
            heading=90.0, is_absolute=True
        )

        self.assertEqual(loc.heading, 90.0)
        self.assertTrue(loc.is_absolute)

    def test_str_representation(self):
        """__str__ returns formatted string."""
        loc = Location(lat=40.712800, lng=-74.006000, alt=100.5)

        result = str(loc)

        self.assertIn("40.712800", result)
        self.assertIn("-74.006000", result)
        self.assertIn("100.5", result)

    def test_distance_to_same_location(self):
        """distance_to returns 0 for same location."""
        loc1 = Location(lat=40.0, lng=-74.0, alt=100.0)
        loc2 = Location(lat=40.0, lng=-74.0, alt=200.0)  # Alt doesn't matter

        distance = loc1.distance_to(loc2)

        self.assertAlmostEqual(distance, 0.0, places=5)

    def test_distance_to_known_distance(self):
        """distance_to calculates correct haversine distance."""
        # NYC to LA is approximately 3944 km
        nyc = Location(lat=40.7128, lng=-74.0060, alt=0)
        la = Location(lat=34.0522, lng=-118.2437, alt=0)

        distance = nyc.distance_to(la)

        # Should be approximately 3944 km (allow 50km tolerance)
        self.assertAlmostEqual(distance / 1000, 3944, delta=50)

    def test_distance_to_short_distance(self):
        """distance_to works for short distances."""
        loc1 = Location(lat=40.0, lng=-74.0, alt=0)
        # Move ~111 meters north (0.001 degrees latitude)
        loc2 = Location(lat=40.001, lng=-74.0, alt=0)

        distance = loc1.distance_to(loc2)

        # 0.001 degrees latitude ~ 111 meters
        self.assertAlmostEqual(distance, 111, delta=5)

    def test_distance_to_symmetric(self):
        """distance_to is symmetric (a->b == b->a)."""
        loc1 = Location(lat=40.0, lng=-74.0, alt=0)
        loc2 = Location(lat=41.0, lng=-75.0, alt=0)

        d1 = loc1.distance_to(loc2)
        d2 = loc2.distance_to(loc1)

        self.assertAlmostEqual(d1, d2, places=5)


class TestAttitude(unittest.TestCase):
    """Tests for Attitude class."""

    def test_init(self):
        """Attitude initializes with pitch, yaw, roll."""
        att = Attitude(pitch=10.0, yaw=45.0, roll=-5.0)

        self.assertEqual(att.pitch, 10.0)
        self.assertEqual(att.yaw, 45.0)
        self.assertEqual(att.roll, -5.0)

    def test_str_representation(self):
        """__str__ returns formatted string."""
        att = Attitude(pitch=10.5, yaw=45.2, roll=-5.3)

        result = str(att)

        self.assertIn("p=10.5", result)
        self.assertIn("y=45.2", result)
        self.assertIn("r=-5.3", result)

    def test_to_radians(self):
        """to_radians converts degrees to radians."""
        att = Attitude(pitch=90.0, yaw=180.0, roll=45.0)

        rad = att.to_radians()

        self.assertAlmostEqual(rad.pitch, math.pi / 2, places=5)
        self.assertAlmostEqual(rad.yaw, math.pi, places=5)
        self.assertAlmostEqual(rad.roll, math.pi / 4, places=5)

    def test_to_radians_zero(self):
        """to_radians handles zero angles."""
        att = Attitude(pitch=0.0, yaw=0.0, roll=0.0)

        rad = att.to_radians()

        self.assertEqual(rad.pitch, 0.0)
        self.assertEqual(rad.yaw, 0.0)
        self.assertEqual(rad.roll, 0.0)

    def test_to_radians_negative(self):
        """to_radians handles negative angles."""
        att = Attitude(pitch=-90.0, yaw=-180.0, roll=-45.0)

        rad = att.to_radians()

        self.assertAlmostEqual(rad.pitch, -math.pi / 2, places=5)
        self.assertAlmostEqual(rad.yaw, -math.pi, places=5)
        self.assertAlmostEqual(rad.roll, -math.pi / 4, places=5)

    def test_to_radians_returns_new_instance(self):
        """to_radians returns new Attitude, doesn't modify original."""
        att = Attitude(pitch=90.0, yaw=180.0, roll=45.0)

        rad = att.to_radians()

        # Original unchanged
        self.assertEqual(att.pitch, 90.0)
        # Result is different instance
        self.assertIsNot(rad, att)

    def test_from_radians(self):
        """from_radians creates Attitude from radian values."""
        att = Attitude.from_radians(
            pitch=math.pi / 2,
            yaw=math.pi,
            roll=math.pi / 4
        )

        self.assertAlmostEqual(att.pitch, 90.0, places=3)
        self.assertAlmostEqual(att.yaw, 180.0, places=3)
        self.assertAlmostEqual(att.roll, 45.0, places=3)

    def test_to_radians_from_radians_roundtrip(self):
        """to_radians and from_radians are inverses."""
        original = Attitude(pitch=30.0, yaw=60.0, roll=15.0)

        rad = original.to_radians()
        back = Attitude.from_radians(rad.pitch, rad.yaw, rad.roll)

        self.assertAlmostEqual(back.pitch, original.pitch, places=5)
        self.assertAlmostEqual(back.yaw, original.yaw, places=5)
        self.assertAlmostEqual(back.roll, original.roll, places=5)


class TestWind(unittest.TestCase):
    """Tests for Wind class."""

    def test_init(self):
        """Wind initializes with direction and speed."""
        wind = Wind(direction=180.0, speed=10.0)

        self.assertEqual(wind.direction, 180.0)
        self.assertEqual(wind.speed, 10.0)
        self.assertEqual(wind.speed_z, 0.0)

    def test_init_with_speed_z(self):
        """Wind accepts vertical speed."""
        wind = Wind(direction=90.0, speed=5.0, speed_z=2.0)

        self.assertEqual(wind.speed_z, 2.0)

    def test_direction_normalized_negative(self):
        """Negative direction is normalized to 0-360."""
        wind = Wind(direction=-90.0, speed=10.0)

        self.assertEqual(wind.direction, 270.0)

    def test_direction_normalized_large_negative(self):
        """Large negative direction is normalized."""
        wind = Wind(direction=-180.0, speed=10.0)

        self.assertEqual(wind.direction, 180.0)

    def test_heading_property(self):
        """heading returns direction wind is going TO."""
        # Wind from north (0) goes to south (180)
        wind = Wind(direction=0.0, speed=10.0)
        self.assertEqual(wind.heading, 180.0)

        # Wind from south (180) goes to north (0)
        wind = Wind(direction=180.0, speed=10.0)
        self.assertEqual(wind.heading, 0.0)

        # Wind from east (90) goes to west (270)
        wind = Wind(direction=90.0, speed=10.0)
        self.assertEqual(wind.heading, 270.0)

    def test_heading_wraps_correctly(self):
        """heading wraps around 360 correctly."""
        # Wind from 270 (west) goes to 90 (east)
        wind = Wind(direction=270.0, speed=10.0)
        self.assertEqual(wind.heading, 90.0)

    def test_str_representation(self):
        """__str__ returns formatted string."""
        wind = Wind(direction=45.0, speed=12.5)

        result = str(wind)

        self.assertIn("dir=45", result)
        self.assertIn("spd=12.5", result)

    def test_repr(self):
        """__repr__ returns constructor-like string."""
        wind = Wind(direction=90.0, speed=5.0, speed_z=1.5)

        result = repr(wind)

        self.assertIn("Wind(", result)
        self.assertIn("direction=90", result)
        self.assertIn("speed=5", result)
        self.assertIn("speed_z=1.5", result)


if __name__ == '__main__':
    unittest.main()
