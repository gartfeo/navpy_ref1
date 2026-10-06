import math
import unittest

from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits, r_nav_min


class TestRNavMin(unittest.TestCase):
    """Minimum valid orbit standoff: max of turn-radius and descent terms,
    floored, with degenerate terms dropping out."""

    def test_descent_term_dominates_typical(self):
        # h=150, V=25, roll 45, min_pitch -40, eta 0.65 -> descent ~307m
        # dominates the turn term (~127m).
        limits = OrbitNavigationLimits(25.0, 45.0, -40.0)
        expected = 150.0 / math.tan(math.radians(0.65 * 40.0))
        self.assertAlmostEqual(r_nav_min(limits, 150.0, 80.0), expected, delta=0.5)
        self.assertGreater(r_nav_min(limits, 150.0, 80.0), 300.0)

    def test_turn_term_dominates_low_altitude_fast(self):
        # Low orbit altitude + fast airspeed -> turn radius binds.
        limits = OrbitNavigationLimits(40.0, 45.0, -40.0)
        turn = 2.0 * 40.0 ** 2 / (9.81 * math.tan(math.radians(45.0)))
        descent = 10.0 / math.tan(math.radians(0.65 * 40.0))
        self.assertGreater(turn, descent)
        self.assertAlmostEqual(r_nav_min(limits, 10.0, 80.0), turn, delta=0.5)

    def test_floor_when_both_terms_small(self):
        limits = OrbitNavigationLimits(1.0, 45.0, -40.0)
        self.assertEqual(r_nav_min(limits, 1.0, 80.0), 80.0)

    def test_degenerate_roll_drops_turn_term(self):
        # roll 90 -> tan(90) non-finite; roll 0 -> tan 0; both drop the
        # turn term, descent governs.
        descent = 150.0 / math.tan(math.radians(0.65 * 40.0))
        for roll in (90.0, 0.0):
            limits = OrbitNavigationLimits(25.0, roll, -40.0)
            self.assertAlmostEqual(
                r_nav_min(limits, 150.0, 0.0), descent, delta=0.5)

    def test_degenerate_min_pitch_drops_descent_term(self):
        # min_pitch 0 -> descent term drops; turn governs (above floor 0).
        limits = OrbitNavigationLimits(25.0, 45.0, 0.0)
        turn = 2.0 * 25.0 ** 2 / (9.81 * math.tan(math.radians(45.0)))
        self.assertAlmostEqual(r_nav_min(limits, 150.0, 0.0), turn, delta=0.5)

    def test_non_positive_altitude_drops_descent_term(self):
        limits = OrbitNavigationLimits(25.0, 45.0, -40.0)
        turn = 2.0 * 25.0 ** 2 / (9.81 * math.tan(math.radians(45.0)))
        self.assertAlmostEqual(r_nav_min(limits, 0.0, 0.0), turn, delta=0.5)

    def test_eta_widens_radius(self):
        # Lower eta (less pitch authority used) -> shallower dive -> farther
        # orbit.
        safe = OrbitNavigationLimits(25.0, 45.0, -40.0, eta=0.65)
        aggressive = OrbitNavigationLimits(25.0, 45.0, -40.0, eta=0.85)
        self.assertGreater(
            r_nav_min(safe, 150.0, 80.0), r_nav_min(aggressive, 150.0, 80.0))


if __name__ == "__main__":
    unittest.main()
