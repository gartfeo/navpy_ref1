"""Tests for characteristic_pixels — the bbox-diagonal recognition measure."""
import math
import unittest

from navpy.modules.vision.target_size import characteristic_pixels


class TestCharacteristicPixels(unittest.TestCase):
    def test_diagonal_of_3_4_is_5(self):
        self.assertAlmostEqual(characteristic_pixels(3.0, 4.0), 5.0)

    def test_square_is_side_times_sqrt2(self):
        self.assertAlmostEqual(characteristic_pixels(10.0, 10.0), 10.0 * math.sqrt(2))

    def test_tall_narrow_reads_about_height(self):
        # A standing person (w << h): diagonal ~ height (dominant dim).
        size = characteristic_pixels(20.0, 180.0)
        self.assertGreater(size, 180.0)
        self.assertLess(size, 182.0)  # within ~1% of height

    def test_wide_target_exceeds_height(self):
        # A broadside medium (w > h): diagonal clearly exceeds height alone.
        self.assertAlmostEqual(
            characteristic_pixels(140.0, 60.0), math.sqrt(140.0**2 + 60.0**2))
        self.assertGreater(characteristic_pixels(140.0, 60.0), 60.0)

    def test_symmetric_in_w_and_h(self):
        self.assertAlmostEqual(
            characteristic_pixels(30.0, 70.0), characteristic_pixels(70.0, 30.0))

    def test_rejects_non_finite(self):
        for w, h in [(math.nan, 10.0), (10.0, math.inf), (math.inf, math.nan)]:
            with self.assertRaises(ValueError):
                characteristic_pixels(w, h)

    def test_rejects_non_positive(self):
        for w, h in [(0.0, 10.0), (10.0, 0.0), (-5.0, 10.0), (10.0, -5.0)]:
            with self.assertRaises(ValueError):
                characteristic_pixels(w, h)


if __name__ == "__main__":
    unittest.main()
