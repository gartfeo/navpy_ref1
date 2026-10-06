import unittest

from navpy.modules.vision.zoom_calibration import (
    ZoomCalibrationEntry,
    ZoomCalibrationTable,
)


class TestFromProfile(unittest.TestCase):
    def test_none_when_missing(self):
        self.assertIsNone(ZoomCalibrationTable.from_profile(None))
        self.assertIsNone(ZoomCalibrationTable.from_profile({}))

    def test_none_when_fewer_than_two_parseable_entries(self):
        self.assertIsNone(ZoomCalibrationTable.from_profile({"1.0": 1.1}))
        self.assertIsNone(
            ZoomCalibrationTable.from_profile({"1.0": 1.1, "bad": "x"})
        )

    def test_skips_non_numeric_entries(self):
        t = ZoomCalibrationTable.from_profile(
            {"1.0": 1.1, "2.0": 1.8, "weird": "meep"}
        )
        self.assertIsNotNone(t)
        self.assertEqual(len(t.entries), 2)

    def test_rejects_non_positive_values(self):
        t = ZoomCalibrationTable.from_profile(
            {"1.0": 1.1, "2.0": 1.8, "-3.0": 2.5, "4.0": 0, "5.0": -1.0}
        )
        self.assertIsNotNone(t)
        self.assertEqual({e.commanded for e in t.entries}, {"1.0", "2.0"})

    def test_sorts_by_actual_ascending(self):
        t = ZoomCalibrationTable.from_profile(
            {"10.0": 10.0, "1.0": 1.1, "5.0": 4.8, "3.0": 2.8}
        )
        actuals = [e.actual for e in t.entries]
        self.assertEqual(actuals, sorted(actuals))

    def test_duplicate_actuals_order_by_commanded_ascending(self):
        # cmd 1.0, 1.1, 1.2, 1.3 all → actual 1.1 (floor clamp).
        # Entries must be ordered by (actual, float(commanded)) so the
        # first match for actual=1.1 is "1.0", not "1.3".
        t = ZoomCalibrationTable.from_profile(
            {"1.3": 1.1, "1.2": 1.1, "1.0": 1.1, "1.1": 1.1, "1.5": 1.3}
        )
        floor_commands = [e.commanded for e in t.entries if e.actual == 1.1]
        self.assertEqual(floor_commands, ["1.0", "1.1", "1.2", "1.3"])


class TestReadbackToCommand(unittest.TestCase):
    """readback_to_command: reverse-lookup commanded-float from readback."""

    def test_none_for_empty_table(self):
        # from_profile returns None for < 2 entries, so we construct
        # directly to exercise the empty path.
        t = ZoomCalibrationTable(entries=())
        self.assertIsNone(t.readback_to_command(1.5))

    def test_clamps_below_range_to_lowest_canonical_cmd(self):
        # Canonical lowest actual is 1.1 (cmd 1.0 wins the duplicate
        # group per from_profile sort). Input 0.5 < 1.1 → clamp.
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "1.3": 1.1, "1.4": 1.2, "2.0": 1.8,
        })
        self.assertAlmostEqual(t.readback_to_command(0.5), 1.0)

    def test_clamps_above_range_to_highest_canonical_cmd(self):
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "2.0": 1.8, "10.0": 10.0,
        })
        self.assertAlmostEqual(t.readback_to_command(15.0), 10.0)

    def test_exact_match_returns_canonical_cmd(self):
        # readback 1.1 has duplicate cmds 1.0, 1.1, 1.2, 1.3.
        # Canonical is the lowest (1.0).
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "1.1": 1.1, "1.2": 1.1, "1.3": 1.1, "1.4": 1.2,
        })
        self.assertAlmostEqual(t.readback_to_command(1.1), 1.0)

    def test_linear_interpolation_between_canonical_points(self):
        # Canonical: (1.1 → 1.0), (1.8 → 2.0). Interpolate at 1.45:
        # t = (1.45 - 1.1) / (1.8 - 1.1) = 0.5 → cmd = 1.0 + 0.5 * 1.0 = 1.5
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "2.0": 1.8,
        })
        self.assertAlmostEqual(t.readback_to_command(1.45), 1.5)

    def test_interpolation_ignores_duplicate_actual_brackets(self):
        # cmd 1.0/1.1/1.2/1.3 all → actual 1.1 (canonical cmd 1.0).
        # cmd 1.4 → actual 1.2 (next canonical). Interpolation between
        # readback 1.1 (cmd 1.0) and 1.2 (cmd 1.4), NOT between any
        # of the 1.1-actual duplicates which would collapse the bracket.
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "1.1": 1.1, "1.2": 1.1, "1.3": 1.1,
            "1.4": 1.2, "1.5": 1.3,
        })
        # At readback 1.15 (halfway between 1.1 and 1.2):
        # t = (1.15 - 1.1) / (1.2 - 1.1) = 0.5
        # cmd = 1.0 + 0.5 * (1.4 - 1.0) = 1.2
        self.assertAlmostEqual(t.readback_to_command(1.15), 1.2, places=5)

    def test_siyi_calibration_sample(self):
        # A realistic slice of the SIYI table. Verify interpolation at
        # one off-calibrated readback.
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "1.4": 1.2, "1.5": 1.3, "1.6": 1.4, "1.7": 1.5,
            "1.8": 1.6, "1.9": 1.7, "2.0": 1.8, "3.0": 2.8,
        })
        # readback exactly 1.5 → canonical cmd 1.7
        self.assertAlmostEqual(t.readback_to_command(1.5), 1.7)
        # readback 2.3 → between (1.8 → 2.0) and (2.8 → 3.0):
        # t = (2.3 - 1.8) / (2.8 - 1.8) = 0.5 → cmd = 2.0 + 0.5 = 2.5
        self.assertAlmostEqual(t.readback_to_command(2.3), 2.5)

    def test_returns_float_not_str(self):
        t = ZoomCalibrationTable.from_profile({"1.0": 1.1, "2.0": 1.8})
        result = t.readback_to_command(1.1)
        self.assertIsInstance(result, float)


class TestPropertiesAndEntry(unittest.TestCase):
    def test_actual_min_and_max(self):
        t = ZoomCalibrationTable.from_profile({
            "1.0": 1.1, "2.0": 1.8, "10.0": 10.0,
        })
        self.assertAlmostEqual(t.actual_min, 1.1)
        self.assertAlmostEqual(t.actual_max, 10.0)

    def test_entry_is_frozen_dataclass(self):
        e = ZoomCalibrationEntry(commanded="1.0", actual=1.1)
        with self.assertRaises(Exception):
            e.actual = 2.0  # frozen


if __name__ == "__main__":
    unittest.main()
