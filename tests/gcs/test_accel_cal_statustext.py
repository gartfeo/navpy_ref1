"""Tests for accel-cal STATUSTEXT classification + VehicleEntry relay buffering."""
import types
import unittest
from unittest.mock import MagicMock

from gcs.backend.vehicle_manager import (
    VehicleEntry,
    _classify_accel_cal_statustext,
    ACCEL_CAL_POS_LEVEL,
    ACCEL_CAL_POS_LEFT,
    ACCEL_CAL_POS_RIGHT,
    ACCEL_CAL_POS_NOSEDOWN,
    ACCEL_CAL_POS_NOSEUP,
    ACCEL_CAL_POS_BACK,
)


def _statustext(text, severity=6):
    return types.SimpleNamespace(severity=severity, text=text)


class TestClassify(unittest.TestCase):
    def test_prompt_positions_map_to_codes(self):
        cases = {
            "Place vehicle level and press any key.": ACCEL_CAL_POS_LEVEL,
            "Place vehicle on its LEFT side and press any key.": ACCEL_CAL_POS_LEFT,
            "Place vehicle on its RIGHT side and press any key.": ACCEL_CAL_POS_RIGHT,
            "Place vehicle nose DOWN and press any key.": ACCEL_CAL_POS_NOSEDOWN,
            "Place vehicle nose UP and press any key.": ACCEL_CAL_POS_NOSEUP,
            "Place vehicle on its BACK and press any key.": ACCEL_CAL_POS_BACK,
        }
        for text, code in cases.items():
            evt = _classify_accel_cal_statustext(text)
            self.assertIsNotNone(evt, text)
            self.assertEqual(evt["status"], "prompt", text)
            self.assertEqual(evt["step"], code, text)
            self.assertEqual(evt["prompt_text"], text)

    def test_unknown_prompt_orientation_has_no_step(self):
        evt = _classify_accel_cal_statustext("Place vehicle sideways and press any key.")
        self.assertEqual(evt["status"], "prompt")
        self.assertIsNone(evt["step"])

    def test_result_lines(self):
        ok = _classify_accel_cal_statustext("Calibration successful")
        self.assertEqual(ok["status"], "success")
        self.assertIsNone(ok["step"])
        fail = _classify_accel_cal_statustext("Calibration FAILED")
        self.assertEqual(fail["status"], "failed")

    def test_compass_result_excluded(self):
        self.assertIsNone(_classify_accel_cal_statustext("Compass calibration successful"))

    def test_unrelated_text_ignored(self):
        self.assertIsNone(_classify_accel_cal_statustext("EKF3 IMU0 is using GPS"))
        self.assertIsNone(_classify_accel_cal_statustext(""))


class TestVehicleEntryBuffering(unittest.TestCase):
    def _entry(self):
        return VehicleEntry(1, MagicMock(), "uav1")

    def test_prompt_is_buffered(self):
        entry = self._entry()
        entry._on_statustext(_statustext("Place vehicle level and press any key."))
        self.assertEqual(len(entry._accel_cal_events), 1)
        self.assertEqual(entry._accel_cal_events[0]["step"], ACCEL_CAL_POS_LEVEL)

    def test_result_buffered_only_when_active(self):
        entry = self._entry()
        # No active cal (no command sent, no prior prompt) -> result dropped.
        entry._on_statustext(_statustext("Calibration successful"))
        self.assertEqual(len(entry._accel_cal_events), 0)

        # After a prompt, the cal is active -> the result is surfaced.
        entry._on_statustext(_statustext("Place vehicle level and press any key."))
        entry._on_statustext(_statustext("Calibration successful"))
        statuses = [e["status"] for e in entry._accel_cal_events]
        self.assertEqual(statuses, ["prompt", "success"])

    def test_mark_active_enables_level_result(self):
        entry = self._entry()
        # Quick level cal emits no prompt; mark_accel_cal_active opens the window.
        entry.mark_accel_cal_active()
        entry._on_statustext(_statustext("Calibration successful"))
        self.assertEqual([e["status"] for e in entry._accel_cal_events], ["success"])

    def test_result_closes_window(self):
        entry = self._entry()
        entry.mark_accel_cal_active()
        entry._on_statustext(_statustext("Calibration successful"))
        # A second result with no new activity must not be surfaced again.
        entry._on_statustext(_statustext("Calibration successful"))
        self.assertEqual(len(entry._accel_cal_events), 1)

    def test_snapshot_drains_events(self):
        entry = self._entry()
        entry._on_statustext(_statustext("Place vehicle on its LEFT side and press any key."))
        snap = entry.snapshot()
        self.assertIn("accel_cal_events", snap)
        self.assertEqual(snap["accel_cal_events"][0]["step"], ACCEL_CAL_POS_LEFT)
        # Drained: a subsequent snapshot omits the events.
        snap2 = entry.snapshot()
        self.assertNotIn("accel_cal_events", snap2)


if __name__ == "__main__":
    unittest.main()
