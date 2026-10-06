"""Tests for the radio (RC) calibration save handler in control.py."""
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend.routes.control import _handle_rc_cal_save


def _disarmed_entry():
    """A connected, disarmed vehicle whose set_parameter always succeeds."""
    entry = MagicMock()
    entry.vehicle.is_armed = False
    entry.vehicle.set_parameter.return_value = True
    return entry


class TestRcCalSave(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_writes_expected_params(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        params = {"channels": {"1": {"min": 1100, "max": 1900, "trim": 1500, "reversed": False}}}
        result = _handle_rc_cal_save([1], params)

        # Every RCn_* param written with the captured value.
        calls = {c.args[0]: c.args[1] for c in entry.vehicle.set_parameter.call_args_list}
        self.assertEqual(calls["RC1_MIN"], 1100)
        self.assertEqual(calls["RC1_MAX"], 1900)
        self.assertEqual(calls["RC1_TRIM"], 1500)
        self.assertEqual(calls["RC1_REVERSED"], 0)
        # set_parameter called with the documented timeout.
        for c in entry.vehicle.set_parameter.call_args_list:
            self.assertEqual(c.kwargs.get("timeout"), 2.0)
        self.assertEqual(result["1"]["status"], "saved")
        self.assertTrue(all(result["1"]["params"].values()))

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reversed_true_writes_one(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        _handle_rc_cal_save([1], {"channels": {"2": {"min": 1000, "max": 2000, "reversed": True}}})

        calls = {c.args[0]: c.args[1] for c in entry.vehicle.set_parameter.call_args_list}
        self.assertEqual(calls["RC2_REVERSED"], 1)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_trim_defaults_to_midpoint(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        _handle_rc_cal_save([1], {"channels": {"3": {"min": 1000, "max": 2000}}})

        calls = {c.args[0]: c.args[1] for c in entry.vehicle.set_parameter.call_args_list}
        self.assertEqual(calls["RC3_TRIM"], 1500)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_trim_clamped_into_min_max(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        _handle_rc_cal_save([1], {"channels": {"4": {"min": 1100, "max": 1900, "trim": 2200}}})

        calls = {c.args[0]: c.args[1] for c in entry.vehicle.set_parameter.call_args_list}
        self.assertEqual(calls["RC4_TRIM"], 1900)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_refuses_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_rc_cal_save([1], {"channels": {"1": {"min": 1000, "max": 2000}}})

        self.assertEqual(result["1"]["status"], "refused_armed")
        entry.vehicle.set_parameter.assert_not_called()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None

        result = _handle_rc_cal_save([7], {"channels": {"1": {"min": 1000, "max": 2000}}})

        self.assertEqual(result["7"]["status"], "not_connected")

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_invalid_min_max_skipped(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        # min >= max is rejected; with no valid channels nothing is written.
        result = _handle_rc_cal_save([1], {"channels": {"1": {"min": 1900, "max": 1100}}})

        entry.vehicle.set_parameter.assert_not_called()
        self.assertEqual(result["1"]["status"], "no_valid_channels")
        self.assertEqual(result["1"]["channel_errors"]["1"], "invalid_min_max")

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_channel_out_of_range_skipped(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_rc_cal_save([1], {"channels": {"99": {"min": 1000, "max": 2000}}})

        entry.vehicle.set_parameter.assert_not_called()
        self.assertEqual(result["1"]["channel_errors"]["99"], "channel_out_of_range")

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_out_of_range_pwm_rejected(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        # 500/3000 µs are outside the plausible PWM window -> channel skipped.
        result = _handle_rc_cal_save([1], {"channels": {"1": {"min": 500, "max": 3000}}})

        entry.vehicle.set_parameter.assert_not_called()
        self.assertEqual(result["1"]["channel_errors"]["1"], "invalid_min_max")

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_partial_when_a_write_fails(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = False
        # RC1_MAX fails to echo; everything else succeeds.
        entry.vehicle.set_parameter.side_effect = (
            lambda name, value, timeout=2.0: name != "RC1_MAX"
        )
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_rc_cal_save([1], {"channels": {"1": {"min": 1000, "max": 2000}}})

        self.assertEqual(result["1"]["status"], "partial")
        self.assertFalse(result["1"]["params"]["RC1_MAX"])
        self.assertTrue(result["1"]["params"]["RC1_MIN"])

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_write_exception_is_contained(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = False
        entry.vehicle.set_parameter.side_effect = RuntimeError("link down")
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_rc_cal_save([1], {"channels": {"1": {"min": 1000, "max": 2000}}})

        # No exception bubbles up; the failed writes are reported as False.
        self.assertEqual(result["1"]["status"], "partial")
        self.assertFalse(any(result["1"]["params"].values()))

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_multiple_channels(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        params = {"channels": {
            "1": {"min": 1100, "max": 1900},
            "3": {"min": 1000, "max": 2000, "reversed": True},
        }}
        result = _handle_rc_cal_save([1], params)

        names = {c.args[0] for c in entry.vehicle.set_parameter.call_args_list}
        self.assertEqual(result["1"]["status"], "saved")
        for n in ("RC1_MIN", "RC1_MAX", "RC1_TRIM", "RC1_REVERSED",
                  "RC3_MIN", "RC3_MAX", "RC3_TRIM", "RC3_REVERSED"):
            self.assertIn(n, names)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_malformed_channels_list_does_not_crash(self, mock_mgr):
        """params['channels'] as a list (not a dict) must not raise —
        _write_rc_cal's .items() call would AttributeError uncaught, and the
        /api/control/command dispatcher invokes handlers without a try/except,
        so an unhandled exception here would 500 the whole request."""
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_rc_cal_save([1], {"channels": ["not", "a", "dict"]})

        entry.vehicle.set_parameter.assert_not_called()
        self.assertIn("error", result["1"]["status"])

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_malformed_channels_does_not_block_other_vehicles(self, mock_mgr):
        """A malformed payload must still report a per-vehicle error for every
        target, not abort the loop after the first exception."""
        entry1, entry2 = _disarmed_entry(), _disarmed_entry()
        mock_mgr.get_vehicle.side_effect = [entry1, entry2]

        result = _handle_rc_cal_save([1, 2], {"channels": ["not", "a", "dict"]})

        self.assertIn("error", result["1"]["status"])
        self.assertIn("error", result["2"]["status"])


if __name__ == "__main__":
    unittest.main()
