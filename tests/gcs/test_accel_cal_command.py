"""Tests for accelerometer / level calibration command handlers in control.py."""
import unittest
from unittest.mock import MagicMock, patch

from fastapi import HTTPException

from gcs.backend.routes.control import (
    _handle_accel_level,
    _handle_accel_cal_start,
    _handle_accel_cal_pos,
    _COMMAND_HANDLERS,
)
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_PREFLIGHT_CALIBRATION,
    MAV_CMD_ACCELCAL_VEHICLE_POS,
)


def _disarmed_entry():
    """A connected, disarmed vehicle (T-1-07: cal handlers refuse armed)."""
    entry = MagicMock()
    entry.vehicle.is_armed = False
    return entry


class TestHandleAccelLevel(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_level_sends_preflight_calibration_p5_2(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_accel_level([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p5=2,
        )
        entry.mark_accel_cal_active.assert_called_once()
        self.assertEqual(result, {"1": "level_cal_sent"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_level_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None
        self.assertEqual(_handle_accel_level([5], None), {"5": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_level_refused_when_armed(self, mock_mgr):
        """T-1-07: calibrating a flying UAV must never be possible from the GCS."""
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_accel_level([1], None)

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()
        entry.mark_accel_cal_active.assert_not_called()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_level_exception(self, mock_mgr):
        entry = _disarmed_entry()
        entry.vehicle.send_command_long.side_effect = RuntimeError("timeout")
        mock_mgr.get_vehicle.return_value = entry
        self.assertIn("error", _handle_accel_level([1], None)["1"])


class TestHandleAccelCalStart(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_sends_preflight_calibration_p5_1(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_accel_cal_start([2], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_CALIBRATION, p5=1,
        )
        entry.mark_accel_cal_active.assert_called_once()
        self.assertEqual(result, {"2": "accel_cal_started"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None
        self.assertEqual(_handle_accel_cal_start([7], None), {"7": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_refused_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_accel_cal_start([1], None)

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()


class TestHandleAccelCalPos(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_sends_accelcal_vehicle_pos_p1(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_accel_cal_pos([1], {"pos": 3})

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_ACCELCAL_VEHICLE_POS, p1=3,
        )
        self.assertEqual(result, {"1": "pos_3_sent"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_accepts_full_range(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry
        for pos in range(1, 7):
            entry.reset_mock()
            entry.vehicle.is_armed = False
            result = _handle_accel_cal_pos([1], {"pos": pos})
            entry.vehicle.send_command_long.assert_called_once_with(
                MAV_CMD_ACCELCAL_VEHICLE_POS, p1=pos,
            )
            self.assertEqual(result, {"1": f"pos_{pos}_sent"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_missing_param_raises_400(self, mock_mgr):
        with self.assertRaises(HTTPException) as ctx:
            _handle_accel_cal_pos([1], None)
        self.assertEqual(ctx.exception.status_code, 400)
        with self.assertRaises(HTTPException):
            _handle_accel_cal_pos([1], {})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_out_of_range_raises_400(self, mock_mgr):
        for bad in (0, 7, -1, 99):
            with self.assertRaises(HTTPException) as ctx:
                _handle_accel_cal_pos([1], {"pos": bad})
            self.assertEqual(ctx.exception.status_code, 400)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_non_numeric_raises_400(self, mock_mgr):
        with self.assertRaises(HTTPException) as ctx:
            _handle_accel_cal_pos([1], {"pos": "abc"})
        self.assertEqual(ctx.exception.status_code, 400)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_string_digit_is_coerced(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry
        result = _handle_accel_cal_pos([1], {"pos": "4"})
        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_ACCELCAL_VEHICLE_POS, p1=4,
        )
        self.assertEqual(result, {"1": "pos_4_sent"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None
        self.assertEqual(_handle_accel_cal_pos([5], {"pos": 1}), {"5": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_pos_refused_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_accel_cal_pos([1], {"pos": 2})

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()


class TestRegistration(unittest.TestCase):
    def test_handlers_registered(self):
        self.assertIs(_COMMAND_HANDLERS["accel_level"], _handle_accel_level)
        self.assertIs(_COMMAND_HANDLERS["accel_cal_start"], _handle_accel_cal_start)
        self.assertIs(_COMMAND_HANDLERS["accel_cal_pos"], _handle_accel_cal_pos)


if __name__ == "__main__":
    unittest.main()
