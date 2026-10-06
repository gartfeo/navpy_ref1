"""Tests for compass/magnetometer calibration command handlers in control.py."""
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend.routes.control import (
    _COMMAND_HANDLERS,
    _handle_compass_cal_start,
    _handle_compass_cal_cancel,
    _handle_compass_cal_accept,
    _handle_compass_cal_reboot,
)
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_DO_START_MAG_CAL,
    MAV_CMD_DO_CANCEL_MAG_CAL,
    MAV_CMD_DO_ACCEPT_MAG_CAL,
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
)


def _disarmed_entry():
    """A connected, disarmed vehicle (T-1-07: cal handlers refuse armed)."""
    entry = MagicMock()
    entry.vehicle.is_armed = False
    return entry


class TestCompassCalStart(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_sends_correct_command(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_start([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_DO_START_MAG_CAL, p1=0, p2=0, p3=1, p4=0, p5=0,
        )
        self.assertEqual(result, {"1": "cal_started"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_autosave_on_and_autoreboot_off(self, mock_mgr):
        """p3 (autosave) must be 1 and p5 (autoreboot) must be 0."""
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        _handle_compass_cal_start([1], None)

        kwargs = entry.vehicle.send_command_long.call_args.kwargs
        self.assertEqual(kwargs["p3"], 1)
        self.assertEqual(kwargs["p5"], 0)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_respects_mag_mask_param(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        _handle_compass_cal_start([1], {"mag_mask": 2})

        self.assertEqual(entry.vehicle.send_command_long.call_args.kwargs["p1"], 2)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None

        result = _handle_compass_cal_start([5], None)

        self.assertEqual(result, {"5": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_refused_when_armed(self, mock_mgr):
        """T-1-07: calibrating a flying UAV must never be possible from the GCS."""
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_start([1], None)

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_multiple_pois(self, mock_mgr):
        entry1, entry2 = _disarmed_entry(), _disarmed_entry()
        mock_mgr.get_vehicle.side_effect = [entry1, entry2]

        result = _handle_compass_cal_start([1, 2], None)

        self.assertEqual(result, {"1": "cal_started", "2": "cal_started"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_start_exception(self, mock_mgr):
        entry = _disarmed_entry()
        entry.vehicle.send_command_long.side_effect = RuntimeError("timeout")
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_start([1], None)

        self.assertIn("error", result["1"])


class TestCompassCalCancel(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_cancel_sends_correct_command(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_cancel([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_DO_CANCEL_MAG_CAL,
        )
        self.assertEqual(result, {"1": "cal_cancelled"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_cancel_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None
        self.assertEqual(_handle_compass_cal_cancel([3], None), {"3": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_cancel_refused_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_cancel([1], None)

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()


class TestCompassCalAccept(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_accept_sends_correct_command(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_accept([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_DO_ACCEPT_MAG_CAL,
        )
        self.assertEqual(result, {"1": "cal_accepted"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_accept_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None
        self.assertEqual(_handle_compass_cal_accept([3], None), {"3": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_accept_refused_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_accept([1], None)

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()


class TestCompassCalReboot(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_sends_correct_command(self, mock_mgr):
        entry = _disarmed_entry()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_reboot([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, p1=1,
        )
        self.assertEqual(result, {"1": "reboot_sent"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None
        self.assertEqual(_handle_compass_cal_reboot([3], None), {"3": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_refused_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_compass_cal_reboot([1], None)

        self.assertEqual(result, {"1": "refused_armed"})
        entry.vehicle.send_command_long.assert_not_called()


class TestCompassCalRegistration(unittest.TestCase):
    def test_handlers_registered_in_dispatch_table(self):
        for cmd in ("compass_cal_start", "compass_cal_cancel",
                    "compass_cal_accept", "compass_cal_reboot"):
            self.assertIn(cmd, _COMMAND_HANDLERS)


if __name__ == "__main__":
    unittest.main()
