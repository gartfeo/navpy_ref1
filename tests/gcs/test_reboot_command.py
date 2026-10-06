"""Tests for the reboot (autopilot restart) command handler in control.py."""
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend.routes.control import _handle_reboot, _COMMAND_HANDLERS
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN


class TestHandleReboot(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_sends_correct_command_when_disarmed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = False
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_reboot([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, p1=1,
        )
        self.assertEqual(result, {"1": "reboot_sent"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_refused_when_armed(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = True
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_reboot([1], None)

        # Must not send any command to an armed vehicle.
        entry.vehicle.send_command_long.assert_not_called()
        self.assertEqual(result, {"1": "refused_armed"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None

        result = _handle_reboot([5], None)

        self.assertEqual(result, {"5": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_does_not_request_bootloader(self, mock_mgr):
        """Reboot must use p1=1 (autopilot), never p1=3 (bootloader)."""
        entry = MagicMock()
        entry.vehicle.is_armed = False
        mock_mgr.get_vehicle.return_value = entry

        _handle_reboot([1], None)

        call = entry.vehicle.send_command_long.call_args
        self.assertEqual(call.kwargs.get("p1"), 1)

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_does_not_change_mode(self, mock_mgr):
        """Reboot should NOT touch flight mode (unlike estop)."""
        entry = MagicMock()
        entry.vehicle.is_armed = False
        mock_mgr.get_vehicle.return_value = entry

        _handle_reboot([1], None)

        entry.vehicle.set_mode.assert_not_called()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_mixed_targets(self, mock_mgr):
        """Disarmed reboots, armed is refused, missing is not_connected."""
        disarmed = MagicMock()
        disarmed.vehicle.is_armed = False
        armed = MagicMock()
        armed.vehicle.is_armed = True
        mock_mgr.get_vehicle.side_effect = [disarmed, armed, None]

        result = _handle_reboot([1, 2, 3], None)

        self.assertEqual(
            result,
            {"1": "reboot_sent", "2": "refused_armed", "3": "not_connected"},
        )
        disarmed.vehicle.send_command_long.assert_called_once()
        armed.vehicle.send_command_long.assert_not_called()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_reboot_exception(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.is_armed = False
        entry.vehicle.send_command_long.side_effect = RuntimeError("timeout")
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_reboot([1], None)

        self.assertIn("error", result["1"])

    def test_reboot_registered_in_dispatch_table(self):
        self.assertIs(_COMMAND_HANDLERS["reboot"], _handle_reboot)


if __name__ == "__main__":
    unittest.main()
