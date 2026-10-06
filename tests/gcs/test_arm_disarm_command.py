"""Tests for arm/disarm command handlers in control.py."""
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend.routes.control import _handle_arm, _handle_disarm, _handle_force_disarm
from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM


class TestHandleArm(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_arm_sends_correct_command(self, mock_mgr):
        entry = MagicMock()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_arm([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_COMPONENT_ARM_DISARM, p1=1,
        )
        self.assertEqual(result, {"1": "armed"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_arm_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None

        result = _handle_arm([5], None)

        self.assertEqual(result, {"5": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_arm_multiple_targets(self, mock_mgr):
        entry1 = MagicMock()
        entry2 = MagicMock()
        mock_mgr.get_vehicle.side_effect = [entry1, entry2]

        result = _handle_arm([1, 2], None)

        self.assertEqual(result, {"1": "armed", "2": "armed"})
        entry1.vehicle.send_command_long.assert_called_once()
        entry2.vehicle.send_command_long.assert_called_once()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_arm_exception(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.send_command_long.side_effect = RuntimeError("timeout")
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_arm([1], None)

        self.assertIn("error", result["1"])


class TestHandleDisarm(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_disarm_sends_correct_command(self, mock_mgr):
        entry = MagicMock()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_disarm([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_COMPONENT_ARM_DISARM, p1=0,
        )
        self.assertEqual(result, {"1": "disarmed"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_disarm_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None

        result = _handle_disarm([3], None)

        self.assertEqual(result, {"3": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_disarm_no_force_flag(self, mock_mgr):
        """Ensure disarm does NOT use force flag (p2=21196) — that's for E-STOP only."""
        entry = MagicMock()
        mock_mgr.get_vehicle.return_value = entry

        _handle_disarm([1], None)

        call_kwargs = entry.vehicle.send_command_long.call_args
        # Should only have p1=0, no p2 (force flag)
        self.assertNotIn('p2', call_kwargs.kwargs)


class TestHandleForceDisarm(unittest.TestCase):
    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_force_disarm_sends_correct_command(self, mock_mgr):
        entry = MagicMock()
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_force_disarm([1], None)

        entry.vehicle.send_command_long.assert_called_once_with(
            MAV_CMD_COMPONENT_ARM_DISARM, p1=0, p2=21196,
        )
        self.assertEqual(result, {"1": "force_disarmed"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_force_disarm_not_connected(self, mock_mgr):
        mock_mgr.get_vehicle.return_value = None

        result = _handle_force_disarm([5], None)

        self.assertEqual(result, {"5": "not_connected"})

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_force_disarm_multiple_targets(self, mock_mgr):
        entry1 = MagicMock()
        entry2 = MagicMock()
        mock_mgr.get_vehicle.side_effect = [entry1, entry2]

        result = _handle_force_disarm([1, 2], None)

        self.assertEqual(result, {"1": "force_disarmed", "2": "force_disarmed"})
        entry1.vehicle.send_command_long.assert_called_once()
        entry2.vehicle.send_command_long.assert_called_once()

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_force_disarm_exception(self, mock_mgr):
        entry = MagicMock()
        entry.vehicle.send_command_long.side_effect = RuntimeError("timeout")
        mock_mgr.get_vehicle.return_value = entry

        result = _handle_force_disarm([1], None)

        self.assertIn("error", result["1"])

    @patch("gcs.backend.routes.control.vehicle_mgr")
    def test_force_disarm_does_not_change_mode(self, mock_mgr):
        """Force disarm should NOT set mode (unlike estop)."""
        entry = MagicMock()
        mock_mgr.get_vehicle.return_value = entry

        _handle_force_disarm([1], None)

        entry.vehicle.set_mode.assert_not_called()


if __name__ == "__main__":
    unittest.main()
