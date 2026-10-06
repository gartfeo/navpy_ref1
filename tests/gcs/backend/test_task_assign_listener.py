"""Tests for TaskAssignListener available-task filtering."""
import asyncio
import unittest
from unittest.mock import MagicMock, patch

from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.task_assign_listener import TaskAssignListener

AUTOPILOT_COMPONENT_ID = 1


def _make_available_task_msg(sender_id: int,
                             src_component: int = COMPANION_COMPONENT_ID):
    """Create a mock NAVLINK msg that decodes to AvailableTaskRequestMsg."""
    from navpy.modules.comm.messages.available_task_msg import (
        AvailableTaskRequestMsg,
        MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST,
    )
    from navpy.modules.comm.messages.msg_abc import MsgRegistry

    # Ensure registry knows the class
    assert MsgRegistry.get_class_by_mav_id(MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST) is AvailableTaskRequestMsg

    msg = MagicMock()
    msg.get_msgId.return_value = MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST
    msg.get_srcSystem.return_value = sender_id
    msg.get_srcComponent.return_value = src_component
    msg.count = 0
    msg.task_id = [0] * 5
    msg.task_type = [0] * 5
    msg.lat = [0.0] * 5
    msg.lng = [0.0] * 5
    msg.alt = [0.0] * 5
    msg.boot_id = 0
    msg.msg_seq = 0
    msg.time_ms = 0
    msg.ttl_ms = 0
    return msg


class TestAvailableTaskFiltering(unittest.TestCase):
    """TaskAssignListener skips available-task broadcasts when no peers exist."""

    def setUp(self):
        self.loop = MagicMock(spec=asyncio.AbstractEventLoop)
        self.listener = TaskAssignListener(self.loop)

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_single_vehicle_own_available_task_skipped(self, mock_ws):
        """Single vehicle's own available task is dropped (no peers)."""
        vehicle = MagicMock()
        self.listener.register_vehicle(1, vehicle)

        msg = _make_available_task_msg(sender_id=1)
        self.listener._on_navlink(1, msg)

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_single_vehicle_external_available_task_skipped(self, mock_ws):
        """Available task from unknown sender is dropped when only one vehicle connected."""
        vehicle = MagicMock()
        self.listener.register_vehicle(1, vehicle)

        msg = _make_available_task_msg(sender_id=99)
        self.listener._on_navlink(1, msg)

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_assign_listener.asyncio.run_coroutine_threadsafe")
    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_peer_available_task_broadcast_with_multiple_vehicles(self, mock_ws, mock_rcts):
        """Available task from a registered peer is broadcast when multiple vehicles connected."""
        v1, v2 = MagicMock(), MagicMock()
        self.listener.register_vehicle(1, v1)
        self.listener.register_vehicle(2, v2)

        msg = _make_available_task_msg(sender_id=1)
        self.listener._on_navlink(1, msg)

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        self.assertEqual(payload["type"], "available_task_request")
        self.assertEqual(payload["sender_id"], 1)

    @patch("gcs.backend.task_assign_listener.asyncio.run_coroutine_threadsafe")
    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_both_peers_available_tasks_broadcast(self, mock_ws, mock_rcts):
        """Available tasks from both vehicles are broadcast when peers exist."""
        v1, v2 = MagicMock(), MagicMock()
        self.listener.register_vehicle(1, v1)
        self.listener.register_vehicle(2, v2)

        self.listener._on_navlink(1, _make_available_task_msg(sender_id=1))
        self.listener._on_navlink(2, _make_available_task_msg(sender_id=2))

        self.assertEqual(mock_ws.broadcast.call_count, 2)

    @patch("gcs.backend.task_assign_listener.asyncio.run_coroutine_threadsafe")
    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_own_autopilot_component_is_not_companion_traffic(self, mock_ws, mock_rcts):
        """The aircraft's own autopilot shares the companion's srcSystem now,
        so component 1 traffic must never be attributed to the companion."""
        v1, v2 = MagicMock(), MagicMock()
        self.listener.register_vehicle(1, v1)
        self.listener.register_vehicle(2, v2)

        msg = _make_available_task_msg(
            sender_id=1, src_component=AUTOPILOT_COMPONENT_ID,
        )
        self.listener._on_navlink(1, msg)

        mock_ws.broadcast.assert_not_called()

    def test_register_vehicle_tracks_ids(self):
        """register_vehicle adds sys_id to the registered set."""
        v = MagicMock()
        self.listener.register_vehicle(5, v)
        self.listener.register_vehicle(10, v)
        self.assertEqual(self.listener._registered_ids, {5, 10})


if __name__ == "__main__":
    unittest.main()
