"""Tests for TaskAssignListener — NAVLINK parsing, WS broadcast."""
from __future__ import annotations

import asyncio
from unittest.mock import MagicMock, patch, AsyncMock

import pytest

from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.task_assign_listener import TaskAssignListener
from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
    TaskAssignMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import TaskTypeMsgData


@pytest.fixture
def loop():
    """Provide an asyncio event loop for the listener."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def listener(loop):
    return TaskAssignListener(loop)


@pytest.fixture
def mock_vehicle():
    vehicle = MagicMock()
    vehicle.on_message = MagicMock()
    return vehicle


AUTOPILOT_COMPONENT_ID = 1


def _make_assign_request_msg(sender_id, receiver_id, task_id, lat, lon, alt,
                             src_component=COMPANION_COMPONENT_ID):
    """Create a mock MAVLink message for TaskAssignRequest."""
    req = TaskAssignRequestMsg(
        sender_id=sender_id,
        receiver_id=receiver_id,
        task=TaskAssignMsgData(
            task_id=task_id,
            task_type=TaskTypeMsgData.MEDIUM,
            location=LocationMsgData(lat=lat, lng=lon, alt=alt),
        ),
    )
    mav_msg = req.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=sender_id)
    mav_msg.get_srcComponent = MagicMock(return_value=src_component)
    return mav_msg


def _make_assign_response_msg(sender_id, receiver_id, task_id, is_accepted):
    """Create a mock MAVLink message for TaskAssignResponse."""
    resp = TaskAssignResponseMsg(
        sender_id=sender_id,
        receiver_id=receiver_id,
        task_id=task_id,
        is_accepted=is_accepted,
    )
    mav_msg = resp.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=sender_id)
    mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return mav_msg


class TestRegisterVehicle:
    def test_registers_navlink_callback(self, listener, mock_vehicle):
        listener.register_vehicle(1, mock_vehicle)
        assert mock_vehicle.on_message.call_count == 1
        registered = [call.args[0] for call in mock_vehicle.on_message.call_args_list]
        assert "NAVLINK" in registered


class TestOnNavlink:
    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_broadcasts_assign_request(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        mav_msg = _make_assign_request_msg(
            sender_id=1, receiver_id=2, task_id=7, lat=32.5, lon=34.8, alt=100.0,
        )

        listener._on_navlink(1, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_assign_request"
        assert payload["sender_id"] == 1
        assert payload["receiver_id"] == 2
        assert payload["task_id"] == 7
        assert payload["task_type"] == "MEDIUM"
        assert payload["lat"] == pytest.approx(32.5, abs=0.01)
        assert payload["lon"] == pytest.approx(34.8, abs=0.01)
        assert payload["alt"] == pytest.approx(100.0, abs=0.1)

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_swarm_node_ids_need_no_translation(self, mock_ws, listener, loop):
        """Swarm node ids ARE the aircraft sysids now, so they reach the
        frontend unchanged - there is no companion-id mapping table left."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        mav_msg = _make_assign_request_msg(
            sender_id=2, receiver_id=1, task_id=7, lat=32.5, lon=34.8, alt=100.0,
        )

        # Deliver on the link that owns the sender (sysid 2). The shared-bus
        # gate makes every other listener drop this same packet.
        listener._on_navlink(2, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["sender_id"] == 2
        assert payload["receiver_id"] == 1

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_autopilot_component_is_not_attributed_to_the_companion(
        self, mock_ws, listener, loop,
    ):
        """Under the shared sysid the aircraft's OWN autopilot traffic
        (component 1) carries the same srcSystem as its companion. Only
        component 191 may be treated as companion swarm traffic."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        mav_msg = _make_assign_request_msg(
            sender_id=2, receiver_id=1, task_id=7, lat=32.5, lon=34.8, alt=100.0,
            src_component=AUTOPILOT_COMPONENT_ID,
        )

        listener._on_navlink(2, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_broadcasts_assign_response_accepted(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        mav_msg = _make_assign_response_msg(
            sender_id=2, receiver_id=1, task_id=7, is_accepted=True,
        )

        listener._on_navlink(2, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_assign_response"
        assert payload["sender_id"] == 2
        assert payload["receiver_id"] == 1
        assert payload["task_id"] == 7
        assert payload["is_accepted"] is True

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_assign_response_broadcast_uses_frontend_vehicle_ids(
        self, mock_ws, listener, loop,
    ):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        mav_msg = _make_assign_response_msg(
            sender_id=2,
            receiver_id=1,
            task_id=7,
            is_accepted=True,
        )

        listener._on_navlink(2, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["sender_id"] == 2
        assert payload["receiver_id"] == 1

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_broadcasts_assign_response_rejected(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        mav_msg = _make_assign_response_msg(
            sender_id=2, receiver_id=1, task_id=7, is_accepted=False,
        )

        listener._on_navlink(2, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["is_accepted"] is False

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_broadcasts_available_task_request(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        # Register 2 vehicles so the peer filter doesn't skip broadcast
        listener.register_vehicle(3, MagicMock())
        listener.register_vehicle(4, MagicMock())
        req = AvailableTaskRequestMsg(
            sender_id=3,
            tasks=[
                TaskMsgData(
                    task_id=10,
                    task_type=TaskTypeMsgData.MEDIUM,
                    location=LocationMsgData(lat=31.0, lng=34.0, alt=50.0),
                ),
                TaskMsgData(
                    task_id=11,
                    task_type=TaskTypeMsgData.BIG,
                    location=LocationMsgData(lat=31.5, lng=34.5, alt=60.0),
                ),
            ],
        )
        mav_msg = req.to_mavlink()
        mav_msg.get_srcSystem = MagicMock(return_value=3)
        mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)

        listener._on_navlink(3, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "available_task_request"
        assert payload["sender_id"] == 3
        assert len(payload["tasks"]) == 2
        t0 = payload["tasks"][0]
        assert t0["task_id"] == 10
        assert t0["task_type"] == "MEDIUM"
        assert t0["lat"] == pytest.approx(31.0, abs=0.01)
        assert t0["lon"] == pytest.approx(34.0, abs=0.01)
        assert t0["alt"] == pytest.approx(50.0, abs=0.1)
        t1 = payload["tasks"][1]
        assert t1["task_id"] == 11
        assert t1["task_type"] == "BIG"

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_broadcasts_available_task_request_empty(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        # Register 2 vehicles so the peer filter doesn't skip broadcast
        listener.register_vehicle(5, MagicMock())
        listener.register_vehicle(6, MagicMock())
        req = AvailableTaskRequestMsg(sender_id=5, tasks=[])
        mav_msg = req.to_mavlink()
        mav_msg.get_srcSystem = MagicMock(return_value=5)
        mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)

        listener._on_navlink(5, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "available_task_request"
        assert payload["sender_id"] == 5
        assert payload["tasks"] == []

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_ignores_unrelated_navlink(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        msg = MagicMock()
        msg.get_msgId = MagicMock(return_value=99999)

        listener._on_navlink(1, msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_ignores_request_from_another_vehicle_link(self, mock_ws, listener, loop):
        """A packet whose source is vehicle 2's companion arriving on vehicle
        1's shared-bus link must be dropped, not rebroadcast."""
        mock_ws.broadcast = AsyncMock()
        # Sysid 2's companion, arriving on vehicle 1's link.
        mav_msg = _make_assign_request_msg(
            sender_id=2, receiver_id=3, task_id=7, lat=32.5, lon=34.8, alt=100.0,
        )

        listener._on_navlink(1, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_assign_listener.ws_manager")
    def test_shared_bus_request_broadcast_exactly_once(self, mock_ws, listener, loop):
        """The same packet delivered to every vehicle link is broadcast once —
        only the listener that owns the sender acts on it."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, MagicMock())
        listener.register_vehicle(2, MagicMock())
        listener.register_vehicle(3, MagicMock())
        # This packet is owned by vehicle 1 only.
        mav_msg = _make_assign_request_msg(
            sender_id=1, receiver_id=2, task_id=7, lat=32.5, lon=34.8, alt=100.0,
        )

        for sid in (1, 2, 3):
            listener._on_navlink(sid, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_assign_request"
        assert payload["task_id"] == 7


