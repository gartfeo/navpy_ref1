"""Tests for TaskConfirmListener — NAVLINK parsing, image reassembly."""
from __future__ import annotations

import asyncio
from unittest.mock import MagicMock, patch, AsyncMock

import pytest

from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.task_confirm_listener import TaskConfirmListener
from navpy.modules.comm.image_transfer import ImageTransferType, CHUNK_SIZE
from navpy.modules.comm.messages.available_task_msg import (
    TaskConfirmRequestMsg,
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
    return TaskConfirmListener(loop)


@pytest.fixture
def mock_vehicle():
    vehicle = MagicMock()
    vehicle.on_message = MagicMock()
    return vehicle


def _make_navlink_msg(task_id, lat, lon, alt, sys_id=1, src_component=COMPANION_COMPONENT_ID):
    """Create a mock MAVLink message that looks like a TaskConfirmRequest."""
    req = TaskConfirmRequestMsg(
        sender_id=sys_id,
        task=TaskMsgData(
            task_id=task_id,
            task_type=TaskTypeMsgData.UNKNOWN,
            location=LocationMsgData(lat=lat, lng=lon, alt=alt),
        ),
    )
    mav_msg = req.to_mavlink()
    # The companion shares its aircraft's sysid and is told apart by
    # component 191 -- both are needed for the ownership filter.
    mav_msg.get_srcSystem = MagicMock(return_value=sys_id)
    mav_msg.get_srcComponent = MagicMock(return_value=src_component)
    return mav_msg


class TestRegisterVehicle:
    def test_registers_three_callbacks(self, listener, mock_vehicle):
        listener.register_vehicle(1, mock_vehicle)
        assert mock_vehicle.on_message.call_count == 3
        registered = [call.args[0] for call in mock_vehicle.on_message.call_args_list]
        assert "NAVLINK" in registered
        assert "DATA_TRANSMISSION_HANDSHAKE" in registered
        assert "ENCAPSULATED_DATA" in registered

    def test_creates_reassembler(self, listener, mock_vehicle):
        listener.register_vehicle(42, mock_vehicle)
        assert 42 in listener._images.reassemblers


class TestOnNavlink:
    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_broadcasts_task_confirm_request(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        mav_msg = _make_navlink_msg(
            task_id=7, lat=32.5, lon=34.8, alt=100.0, sys_id=1,
        )

        listener._on_navlink(1, mav_msg)

        # Wait for the coroutine to be scheduled
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_confirm_request"
        assert payload["sys_id"] == 1
        assert payload["task_id"] == 7
        assert payload["lat"] == pytest.approx(32.5, abs=0.01)
        assert payload["lon"] == pytest.approx(34.8, abs=0.01)

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_ignores_non_confirm_navlink(self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        # Create a message with an unregistered msgId
        msg = MagicMock()
        msg.get_msgId = MagicMock(return_value=99999)

        listener._on_navlink(1, msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_own_autopilot_component_is_not_attributed_to_the_companion(
            self, mock_ws, listener, loop):
        """The aircraft's own autopilot now shares the companion's srcSystem,
        so only component 191 may be treated as companion confirm traffic."""
        mock_ws.broadcast = AsyncMock()
        mav_msg = _make_navlink_msg(
            task_id=7, lat=32.5, lon=34.8, alt=100.0, sys_id=1,
            src_component=1,
        )

        listener._on_navlink(1, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_ignores_request_seen_through_another_vehicle_listener(
            self, mock_ws, listener, loop):
        mock_ws.broadcast = AsyncMock()
        mav_msg = _make_navlink_msg(
            task_id=7, lat=32.5, lon=34.8, alt=100.0, sys_id=2,
        )

        listener._on_navlink(1, mav_msg)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()


class TestImageReassembly:
    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_handshake_sets_current_target(self, mock_ws, listener, mock_vehicle):
        listener.register_vehicle(1, mock_vehicle)

        handshake = MagicMock()
        handshake.type = ImageTransferType.TARGET_CONFIRMATION
        handshake.width = 5  # target_id stored in width
        handshake.size = 100
        handshake.packets = 1
        handshake.payload = CHUNK_SIZE
        handshake.jpg_quality = 80
        handshake.get_srcSystem.return_value = 1
        handshake.get_srcComponent.return_value = COMPANION_COMPONENT_ID

        listener._on_handshake(1, handshake)
        # The active transfer now also carries the round it belongs to, so a
        # late completion cannot land on a later round's card.
        active = listener._images.current_targets.get(1)
        assert active is not None
        assert active.target_id == 5

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_ignores_non_confirmation_handshake(self, mock_ws, listener, mock_vehicle):
        listener.register_vehicle(1, mock_vehicle)

        handshake = MagicMock()
        handshake.type = 0  # Not TARGET_CONFIRMATION
        handshake.get_srcSystem.return_value = 1
        handshake.get_srcComponent.return_value = COMPANION_COMPONENT_ID

        listener._on_handshake(1, handshake)
        assert 1 not in listener._images.current_targets

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_complete_image_broadcasts(self, mock_ws, listener, mock_vehicle, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        listener._rounds.classify(1, 3, "legacy", 1.0, 120.0)

        # Send handshake
        handshake = MagicMock()
        handshake.type = ImageTransferType.TARGET_CONFIRMATION
        handshake.width = 3
        handshake.size = 10  # small image
        handshake.packets = 1
        handshake.payload = CHUNK_SIZE
        handshake.jpg_quality = 80
        handshake.get_srcSystem.return_value = 1
        handshake.get_srcComponent.return_value = COMPANION_COMPONENT_ID
        listener._on_handshake(1, handshake)

        # Send single chunk
        chunk = MagicMock()
        chunk.seqnr = 0
        chunk.data = list(b'\xff\xd8' + b'\x00' * (CHUNK_SIZE - 2))
        chunk.get_srcSystem.return_value = 1
        chunk.get_srcComponent.return_value = COMPANION_COMPONENT_ID
        listener._on_chunk(1, chunk)

        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_confirm_image"
        assert payload["sys_id"] == 1
        assert payload["task_id"] == 3
        assert payload["image_b64"] is not None

    def test_ignores_image_handshake_from_another_companion(
            self, listener, mock_vehicle):
        listener.register_vehicle(1, mock_vehicle)
        handshake = MagicMock()
        handshake.type = ImageTransferType.TARGET_CONFIRMATION
        handshake.width = 5
        handshake.size = 100
        handshake.packets = 1
        handshake.payload = CHUNK_SIZE
        handshake.jpg_quality = 80
        handshake.get_srcSystem.return_value = 2
        handshake.get_srcComponent.return_value = COMPANION_COMPONENT_ID

        listener._on_handshake(1, handshake)

        assert 1 not in listener._images.current_targets
