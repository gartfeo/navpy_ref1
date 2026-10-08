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
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData
from scripts.eval_gcs_demo_assignment_audit import _REQUEST, _RESPONSE


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


def _meta(uid):
    if uid is None:
        return None
    boot_id, msg_seq = uid
    return MsgMeta(boot_id=boot_id, msg_seq=msg_seq, time_ms=0, ttl_ms=5000)


def _make_assign_request_msg(sender_id, receiver_id, task_id, lat, lon, alt,
                             src_component=COMPANION_COMPONENT_ID, uid=None):
    """Create a mock MAVLink message for TaskAssignRequest."""
    req = TaskAssignRequestMsg(
        sender_id=sender_id,
        receiver_id=receiver_id,
        task=TaskAssignMsgData(
            task_id=task_id,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(lat=lat, lng=lon, alt=alt),
        ),
        meta=_meta(uid),
    )
    mav_msg = req.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=sender_id)
    mav_msg.get_srcComponent = MagicMock(return_value=src_component)
    return mav_msg


def _make_assign_response_msg(sender_id, receiver_id, task_id, is_accepted,
                              uid=None):
    """Create a mock MAVLink message for TaskAssignResponse."""
    resp = TaskAssignResponseMsg(
        sender_id=sender_id,
        receiver_id=receiver_id,
        task_id=task_id,
        is_accepted=is_accepted,
        meta=_meta(uid),
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
        assert payload["task_type"] == "DOCK"
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
                    task_type=TaskTypeMsgData.DOCK,
                    location=LocationMsgData(lat=31.0, lng=34.0, alt=50.0),
                ),
                TaskMsgData(
                    task_id=11,
                    task_type=TaskTypeMsgData.UNKNOWN,
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
        assert t0["task_type"] == "DOCK"
        assert t0["lat"] == pytest.approx(31.0, abs=0.01)
        assert t0["lon"] == pytest.approx(34.0, abs=0.01)
        assert t0["alt"] == pytest.approx(50.0, abs=0.1)
        t1 = payload["tasks"][1]
        assert t1["task_id"] == 11
        assert t1["task_type"] == "UNKNOWN"

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


OWNER, HELPER, OTHER = 1, 2, 3


def _make_ack_msg(sender_id, receiver_id, ref, uid,
                  status=ACK_STATUS_APPLIED,
                  ref_type=MsgType.TASK_ASSIGN_RESPONSE):
    """Create a mock MAVLink message for a SWARM_ACK of ``ref``."""
    ack = SwarmAckMsg(
        sender_id=sender_id,
        receiver_id=receiver_id,
        ref_boot_id=ref[0],
        ref_msg_seq=ref[1],
        ref_msg_type=ref_type.value,
        status=status,
        meta=_meta(uid),
    )
    mav_msg = ack.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=sender_id)
    mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return mav_msg


@pytest.fixture
def broadcast():
    """Record every WS payload without scheduling coroutines."""
    with patch("gcs.backend.task_assign_listener.ws_manager") as ws, patch(
        "gcs.backend.task_assign_listener.asyncio.run_coroutine_threadsafe",
    ):
        yield ws.broadcast


@pytest.fixture
def fleet(listener):
    for sys_id in (OWNER, HELPER, OTHER):
        listener.register_vehicle(sys_id, MagicMock())
    return listener


def _payloads(broadcast, kind):
    return [
        call.args[0] for call in broadcast.call_args_list
        if call.args[0]["type"] == kind
    ]


def _doing(listener, task_id, uid, accepted=True, helper=HELPER, owner=OWNER):
    listener._on_navlink(helper, _make_assign_response_msg(
        sender_id=helper, receiver_id=owner, task_id=task_id,
        is_accepted=accepted, uid=uid,
    ))


def _applied(listener, ref, uid, owner=OWNER, helper=HELPER, link=None, **kw):
    listener._on_navlink(
        owner if link is None else link,
        _make_ack_msg(owner, helper, ref, uid, **kw),
    )


class TestHandshakeUids:
    """Every copy reaches the UI with its UID; the logs stay audit-readable."""

    def test_request_copy_payload_and_log_carry_its_uid(
        self, fleet, broadcast, caplog,
    ):
        caplog.set_level("INFO", logger="gcs.backend.task_assign_listener")
        fleet._on_navlink(OWNER, _make_assign_request_msg(
            sender_id=OWNER, receiver_id=HELPER, task_id=9,
            lat=32.5, lon=34.8, alt=100.0, uid=(7, 50),
        ))

        payload, = _payloads(broadcast, "task_assign_request")
        assert payload["uid"] == {"boot_id": 7, "msg_seq": 50}
        line, = [r.getMessage() for r in caplog.records
                 if "Task assign request:" in r.getMessage()]
        assert line.endswith(" uid=7:50")
        match = _REQUEST.search(line)
        assert match is not None
        assert (match["sender"], match["receiver"], match["task"]) == (
            "1", "2", "9",
        )
        assert float(match["lat"]) == pytest.approx(32.5)
        assert float(match["lon"]) == pytest.approx(34.8)

    def test_response_copy_payload_and_log_carry_its_uid(
        self, fleet, broadcast, caplog,
    ):
        caplog.set_level("INFO", logger="gcs.backend.task_assign_listener")
        _doing(fleet, task_id=9, uid=(3, 20))

        payload, = _payloads(broadcast, "task_assign_response")
        assert payload["uid"] == {"boot_id": 3, "msg_seq": 20}
        line, = [r.getMessage() for r in caplog.records
                 if "Task assign response:" in r.getMessage()]
        assert line.endswith(" uid=3:20")
        match = _RESPONSE.search(line)
        assert match is not None
        assert match.groupdict() == {
            "sender": "2", "receiver": "1", "task": "9", "accepted": "True",
        }

    def test_advert_payload_carries_its_uid(self, fleet, broadcast):
        advert = AvailableTaskRequestMsg(
            sender_id=OWNER, tasks=[], meta=_meta((7, 40)),
        ).to_mavlink()
        advert.get_srcSystem = MagicMock(return_value=OWNER)
        advert.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)

        fleet._on_navlink(OWNER, advert)

        payload, = _payloads(broadcast, "available_task_request")
        assert payload["uid"] == {"boot_id": 7, "msg_seq": 40}

    def test_every_copy_is_forwarded(self, fleet, broadcast):
        for seq in (50, 51, 52):
            fleet._on_navlink(OWNER, _make_assign_request_msg(
                sender_id=OWNER, receiver_id=HELPER, task_id=9,
                lat=32.5, lon=34.8, alt=100.0, uid=(7, seq),
            ))
        for seq in (20, 21):
            _doing(fleet, task_id=9, uid=(3, seq))

        assert [p["uid"]["msg_seq"] for p in _payloads(
            broadcast, "task_assign_request",
        )] == [50, 51, 52]
        assert [p["uid"]["msg_seq"] for p in _payloads(
            broadcast, "task_assign_response",
        )] == [20, 21]


class TestAssignAck:
    """An owner's APPLIED of a helper's "doing" marks the helper ASSIGNED."""

    def test_applied_naming_an_accepted_copy_is_reported(
        self, fleet, broadcast, caplog,
    ):
        caplog.set_level("INFO", logger="gcs.backend.task_assign_listener")
        _doing(fleet, task_id=9, uid=(3, 20))
        _doing(fleet, task_id=9, uid=(3, 21))

        _applied(fleet, ref=(3, 20), uid=(7, 60))

        assert _payloads(broadcast, "task_assign_ack") == [{
            "type": "task_assign_ack",
            "owner_id": OWNER,
            "helper_id": HELPER,
            "task_id": 9,
            "status": "APPLIED",
            "ref": {"boot_id": 3, "msg_seq": 20},
            "uid": {"boot_id": 7, "msg_seq": 60},
        }]
        assert (
            "Task assign ack: owner=1 helper=2 task_id=9 status=APPLIED "
            "ref=3:20 uid=7:60"
        ) in [r.getMessage() for r in caplog.records]

    def test_applied_heard_before_its_copy_is_reported_with_the_copy(
        self, fleet, broadcast,
    ):
        # The copy and the APPLIED run on different vehicle-link threads.
        _applied(fleet, ref=(3, 20), uid=(7, 60))
        assert _payloads(broadcast, "task_assign_ack") == []

        _doing(fleet, task_id=9, uid=(3, 20))

        types = [call.args[0]["type"] for call in broadcast.call_args_list]
        assert types == ["task_assign_response", "task_assign_ack"]
        ack, = _payloads(broadcast, "task_assign_ack")
        assert (ack["task_id"], ack["ref"], ack["uid"]) == (
            9, {"boot_id": 3, "msg_seq": 20}, {"boot_id": 7, "msg_seq": 60},
        )

    @pytest.mark.parametrize("ack", [
        {"status": ACK_STATUS_RECEIVED},
        {"status": ACK_STATUS_PROCESSING},
        {"ref_type": MsgType.TASK_ASSIGN_REQUEST},
        {"ref": (3, 99)},
        {"owner": OTHER},
    ], ids=["received", "processing", "step-3-ack", "unknown-ref",
            "other-owner"])
    def test_other_acks_are_not_reported(self, fleet, broadcast, ack):
        _doing(fleet, task_id=9, uid=(3, 20))

        _applied(fleet, **{"ref": (3, 20), "uid": (7, 60), **ack})

        assert _payloads(broadcast, "task_assign_ack") == []

    def test_applied_of_a_rejected_copy_is_not_reported(self, fleet, broadcast):
        _doing(fleet, task_id=9, uid=(3, 20), accepted=False)

        _applied(fleet, ref=(3, 20), uid=(7, 60))

        assert _payloads(broadcast, "task_assign_ack") == []

    def test_applied_of_an_earlier_round_is_not_reported(
        self, fleet, broadcast,
    ):
        _doing(fleet, task_id=9, uid=(3, 20))
        _doing(fleet, task_id=4, uid=(3, 25), owner=OTHER)

        _applied(fleet, ref=(3, 20), uid=(7, 60))

        assert _payloads(broadcast, "task_assign_ack") == []

    def test_shared_bus_ack_is_reported_once(self, fleet, broadcast):
        _doing(fleet, task_id=9, uid=(3, 20))

        for link in (OWNER, HELPER, OTHER):
            _applied(fleet, ref=(3, 20), uid=(7, 60), link=link)

        assert len(_payloads(broadcast, "task_assign_ack")) == 1


