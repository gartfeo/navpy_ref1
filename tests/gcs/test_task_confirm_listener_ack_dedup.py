"""Tests for TaskConfirmListener's (sys_id, task_id) popup-dedup loop
(D-09/D-10/D-13, plan 02-04 Task 1) and the decided-round re-answer that
replaced the immediate dedup clear: once the operator has answered, a
repeated request means the response packet was lost, so it is re-answered
instead of re-opening the popup.
"""
from __future__ import annotations

import asyncio
import threading
from unittest.mock import MagicMock, patch, AsyncMock

import pytest

from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.task_confirm_listener import TaskConfirmListener, _LEGACY_ROUND_UID
from navpy.modules.comm.image_transfer import ImageTransferType
from navpy.modules.comm.messages.available_task_msg import TaskConfirmRequestMsg, TaskMsgData
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import TaskTypeMsgData

# The companion shares its aircraft's system id; component 191 is what
# tells it apart from the autopilot.
_COMPANION_SYS_ID = 1
_BOOT_ID = 424242


def _uid(msg_seq: int) -> str:
    """The wire form of a round uid, as the listener publishes it."""
    return f"{_BOOT_ID}:{msg_seq}"


@pytest.fixture
def loop():
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
    vehicle.send_mavlink_message = MagicMock()
    return vehicle


def _make_navlink_msg(task_id, lat=32.5, lon=34.8, alt=100.0, sys_id=_COMPANION_SYS_ID,
                      msg_seq=None):
    """Create a mock MAVLink message that looks like a TaskConfirmRequest.

    ``msg_seq`` is the round identity: the companion builds ONE request per
    confirm round and rebroadcasts that same object, so every retransmission
    of a round carries the same (boot_id, msg_seq) and a new round carries a
    fresh msg_seq. Left None, the message goes out with unset meta (all
    zeros), which is the legacy "no dedup metadata" shape.
    """
    meta = None if msg_seq is None else MsgMeta(
        boot_id=_BOOT_ID, msg_seq=msg_seq, time_ms=0, ttl_ms=5000,
    )
    req = TaskConfirmRequestMsg(
        sender_id=sys_id,
        task=TaskMsgData(
            task_id=task_id,
            task_type=TaskTypeMsgData.UNKNOWN,
            location=LocationMsgData(lat=lat, lng=lon, alt=alt),
        ),
        meta=meta,
    )
    mav_msg = req.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=sys_id)
    mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return mav_msg


class _HookedLock:
    """Wraps the listener's lock so a test can run code at the exact moment
    it is released -- the window a two-acquisition classify/transition would
    leave open between the decided check and the first-sighting write."""

    def __init__(self, on_release=None):
        self._real = threading.Lock()
        self.on_release = on_release
        self.acquisitions = 0

    def acquire(self, *args, **kwargs):
        self.acquisitions += 1
        return self._real.acquire(*args, **kwargs)

    def release(self):
        self._real.release()
        callback, self.on_release = self.on_release, None
        if callback is not None:
            callback()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc_info):
        self.release()
        return False


class TestRepeatRequestDedupPopup:
    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_n_repeats_broadcast_once(self, mock_ws, listener, mock_vehicle, loop):
        """D-10: of N repeated requests for the same (sys_id, task_id) only
        the first emits a task_confirm_request WS event -- a repeat never
        double-spawns the popup."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        for _ in range(4):
            listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_confirm_request"
        assert payload["task_id"] == 7

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_unregistered_companion_does_not_broadcast(self, mock_ws, listener, mock_vehicle, loop):
        """A request whose MAVLink source is NOT the registered companion for
        this sys_id gets no WS emit."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        forged = _make_navlink_msg(task_id=7, sys_id=999)  # not vehicle 1
        listener._on_navlink(1, forged)
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_response_sent_clears_dedup_for_fresh_round(self, mock_ws, listener, mock_vehicle, loop):
        """After clear_handled() (called by the /task_confirm route once a
        response is sent), a later fresh request for the same task_id can
        re-emit the WS event."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 1

        # Still within TTL and no clear -> deduped, no second broadcast.
        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 1

        listener.clear_handled(1, 7)

        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 2, "cleared dedup entry should allow a fresh popup"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_different_task_ids_each_get_their_own_first_broadcast(self, mock_ws, listener, mock_vehicle, loop):
        """Dedup is keyed by (sys_id, task_id) -- a different task_id is its
        own independent round and gets its own first-sighting broadcast."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        listener._on_navlink(1, _make_navlink_msg(task_id=8))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_ws.broadcast.call_count == 2
        task_ids = {c.args[0]["task_id"] for c in mock_ws.broadcast.call_args_list}
        assert task_ids == {7, 8}


class _FrozenClock:
    """Controllable stand-in for the listener module's ``time``."""

    def __init__(self, now: float = 1000.0):
        self.now = now

    def monotonic(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _sent_response(mock_vehicle, index: int = -1):
    """The MAVLink task-confirm response the listener re-sent."""
    return mock_vehicle.send_mavlink_message.call_args_list[index].args[0]


class TestDecidedRoundReanswer:
    """A decided round re-answers repeats instead of re-popping the card.

    The response is an unacknowledged packet sent x3 by the /task_confirm
    route; if all three copies are lost the companion keeps resending its
    request. Before this behavior existed that repeat re-opened the popup as
    if undecided (and pre-diet it was simply a dead round that expired
    silently).
    """

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_repeat_after_decision_resends_response_and_no_popup(
            self, mock_ws, listener, mock_vehicle, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 1

        listener.mark_decided(1, 7, is_confirmed=True, round_uid=_LEGACY_ROUND_UID)
        mock_vehicle.send_mavlink_message.reset_mock()

        # All 3 response copies were lost -> the companion asks again.
        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_vehicle.send_mavlink_message.call_count == 1, \
            "the repeat must be answered with the stored decision"
        sent = _sent_response(mock_vehicle)
        assert sent.task_id == 7
        assert sent.confirmed == 1
        assert sent.target_system == 1
        assert mock_ws.broadcast.call_count == 1, \
            "a decided round must never re-open the popup"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_every_repeat_gets_its_own_answer(self, mock_ws, listener, mock_vehicle, loop):
        """One answer per repeat: the listener runs on the reader thread and
        cannot sleep, so repetition comes from the companion's own cadence."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        listener.mark_decided(1, 7, is_confirmed=True, round_uid=_LEGACY_ROUND_UID)

        for _ in range(3):
            listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_vehicle.send_mavlink_message.call_count == 3
        mock_ws.broadcast.assert_not_called()

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_deny_decision_is_resent_as_deny(self, mock_ws, listener, mock_vehicle, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        listener.mark_decided(1, 7, is_confirmed=False, round_uid=_LEGACY_ROUND_UID)

        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert _sent_response(mock_vehicle).confirmed == 0

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_same_round_retransmission_is_re_answered(
            self, mock_ws, listener, mock_vehicle, loop):
        """A retransmission carries the round's own (boot_id, msg_seq), so it
        is recognised as the SAME round and gets the stored decision."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        listener.mark_decided(1, 7, is_confirmed=True, round_uid=_uid(11))

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_vehicle.send_mavlink_message.call_count == 1
        response = _sent_response(mock_vehicle)
        assert (response.boot_id, response.msg_seq) == (_BOOT_ID, 11)
        assert response.time_ms > 0
        assert response.ttl_ms > 0
        assert mock_ws.broadcast.call_count == 1, "same round must not re-pop"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_new_round_uid_opens_a_fresh_popup_immediately(
            self, mock_ws, listener, mock_vehicle, loop):
        """D-14: the companion can re-ask the moment its confirm window
        closes, which is well inside any decision-retention window. The
        re-ask is a NEW round (fresh msg_seq) and must open a fresh popup
        rather than silently collecting the previous round's answer -- no
        elapsed-time rule can tell those two cases apart.
        """
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        listener.mark_decided(1, 7, is_confirmed=True, round_uid=_uid(11))

        # Same target, same second -- but a new confirm round.
        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        assert mock_ws.broadcast.call_count == 2, \
            "a new round must get its own popup, not the old decision"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_new_round_after_an_unanswered_round_opens_a_fresh_popup(
            self, mock_ws, listener, mock_vehicle, loop):
        """Same rule on the undecided path: a re-ask that arrives while the
        previous, never-answered round is still inside the dedup TTL is a new
        round and must re-open the popup."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))  # retransmit
        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))  # re-ask
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_ws.broadcast.call_count == 2

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_decision_is_dropped_once_it_is_only_garbage(
            self, mock_ws, listener, mock_vehicle, loop):
        """The retention TTL now only bounds memory: an entry older than it
        is dropped, and the next request starts a clean round."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        clock = _FrozenClock()

        with patch("gcs.backend.task_confirm_listener.time", clock):
            listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
            loop.run_until_complete(asyncio.sleep(0.05))
            listener.mark_decided(1, 7, is_confirmed=True, round_uid=_uid(11))

            clock.advance(TaskConfirmListener._HANDLED_TTL_S + 1.0)
            listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
            loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        assert mock_ws.broadcast.call_count == 2

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_answer_binds_to_the_round_the_operator_saw_not_the_open_one(
            self, mock_ws, listener, mock_vehicle, loop):
        """The route spends ~0.5s sending its x3 response burst, so a new
        round B can arrive before the decision for round A is recorded.
        The answer belongs to the round the operator was shown, so it must
        be bound to A's uid -- binding it to whatever happens to be open
        would auto-answer B with a decision nobody made about B.
        """
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))   # round A
        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))   # round B
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 2

        # The operator's answer to A lands only now (the x3 burst took time).
        listener.mark_decided(1, 7, True, round_uid=_uid(11))

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))
        loop.run_until_complete(asyncio.sleep(0.05))
        mock_vehicle.send_mavlink_message.assert_not_called()

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_vehicle.send_mavlink_message.call_count == 1, \
            "the answered round is the one that gets re-answered"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_uncorrelated_response_does_not_answer_every_later_round(
            self, mock_ws, listener, mock_vehicle, loop):
        """A response that cannot be tied to a round (a duplicate POST, or a
        client that did not echo the uid) must record nothing. Recording it
        as "matches anything" would make the next, unrelated round collect
        that answer with no popup."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        listener.mark_decided(1, 7, True, round_uid=_uid(11))
        # Duplicate / uncorrelated response for the same target.
        listener.mark_decided(1, 7, True, round_uid=None)
        mock_vehicle.send_mavlink_message.reset_mock()

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        assert mock_ws.broadcast.call_count == 2, \
            "a new round must still get its own popup"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_popup_payload_carries_the_round_uid(
            self, mock_ws, listener, mock_vehicle, loop):
        """The uid has to reach the operator's card so it can be echoed back
        with the decision."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_ws.broadcast.call_args[0][0]["round_uid"] == _uid(11)

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_new_round_starts_with_clean_thumbnail_bookkeeping(
            self, mock_ws, listener, mock_vehicle, loop):
        """D-12 per-round state must not leak across a round rollover: round
        A's "image already received" flag would otherwise stop round B from
        ever pulling its own lost thumbnail."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        clock = _FrozenClock()

        with patch("gcs.backend.task_confirm_listener.time", clock):
            listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
            # Round A got its thumbnail, and had already asked for one.
            listener._rounds._image_received.add((1, 7))
            listener._rounds._thumbnail_requested.add((1, 7))

            listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))  # round B
            loop.run_until_complete(asyncio.sleep(0.05))
            assert (1, 7) not in listener._rounds._image_received, \
                "round A's image flag must not carry into round B"
            assert (1, 7) not in listener._rounds._thumbnail_requested

            # B's own thumbnail never arrives: past the grace window a repeat
            # must pull it on demand.
            clock.advance(TaskConfirmListener._THUMBNAIL_GRACE_S + 1.0)
            listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))
            loop.run_until_complete(asyncio.sleep(0.05))

        sent = mock_vehicle.send_mavlink_message.call_args_list
        assert len(sent) == 1, "round B must be able to request its thumbnail"
        request = sent[0].args[0]
        assert request.get_type() == "SWARM_REQUEST"
        assert (request.boot_id, request.msg_seq) == (_BOOT_ID, 12)
        assert request.time_ms > 0 and request.ttl_ms > 0

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_uncorrelated_response_does_not_mutate_the_live_round(
            self, mock_ws, listener, mock_vehicle, loop):
        """An uncorrelated response cannot dismiss UID-bearing round state."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 1

        listener.mark_decided(1, 7, True, round_uid=None)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        assert mock_ws.broadcast.call_count == 1
        assert listener._rounds.open_uid(1, 7) == _uid(11)

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_legacy_decision_never_answers_a_round_that_has_a_real_uid(
            self, mock_ws, listener, mock_vehicle, loop):
        """The legacy token stands for "this sender cannot identify its
        rounds", not "matches anything". After a companion upgrade (or a
        reconnect that starts sending metadata) a retained legacy decision
        must not auto-answer the first real round."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7))  # no metadata
        loop.run_until_complete(asyncio.sleep(0.05))
        listener.mark_decided(1, 7, True, round_uid=_LEGACY_ROUND_UID)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        assert mock_ws.broadcast.call_count == 2

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_legacy_answer_does_not_close_a_live_round_with_a_real_uid(
            self, mock_ws, listener, mock_vehicle, loop):
        """The mirror case: a late legacy-shaped answer must not dismiss the
        popup bookkeeping of a live, metadata-bearing round."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        listener.mark_decided(1, 7, True, round_uid=_LEGACY_ROUND_UID)

        # Round 11 is still open and unanswered -> a repeat is a duplicate,
        # not a fresh popup, and certainly not answered by the legacy decision.
        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        assert mock_ws.broadcast.call_count == 1

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_decided_check_and_first_sighting_are_one_locked_step(
            self, mock_ws, listener, mock_vehicle, loop):
        """The decided lookup and the first-sighting transition must be a
        SINGLE locked state change.

        If they are two acquisitions, a repeat can read "not decided", then
        the operator's answer lands in the gap and clears the round, and the
        repeat then records itself as a first sighting and pops a second card
        for a target that is already settled. Here the answer is injected at
        exactly that release point.
        """
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))
        assert mock_ws.broadcast.call_count == 1

        hooked = _HookedLock(on_release=lambda: listener.mark_decided(1, 7, True, _uid(11)))
        listener._rounds._lock = hooked

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        loop.run_until_complete(asyncio.sleep(0.05))

        assert mock_ws.broadcast.call_count == 1, \
            "a decision landing mid-classification must not produce a second popup"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_clear_handled_drops_the_decision_too(self, mock_ws, listener, mock_vehicle, loop):
        """clear_handled() is the hard reset: it wipes the recorded decision
        as well, so the next request is a plain first sighting."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        listener.mark_decided(1, 7, is_confirmed=True, round_uid=_LEGACY_ROUND_UID)

        listener.clear_handled(1, 7)
        listener._on_navlink(1, _make_navlink_msg(task_id=7))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        mock_ws.broadcast.assert_called_once()

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_decision_is_scoped_to_its_own_task_id(self, mock_ws, listener, mock_vehicle, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)
        listener.mark_decided(1, 7, is_confirmed=True, round_uid=_LEGACY_ROUND_UID)

        listener._on_navlink(1, _make_navlink_msg(task_id=8))
        loop.run_until_complete(asyncio.sleep(0.05))

        mock_vehicle.send_mavlink_message.assert_not_called()
        mock_ws.broadcast.assert_called_once()
        assert mock_ws.broadcast.call_args[0][0]["task_id"] == 8


def _handshake_msg(task_id, num_packets=1, size=4):
    """DATA_TRANSMISSION_HANDSHAKE opening a confirmation-image transfer."""
    msg = MagicMock()
    msg.type = ImageTransferType.TARGET_CONFIRMATION
    msg.width = task_id
    msg.size = size
    msg.packets = num_packets
    msg.payload = 253
    msg.jpg_quality = 80
    msg.get_srcSystem = MagicMock(return_value=_COMPANION_SYS_ID)
    msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return msg


def _chunk_msg(seqnr=0, data=b"abcd"):
    msg = MagicMock()
    msg.seqnr = seqnr
    msg.data = list(data)
    msg.get_srcSystem = MagicMock(return_value=_COMPANION_SYS_ID)
    msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return msg


class TestImageTransferIsRoundScoped:
    """A thumbnail belongs to the round that sent it (D-12).

    The transfer is chunked and can still be in flight when the companion
    gives up on that round and asks again. Delivering the late image to the
    new round would show the operator round A's picture AND make the listener
    believe round B already has its thumbnail, so B could never pull its own.
    """

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_image_from_a_finished_round_is_dropped(
            self, mock_ws, listener, mock_vehicle, loop):
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))   # round A
        listener._on_handshake(1, _handshake_msg(7))                        # A's image starts
        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=12))   # round B opens
        listener._on_chunk(1, _chunk_msg())                                 # A's image completes
        loop.run_until_complete(asyncio.sleep(0.05))

        broadcast_types = [c.args[0]["type"] for c in mock_ws.broadcast.call_args_list]
        assert "task_confirm_image" not in broadcast_types, \
            "round A's thumbnail must not be shown on round B's card"
        assert (1, 7) not in listener._rounds._image_received, \
            "a dropped image must not count as round B's thumbnail"

    @patch("gcs.backend.task_confirm_listener.ws_manager")
    def test_image_for_the_open_round_is_delivered_with_its_uid(
            self, mock_ws, listener, mock_vehicle, loop):
        """Control: the normal path still delivers, and carries the uid so the
        card can reject an image meant for a round it has moved past."""
        mock_ws.broadcast = AsyncMock()
        listener.register_vehicle(1, mock_vehicle)

        listener._on_navlink(1, _make_navlink_msg(task_id=7, msg_seq=11))
        listener._on_handshake(1, _handshake_msg(7))
        listener._on_chunk(1, _chunk_msg())
        loop.run_until_complete(asyncio.sleep(0.05))

        images = [c.args[0] for c in mock_ws.broadcast.call_args_list
                  if c.args[0]["type"] == "task_confirm_image"]
        assert len(images) == 1
        assert images[0]["task_id"] == 7
        assert images[0]["round_uid"] == _uid(11)
        assert (1, 7) in listener._rounds._image_received


if __name__ == "__main__":
    pytest.main([__file__])
