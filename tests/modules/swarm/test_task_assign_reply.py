"""Helper step-4 repeats: accept bound to the WAITING token, rejects by key."""

import itertools
import threading
from unittest.mock import Mock

import pytest

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
)
from navpy.core.link.task_message_data import TaskAssignMsgData
from navpy.core.link.types import MsgType, TaskTypeMsgData
from navpy.core.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.core.swarm.task_actor_slots import SelectedTaskSlot, SlotState
from navpy.core.swarm.task_assign_reply import AssignReplies
from navpy.core.swarm.task_messaging import TaskMessageSender
from navpy.core.swarm.task_msg_refs import MsgRef
from tests.modules.swarm.test_task_assign_confirmation import FakeTimers

TIMING = ASSIGN_ACK_TIMING.response
OWNER = 1
# Copies sent before a silence ends a repeat: at 0, then every interval
# while less than the silence has passed.
COPIES_IN_SILENCE = round(TIMING.silence_s / TIMING.interval_s)


class Harness:
    clock = None  # the test's FakeTimers, set by the timers fixture
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.clock = Harness.clock
        self.slot = SelectedTaskSlot(self.lock, self.clock)
        self.sender = Mock(spec=TaskMessageSender)
        seqs = itertools.count(1)
        self.sent: list[tuple[int, int, bool, MsgRef]] = []

        def answer(owner_id, task_id, accepted):
            ref = MsgRef(2, 9, next(seqs))
            self.sent.append((owner_id, task_id, accepted, ref))
            return ref

        self.sender.assignment_response.side_effect = answer
        self.logger = Mock(spec=ILogger)
        self.replies = AssignReplies(
            self.lock, self.slot, self.sender, TIMING, self.logger, self.clock,
        )

    def offer(self, task_id=10):
        task = TaskAssignMsgData(
            task_id=task_id,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(1.0, 2.0, 3.0),
        )
        return self.slot.on_request(task, MsgRef(OWNER, 7, 50), True).held

    def answers(self, accepted=True):
        return [entry for entry in self.sent if entry[2] is accepted]


@pytest.fixture
def timers():
    fake = Harness.clock = FakeTimers()
    with fake.patch():
        yield fake


def _drain(timers) -> list[float]:
    intervals = []
    while (pending := [t for t in timers.created if t.alive]):
        intervals.append(pending[0].interval)
        timers.fire_pending()
    return intervals


def test_accept_repeats_every_interval_then_expires_after_silence(timers):
    harness = Harness()
    held = harness.offer()

    harness.replies.start_accept(held)
    assert len(harness.answers()) == 1  # the first copy goes out at once
    harness.sender.ack_ref.assert_not_called()  # it follows the step-3 ack
    intervals = _drain(timers)

    assert len(harness.answers()) == COPIES_IN_SILENCE
    assert set(intervals) == {TIMING.interval_s}
    assert timers.now - 1000.0 == TIMING.silence_s
    assert harness.slot.state() is SlotState.EMPTY
    harness.logger.error.assert_called_once()
    # Every copy is one the owner's APPLIED may name.
    assert len({ref for *_, ref in harness.answers()}) == COPIES_IN_SILENCE


def test_each_scheduled_accept_copy_follows_a_re_ack_of_the_held_request(timers):
    harness = Harness()
    order = []
    harness.sender.ack_ref.side_effect = lambda *args: order.append(("ack", args))
    answer = harness.sender.assignment_response.side_effect
    harness.sender.assignment_response.side_effect = (
        lambda *args: order.append(("step4", args)) or answer(*args)
    )
    held = harness.offer()
    harness.replies.start_accept(held)

    timers.fire_pending()
    timers.fire_pending()

    assert [kind for kind, _ in order] == ["step4", "ack", "step4", "ack", "step4"]
    assert order[1][1] == (held.request, MsgType.TASK_ASSIGN_REQUEST, ACK_STATUS_RECEIVED)


def test_owner_processing_ack_restarts_the_accept_silence(timers):
    harness = Harness()
    held = harness.offer()
    harness.replies.start_accept(held)
    for _ in range(COPIES_IN_SILENCE - 2):
        timers.fire_pending()
    kind, _ = harness.slot.on_ack(
        harness.answers()[-1][3], ACK_STATUS_PROCESSING, MsgRef(OWNER, 7, 60),
    )
    harness.replies.accept_progress(held.token)
    progress_s = timers.now

    _drain(timers)

    assert timers.now - progress_s == TIMING.silence_s
    assert harness.slot.state() is SlotState.EMPTY


def test_progress_for_an_older_wait_does_not_extend_the_new_one(timers):
    harness = Harness()
    held = harness.offer()
    harness.replies.start_accept(held)
    timers.fire_pending()

    harness.replies.accept_progress(held.token + 1)
    _drain(timers)

    assert timers.now - 1000.0 == TIMING.silence_s


def test_accept_repeat_stops_once_assigned(timers):
    harness = Harness()
    held = harness.offer()
    harness.replies.start_accept(held)
    first = harness.answers()[0][3]

    harness.slot.on_ack(first, ACK_STATUS_APPLIED, MsgRef(OWNER, 7, 60))
    timers.fire_pending()

    assert len(harness.answers()) == 1
    assert harness.slot.state() is SlotState.ASSIGNED


def test_repeated_offer_gets_one_extra_copy_recorded_for_the_wait(timers):
    harness = Harness()
    held = harness.offer()
    harness.replies.start_accept(held)

    harness.replies.answer_again(held)

    extra = harness.answers()[-1][3]
    assert len(harness.answers()) == 2
    assert harness.slot.on_ack(extra, ACK_STATUS_APPLIED, MsgRef(OWNER, 7, 60))[1] == held


def test_reject_repeat_is_deferred_then_stops_on_received_or_applied(timers):
    for stop_status in (ACK_STATUS_RECEIVED, ACK_STATUS_APPLIED):
        harness = Harness()
        harness.replies.start_reject(OWNER, 11)
        assert harness.answers(False) == []  # sent from the timer thread

        timers.fire_due_now()
        (copy,) = harness.answers(False)
        assert copy[:3] == (OWNER, 11, False)

        assert harness.replies.on_reject_ack(copy[3], ACK_STATUS_PROCESSING, OWNER)
        timers.fire_pending()
        assert len(harness.answers(False)) == 2  # PROCESSING: keep repeating

        assert harness.replies.on_reject_ack(
            harness.answers(False)[-1][3], stop_status, OWNER,
        )
        timers.fire_pending()
        assert len(harness.answers(False)) == 2


def test_reject_repeat_ends_after_silence_and_forgets_its_key(timers):
    harness = Harness()
    harness.replies.start_reject(OWNER, 11)

    _drain(timers)

    assert len(harness.answers(False)) == COPIES_IN_SILENCE
    assert timers.now - 1000.0 == TIMING.silence_s
    assert not harness.replies.on_reject_ack(
        harness.answers(False)[-1][3], ACK_STATUS_RECEIVED, OWNER,
    )
    harness.replies.start_reject(OWNER, 11)  # a new repeat, deferred again
    assert len(harness.answers(False)) == COPIES_IN_SILENCE


def test_reject_processing_restarts_its_silence_only_from_its_owner(timers):
    for sender_id, extended in ((OWNER, True), (OWNER + 5, False)):
        harness = Harness()
        timers.now = 1000.0
        harness.replies.start_reject(OWNER, 11)
        timers.fire_due_now()
        for _ in range(COPIES_IN_SILENCE - 2):
            timers.fire_pending()
        assert harness.replies.on_reject_ack(
            harness.answers(False)[-1][3], ACK_STATUS_PROCESSING, sender_id,
        )
        progress_s = timers.now

        _drain(timers)

        ended_after = timers.now - (progress_s if extended else 1000.0)
        assert ended_after == TIMING.silence_s


def test_reject_copies_are_forgotten_after_their_retention(timers):
    harness = Harness()
    harness.replies.start_reject(OWNER, 11)
    timers.fire_due_now()
    first = harness.answers(False)[0][3]
    harness.replies.on_reject_ack(  # keep the repeat alive past retention
        first, ACK_STATUS_PROCESSING, OWNER,
    )
    while timers.now - 1000.0 < TIMING.retention_s:
        timers.fire_pending()
        harness.replies.on_reject_ack(
            harness.answers(False)[-1][3], ACK_STATUS_PROCESSING, OWNER,
        )

    assert not harness.replies.on_reject_ack(first, ACK_STATUS_RECEIVED, OWNER)
    assert harness.replies.on_reject_ack(
        harness.answers(False)[-1][3], ACK_STATUS_RECEIVED, OWNER,
    )


def test_repeated_rejected_offer_gets_one_copy_now(timers):
    harness = Harness()
    harness.replies.start_reject(OWNER, 11)
    timers.fire_due_now()

    harness.replies.start_reject(OWNER, 11)

    assert len(harness.answers(False)) == 2


def test_accepting_an_offer_cancels_its_reject_repeat(timers):
    harness = Harness()
    harness.replies.start_reject(OWNER, 10)
    held = harness.offer(10)

    harness.replies.start_accept(held)
    timers.fire_due_now()

    assert harness.answers(False) == []


def test_close_cancels_every_repeat_for_good(timers):
    harness = Harness()
    held = harness.offer()
    harness.replies.start_accept(held)
    harness.replies.start_reject(3, 12)

    harness.replies.close()
    timers.fire_pending()
    harness.replies.start_reject(3, 13)
    harness.replies.answer_again(held)
    timers.fire_pending()

    assert len(harness.sent) == 1
