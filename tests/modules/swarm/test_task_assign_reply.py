"""Helper step-4 repeats: accept bound to the WAITING token, rejects by key."""

import itertools
import threading
from unittest.mock import Mock

import pytest

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
)
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.types import TaskTypeMsgData
from navpy.modules.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.modules.swarm.task_actor_slots import SelectedTaskSlot, SlotState
from navpy.modules.swarm.task_assign_reply import AssignReplies
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_msg_refs import MsgRef
from tests.modules.swarm.test_task_assign_confirmation import FakeTimers

SCHEDULE = ASSIGN_ACK_TIMING.response
OWNER = 1


class Harness:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.slot = SelectedTaskSlot(self.lock)
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
            self.lock, self.slot, self.sender, SCHEDULE, self.logger,
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
    fake = FakeTimers()
    with fake.patch():
        yield fake


def test_accept_repeats_on_schedule_then_expires_with_an_error(timers):
    harness = Harness()
    held = harness.offer()

    harness.replies.start_accept(held)
    assert len(harness.answers()) == 1  # the first copy goes out at once
    intervals = []
    while (pending := [t for t in timers.created if t.alive]):
        intervals.append(pending[0].interval)
        timers.fire_pending()

    assert len(harness.answers()) == SCHEDULE.copies
    assert intervals == [
        SCHEDULE.delay_after(sent) for sent in range(1, SCHEDULE.copies + 1)
    ]
    assert harness.slot.state() is SlotState.EMPTY
    harness.logger.error.assert_called_once()
    # Every copy is one the owner's APPLIED may name.
    assert len({ref for *_, ref in harness.answers()}) == SCHEDULE.copies


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

        assert harness.replies.on_reject_ack(copy[3], ACK_STATUS_PROCESSING)
        timers.fire_pending()
        assert len(harness.answers(False)) == 2  # PROCESSING: keep repeating

        assert harness.replies.on_reject_ack(harness.answers(False)[-1][3], stop_status)
        timers.fire_pending()
        assert len(harness.answers(False)) == 2


def test_reject_repeat_ends_after_its_copies(timers):
    harness = Harness()
    harness.replies.start_reject(OWNER, 11)

    while any(t.alive for t in timers.created):
        timers.fire_pending()

    assert len(harness.answers(False)) == SCHEDULE.copies
    assert not harness.replies.on_reject_ack(
        harness.answers(False)[-1][3], ACK_STATUS_RECEIVED,
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
