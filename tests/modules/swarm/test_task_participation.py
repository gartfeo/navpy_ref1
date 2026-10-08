"""Helper gates: busy UAVs do not bid, unflyable tasks are declined, and a
step 3 is acked before any step-4 copy answers it."""

import math
import threading
from unittest.mock import Mock

import pytest

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    AvailableTaskResponseMsg,
    TaskAssignMsgData,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.modules.swarm.task_actor_slots import SelectedTaskSlot, SlotState
from navpy.modules.swarm.task_assign_reply import AssignReplies
from navpy.modules.swarm.task_auction_models import DECLINED_ETA_MIN
from navpy.modules.swarm.task_capability import (
    TaskCapabilityEvaluator,
    TaskParticipationCoordinator,
)
from navpy.modules.swarm.task_messaging import TaskMessageSender
from tests.modules.swarm.test_task_assign_confirmation import FakeTimers

OWNER, OWNER_BOOT = 1, 7


def _location(lat: float) -> LocationMsgData:
    return LocationMsgData(lat, 2.0, 3.0)


def _meta(seq: int, boot_id: int = OWNER_BOOT) -> MsgMeta:
    return MsgMeta(boot_id=boot_id, msg_seq=seq, time_ms=0, ttl_ms=5000)


class Helper:
    """UAV 2 answering owner 1; ETA per task latitude via ``etas``."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.slot = SelectedTaskSlot(self.lock)
        self.network = Mock(spec=NetworkAbc)
        self.logger = Mock(spec=ILogger)
        sender = TaskMessageSender(2, self.network, self.logger)
        vehicle = Mock()
        vehicle.location.return_value = Location(1.0, 2.0, 3.0)
        self.etas = {}
        self.coordinator = TaskParticipationCoordinator(
            2,
            self.lock,
            self.slot,
            AssignReplies(
                self.lock, self.slot, sender, ASSIGN_ACK_TIMING.response,
                self.logger,
            ),
            TaskCapabilityEvaluator(
                vehicle, lambda location: self.etas.get(location.lat, 4.0),
            ),
            sender,
            self.logger,
        )

    def sent(self, message_type=None):
        return [
            call.args[0]
            for call in self.network.broadcast.call_args_list
            if message_type is None or isinstance(call.args[0], message_type)
        ]

    def advert(self, *lats, seq=40):
        self.coordinator.on_available_request(AvailableTaskRequestMsg(
            sender_id=OWNER,
            tasks=[
                TaskMsgData(
                    task_id=index,
                    task_type=TaskTypeMsgData.DOCK,
                    location=_location(lat),
                )
                for index, lat in enumerate(lats, start=10)
            ],
            meta=_meta(seq),
        ))

    def request(self, task_id=10, lat=1.5, seq=50, meta=True):
        self.coordinator.on_assign_request(TaskAssignRequestMsg(
            sender_id=OWNER,
            receiver_id=2,
            task=TaskAssignMsgData(
                task_id=task_id,
                task_type=TaskTypeMsgData.DOCK,
                location=_location(lat),
            ),
            meta=_meta(seq) if meta else None,
        ))


@pytest.fixture
def timers():
    fake = FakeTimers()
    with fake.patch():
        yield fake


def test_free_helper_bids_and_declines_tasks_it_cannot_fly(timers):
    helper = Helper()
    helper.etas = {1.5: 4.0, 1.6: -1, 1.7: None, 1.8: math.nan}

    helper.advert(1.5, 1.6, 1.7, 1.8)

    (bid,) = helper.sent(AvailableTaskResponseMsg)
    assert bid.receiver_id == OWNER
    assert [(task.task_id, task.time_in_min) for task in bid.tasks] == [
        (10, 4.0),
        (11, DECLINED_ETA_MIN),
        (12, DECLINED_ETA_MIN),
        (13, DECLINED_ETA_MIN),
    ]


@pytest.mark.parametrize("busy", ["waiting", "approaching"])
def test_busy_helper_does_not_bid(timers, busy):
    helper = Helper()
    if busy == "waiting":
        helper.request(task_id=20)
    else:
        helper.coordinator.set_approaching(True)
    helper.network.reset_mock()

    helper.advert(1.5)

    assert helper.sent(AvailableTaskResponseMsg) == []


def test_step_three_is_acked_before_the_first_answer(timers):
    helper = Helper()

    helper.request()

    ack, answer = helper.sent()
    assert isinstance(ack, SwarmAckMsg) and isinstance(answer, TaskAssignResponseMsg)
    assert (ack.receiver_id, ack.ref_msg_seq, ack.status) == (
        OWNER, 50, ACK_STATUS_RECEIVED,
    )
    assert ack.ref_msg_type == MsgType.TASK_ASSIGN_REQUEST.value
    assert answer.is_accepted
    assert answer.meta.msg_seq > ack.meta.msg_seq
    assert helper.slot.state() is SlotState.WAITING


@pytest.mark.parametrize("reason", ["approaching", "unflyable"])
def test_step_three_while_approaching_or_unflyable_is_acked_and_rejected(
    timers, reason,
):
    helper = Helper()
    if reason == "approaching":
        helper.coordinator.set_approaching(True)
    else:
        helper.etas = {1.5: -1}

    helper.request()
    timers.fire_due_now()

    ack, answer = helper.sent()
    assert isinstance(ack, SwarmAckMsg)
    assert isinstance(answer, TaskAssignResponseMsg) and not answer.is_accepted
    assert helper.slot.state() is SlotState.EMPTY


def test_step_three_without_uid_gets_one_reject(timers):
    helper = Helper()

    helper.request(meta=False)
    timers.fire_pending()

    (answer,) = helper.sent()
    assert isinstance(answer, TaskAssignResponseMsg) and not answer.is_accepted
    assert helper.slot.state() is SlotState.EMPTY


def test_owner_applied_ack_assigns_and_is_logged(timers):
    helper = Helper()
    helper.request()
    answer = helper.sent(TaskAssignResponseMsg)[0]

    helper.coordinator.on_response_ack(SwarmAckMsg(
        sender_id=OWNER,
        receiver_id=2,
        ref_boot_id=answer.meta.boot_id,
        ref_msg_seq=answer.meta.msg_seq,
        ref_msg_type=MsgType.TASK_ASSIGN_RESPONSE.value,
        status=ACK_STATUS_APPLIED,
        meta=_meta(60),
    ))

    assert helper.slot.selected().task_id == 10
    helper.logger.info.assert_any_call("Task 10 assigned by owner 1")


def test_own_final_approach_drops_waiting_and_rejects_it(timers):
    helper = Helper()
    helper.request()
    helper.network.reset_mock()

    helper.coordinator.set_approaching(True)
    assert helper.sent() == []  # nothing sent on the caller (nav) thread
    timers.fire_due_now()

    (answer,) = helper.sent()
    assert (answer.receiver_id, answer.task_id, answer.is_accepted) == (
        OWNER, 10, False,
    )
    assert helper.slot.state() is SlotState.EMPTY
