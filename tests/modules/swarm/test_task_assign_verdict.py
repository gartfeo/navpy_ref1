"""Owner side of the handshake: fence floor, verdict table, release.

See docs/design/swarm-task-assignment-ack.md, "Owner verdict".
"""

import threading
from unittest.mock import Mock

import pytest

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.task_assignment_msg import TaskAssignResponseMsg
from navpy.core.link.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.core.link.types import (
    MsgType,
    TaskDispatchStatus,
    TaskTypeMsgData,
)
from navpy.core.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.core.swarm.task_assignment_planner import (
    MinimumEtaAssignmentPlanner,
)
from navpy.core.swarm.task_auction_models import (
    AssignConfirmationPorts,
    ResponseVerdict,
)
from navpy.core.swarm.task_msg_refs import MsgRef
from navpy.core.swarm.task_state_composition import create_task_state
from tests.modules.swarm.test_task_assign_confirmation import FakeTimers

TIMING = ASSIGN_ACK_TIMING.request
HELPER_BOOT = 70
REQUEST = MsgRef(1, 5, 100)


def _task(task_id: int = 7) -> TaskMsgData:
    return TaskMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(1.0, 2.0, 3.0),
    )


def _meta(seq: int, boot_id: int = HELPER_BOOT) -> MsgMeta:
    return MsgMeta(boot_id=boot_id, msg_seq=seq, time_ms=0, ttl_ms=5000)


def _request_ack(peer_id=2, request=REQUEST, seq=10):
    return SwarmAckMsg(
        sender_id=peer_id,
        receiver_id=1,
        ref_boot_id=request.boot_id,
        ref_msg_seq=request.msg_seq,
        ref_msg_type=MsgType.TASK_ASSIGN_REQUEST.value,
        status=ACK_STATUS_RECEIVED,
        meta=_meta(seq),
    )


def _response(accepted, peer_id=2, seq=11, task_id=7, meta=True, boot_id=HELPER_BOOT):
    return TaskAssignResponseMsg(
        sender_id=peer_id,
        receiver_id=1,
        task_id=task_id,
        is_accepted=accepted,
        meta=_meta(seq, boot_id) if meta else None,
    )


class Owner:
    """Owner state with peers 2 and 3 and task 7 reserved to peer 2."""

    clock = None  # the test's FakeTimers, set by the timers fixture

    def __init__(self) -> None:
        _, self.auction, rebroadcast, self.confirmation = create_task_state(
            threading.RLock(), Owner.clock,
        )
        for peer_id in (2, 3):
            rebroadcast.discover_peer(peer_id, lambda *_: None)
        self.store = self.auction._store
        self.dispatch, generation = self.auction.register(_task())
        self.dispatch.on_peer_available(2, TaskHandleMsgData(7, 1.0))
        self.dispatch.on_peer_available(3, TaskHandleMsgData(7, 2.0))
        (self.reservation,) = self.auction.plan_and_reserve(
            MinimumEtaAssignmentPlanner(), generation,
        )
        self.confirmation.arm(self.reservation, REQUEST, Mock(), TIMING)

    def acked(self) -> "Owner":
        assert self.confirmation.on_request_ack(_request_ack()) == 7
        return self

    def confirmed(self) -> "Owner":
        self.acked()
        assert self.verdict(_response(True)) is ResponseVerdict.CONFIRMED
        return self

    def verdict(self, message):
        return self.confirmation.answer_response(message).verdict


@pytest.fixture(autouse=True)
def timers():
    fake = Owner.clock = FakeTimers()
    with fake.patch():
        yield fake


def test_first_request_ack_from_the_reserved_helper_sets_the_floor():
    owner = Owner()

    assert owner.confirmation.on_request_ack(_request_ack(seq=10)) == 7
    assert owner.confirmation.on_request_ack(_request_ack(seq=12)) is None

    assert owner.dispatch.attempt.floor == MsgRef(2, HELPER_BOOT, 10)


@pytest.mark.parametrize(
    "ack",
    [_request_ack(peer_id=3), _request_ack(request=MsgRef(1, 5, 999))],
    ids=["other-peer", "unknown-copy"],
)
def test_other_acks_set_no_floor(ack):
    owner = Owner()

    assert owner.confirmation.on_request_ack(ack) is None
    assert owner.dispatch.attempt.floor is None


@pytest.mark.parametrize(
    "message",
    [
        _response(True, task_id=8),
        _response(True, peer_id=3),
        _response(False, peer_id=3),
    ],
    ids=["missing-task", "other-peer-accepts", "other-peer-rejects"],
)
def test_not_reserved_to_sender_is_received_not_yours(message):
    owner = Owner().acked()

    verdict = owner.confirmation.answer_response(message)

    assert (verdict.verdict, verdict.ack_status) == (
        ResponseVerdict.NOT_YOURS, ACK_STATUS_RECEIVED,
    )
    assert owner.dispatch.status is TaskDispatchStatus.CONFIRMING


def test_available_task_is_not_yours():
    owner = Owner()
    owner.dispatch.peer_reject(2)

    assert owner.verdict(_response(True)) is ResponseVerdict.NOT_YOURS


@pytest.mark.parametrize(
    "message, acked",
    [
        (_response(True), False),
        (_response(True, seq=10), True),
        (_response(True, seq=9), True),
        (_response(True, boot_id=HELPER_BOOT + 1, seq=99), True),
        (_response(True, meta=False), True),
        (_response(False, seq=9), True),
    ],
    ids=[
        "no-floor", "same-seq", "before-floor", "other-boot", "no-uid",
        "reject-before-floor",
    ],
)
def test_unfenced_response_is_processing_and_changes_nothing(message, acked):
    owner = Owner()
    if acked:
        owner.acked()

    verdict = owner.confirmation.answer_response(message)

    assert (verdict.verdict, verdict.ack_status) == (
        ResponseVerdict.UNFENCED, ACK_STATUS_PROCESSING,
    )
    assert owner.dispatch.status is TaskDispatchStatus.CONFIRMING


def test_fenced_accept_confirms_and_is_applied():
    owner = Owner().acked()

    verdict = owner.confirmation.answer_response(_response(True))

    assert (verdict.verdict, verdict.ack_status) == (
        ResponseVerdict.CONFIRMED, ACK_STATUS_APPLIED,
    )
    assert owner.dispatch.status is TaskDispatchStatus.CONFIRMED
    assert owner.dispatch.assigned_peer == 2


def test_accept_copy_of_a_confirmed_task_is_applied_again():
    owner = Owner().confirmed()

    verdict = owner.confirmation.answer_response(_response(True, seq=14))

    assert (verdict.verdict, verdict.ack_status) == (
        ResponseVerdict.REPEAT, ACK_STATUS_APPLIED,
    )


def test_unfenced_copy_of_a_confirmed_task_is_processing():
    owner = Owner().confirmed()

    assert owner.verdict(_response(True, seq=9)) is ResponseVerdict.UNFENCED


def test_fenced_reject_of_a_confirming_task_is_a_retry():
    owner = Owner().acked()

    verdict = owner.confirmation.answer_response(_response(False))

    assert (verdict.verdict, verdict.ack_status) == (
        ResponseVerdict.REJECTED, ACK_STATUS_RECEIVED,
    )
    assert verdict.reject.kind == "retry"
    assert owner.dispatch.status is TaskDispatchStatus.AVAILABLE
    assert set(owner.dispatch.task_handle_by_peer) == {3}


def test_reject_of_a_confirmed_task_is_never_taken_back():
    owner = Owner().confirmed()

    verdict = owner.confirmation.answer_response(_response(False, seq=14))

    assert (verdict.verdict, verdict.ack_status) == (
        ResponseVerdict.KEPT, ACK_STATUS_RECEIVED,
    )
    assert owner.dispatch.status is TaskDispatchStatus.CONFIRMED
    assert owner.dispatch.assigned_peer == 2


def test_every_accepted_copy_reports_its_sender_busy():
    owner = Owner()

    reject = owner.confirmation.answer_response(_response(False, peer_id=3))
    accept = owner.confirmation.answer_response(_response(True, peer_id=3))

    assert not reject.busy_changed
    assert accept.verdict is ResponseVerdict.NOT_YOURS
    assert accept.busy_changed
    assert 3 in owner.store.peers.busy()


def test_timer_of_an_older_attempt_is_stale():
    owner = Owner()
    owner.dispatch.begin_attempt()
    send = Mock()

    outcome = owner.confirmation.due(
        owner.reservation, AssignConfirmationPorts(send, Mock(), Mock()), TIMING,
    )

    assert outcome.kind == "stale"
    send.assert_not_called()


def _ports(seq=101):
    return AssignConfirmationPorts(
        Mock(return_value=MsgRef(1, 5, seq)), Mock(), Mock(),
    )


def _due(owner, ports):
    return owner.confirmation.due(owner.reservation, ports, TIMING).kind


def test_request_copies_go_on_after_the_floor_until_confirmed():
    owner = Owner()
    ports = _ports()

    first = _due(owner, ports)
    owner.acked()
    second = _due(owner, ports)
    assert owner.verdict(_response(True)) is ResponseVerdict.CONFIRMED
    after = _due(owner, ports)

    assert [first, second, after] == ["resent", "acked", "stale"]
    assert ports.send_request.call_count == 2
    assert REQUEST in owner.dispatch.attempt.request_refs
    assert MsgRef(1, 5, 101) in owner.dispatch.attempt.request_refs


def test_silence_without_a_floor_releases_and_re_advertises(timers):
    owner = Owner()
    ports = _ports()
    timers.now += TIMING.silence_s - TIMING.interval_s
    assert _due(owner, ports) == "resent"

    timers.now += TIMING.interval_s
    assert _due(owner, ports) == "released"

    assert owner.dispatch.status is TaskDispatchStatus.AVAILABLE
    assert owner.dispatch.answers.released_to == 2
    assert set(owner.dispatch.task_handle_by_peer) == {3}
    ports.advertise.assert_called_once_with([owner.dispatch.task])


def test_the_floor_restarts_the_silence(timers):
    owner = Owner()
    ports = _ports()
    timers.now += TIMING.silence_s - TIMING.interval_s
    owner.acked()

    timers.now += TIMING.silence_s - TIMING.interval_s
    assert _due(owner, ports) == "acked"
    timers.now += TIMING.interval_s
    assert _due(owner, ports) == "released"


def test_acks_after_the_floor_are_not_progress(timers):
    owner = Owner().acked()
    ports = _ports()
    timers.now += TIMING.silence_s - TIMING.interval_s
    assert owner.confirmation.on_request_ack(_request_ack(seq=12)) is None

    timers.now += TIMING.interval_s
    assert _due(owner, ports) == "released"


@pytest.mark.parametrize(
    ("message", "progress"),
    [
        (_response(True, seq=9), True),  # unfenced, but from the floor's boot
        (_response(True, seq=12, boot_id=HELPER_BOOT + 1), False),
        (_response(True, peer_id=3, seq=12), False),
    ],
    ids=["floor-boot", "other-boot", "other-peer"],
)
def test_only_a_response_from_the_floor_boot_is_progress(timers, message, progress):
    owner = Owner().acked()
    ports = _ports()
    timers.now += TIMING.silence_s - TIMING.interval_s
    owner.confirmation.answer_response(message)

    timers.now += TIMING.interval_s
    assert _due(owner, ports) == ("acked" if progress else "released")


def test_no_response_before_the_floor_is_progress(timers):
    owner = Owner()
    ports = _ports()
    timers.now += TIMING.silence_s - TIMING.interval_s
    owner.confirmation.answer_response(_response(True, seq=9))

    timers.now += TIMING.interval_s
    assert _due(owner, ports) == "released"


def test_an_ack_naming_a_pruned_copy_sets_no_floor(timers):
    owner = Owner()
    timers.now += TIMING.retention_s

    assert owner.confirmation.on_request_ack(_request_ack()) is None
    assert owner.dispatch.attempt.floor is None


@pytest.mark.parametrize("close", ["reset", "shutdown"])
def test_reset_and_shutdown_log_dropped_handed_out_tasks(close):
    owner = Owner().confirmed()
    owner.auction.register(_task(8))
    logger = Mock(spec=ILogger)

    getattr(owner.auction, close)(logger)

    (call,) = logger.warning.call_args_list
    assert "task 7 CONFIRMED to peer 2" in call.args[0]
