"""Lost TASK_ASSIGN_REQUEST / TASK_ASSIGN_RESPONSE must not strand a task.

Transport is best effort: the owner resends an unacked assign request and
finally releases the CONFIRMING reservation; only a response fenced by the
helper's request ack confirms it.
"""

import threading
from unittest.mock import Mock, patch

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    AvailableTaskResponseMsg,
    TaskAssignMsgData,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.modules.comm.messages.types import (
    MsgType,
    TaskDispatchStatus,
    TaskTypeMsgData,
)
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_state_composition import create_task_state
from navpy.modules.swarm.task_dispatch import TaskDispatch
from navpy.modules.vehicle.vehicle_interface import IVehicle


class FakeTimer:
    def __init__(self, interval, function, args=None):
        self.interval = interval
        self.function = function
        self.args = args or ()
        self.alive = False

    def start(self):
        self.alive = True

    def cancel(self):
        self.alive = False

    def is_alive(self):
        return self.alive

    def fire(self):
        self.alive = False
        self.function(*self.args)


class FakeTimers:
    """Deterministic stand-in for TaskDispatch's threading.Timer."""

    def __init__(self):
        self.created: list[FakeTimer] = []

    def make(self, *args, **kwargs):
        timer = FakeTimer(*args, **kwargs)
        self.created.append(timer)
        return timer

    def patch(self):
        return patch(
            "navpy.modules.swarm.task_dispatch.threading.Timer",
            side_effect=self.make,
        )

    def fire_pending(self) -> int:
        pending = [timer for timer in self.created if timer.alive]
        for timer in pending:
            if timer.alive:
                timer.fire()
        return len(pending)

    def fire_due_now(self) -> int:
        """Fire only zero-delay timers (deferred first copies)."""
        due = [t for t in self.created if t.alive and t.interval == 0.0]
        for timer in due:
            if timer.alive:
                timer.fire()
        return len(due)


def _task(task_id: int) -> TaskMsgData:
    return TaskMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(12.34, 56.78, 90.0),
    )


def _assign_task(task_id: int) -> TaskAssignMsgData:
    return TaskAssignMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(12.34, 56.78, 90.0),
    )


def _meta(msg_seq: int, boot_id: int = 70) -> MsgMeta:
    return MsgMeta(boot_id=boot_id, msg_seq=msg_seq, time_ms=0, ttl_ms=5000)


def _ack(sender_id, acked, status, msg_seq, boot_id=70) -> SwarmAckMsg:
    """``sender_id`` acks ``acked``, a message it received."""
    return SwarmAckMsg(
        sender_id=sender_id,
        receiver_id=acked.sender_id,
        ref_boot_id=acked.meta.boot_id,
        ref_msg_seq=acked.meta.msg_seq,
        ref_msg_type=acked.msg_type().value,
        status=status,
        meta=_meta(msg_seq, boot_id),
    )


def _sent(network: Mock, message_type: type) -> list:
    return [
        call.args[0]
        for call in network.broadcast.call_args_list
        if isinstance(call.args[0], message_type)
    ]


class TestOwnerConfirmation:
    def setup_method(self):
        vehicle = Mock(spec=IVehicle)
        vehicle.source_system = 1
        vehicle.location.return_value = Location(12.34, 56.78, 90.0)
        self.network = Mock(spec=NetworkAbc)
        self.actor = TaskActor(vehicle, self.network, Mock(spec=ILogger))
        self.actor.start()
        self.timers = FakeTimers()

    def teardown_method(self):
        self.actor.reset()

    def _reserve_task_10_for_peer_2(self) -> TaskDispatch:
        for peer_id in (2, 3):
            self.actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        dispatch = TaskDispatch(_task(10))
        dispatch.on_peer_available(2, TaskHandleMsgData(10, 1.0))
        dispatch.on_peer_available(3, TaskHandleMsgData(10, 2.0))
        assert self.actor._auction_state.register_dispatch(dispatch) is not None
        self.network.reset_mock()
        self.actor._auction._select_peer_for_task(
            10, self.actor._auction_state.current_generation()
        )
        assert dispatch.status is TaskDispatchStatus.CONFIRMING
        assert dispatch.assigned_peer == 2
        return dispatch

    def test_lost_assign_request_does_not_leave_task_confirming_forever(self):
        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            # Neither the request nor any reply is ever delivered.
            for _ in range(10):
                if dispatch.status is not TaskDispatchStatus.CONFIRMING:
                    break
                self.timers.fire_pending()

        assert dispatch.status is TaskDispatchStatus.AVAILABLE
        assert dispatch.assigned_peer is None

    def test_unanswered_request_is_resent_then_released_as_a_retry(self):
        from navpy.modules.swarm.task_auction_coordinator import (
            ASSIGN_REQUEST_SCHEDULE as schedule,
        )

        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            for _ in range(schedule.copies):
                self.timers.fire_pending()
        # The post-release re-advertisement is the only zero-delay timer.
        confirm_timers = [t for t in self.timers.created if t.interval > 0.0]

        requests = _sent(self.network, TaskAssignRequestMsg)
        assert len(requests) == schedule.copies
        assert {(m.receiver_id, m.task.task_id) for m in requests} == {(2, 10)}
        # Every copy carries a fresh UID.
        assert len({m.meta.msg_seq for m in requests}) == schedule.copies
        assert [t.interval for t in confirm_timers] == [
            schedule.delay_after(sent)
            for sent in range(1, schedule.copies + 1)
        ]
        # Released like a rejection: the silent peer's bid is dropped and
        # the release counts against the retry budget.
        assert dispatch.status is TaskDispatchStatus.AVAILABLE
        assert dispatch.assigned_peer is None
        assert set(dispatch.task_handle_by_peer) == {3}
        assert dispatch.retry_count == 1

    def test_release_re_advertises_task_until_silent_peer_bids_again(self):
        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            while dispatch.status is TaskDispatchStatus.CONFIRMING:
                assert self.timers.fire_pending()
            self.network.reset_mock()
            self.timers.fire_pending()

        adverts = _sent(self.network, AvailableTaskRequestMsg)
        assert [t.task_id for m in adverts for t in m.tasks] == [10]
        assert dispatch.has_active_rebroadcast()

    def test_late_accept_after_release_cannot_confirm(self):
        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            while dispatch.status is TaskDispatchStatus.CONFIRMING:
                assert self.timers.fire_pending()
            self.network.reset_mock()

            self.actor.on_message(TaskAssignResponseMsg(
                sender_id=2, receiver_id=1, task_id=10, is_accepted=True,
                meta=_meta(11),
            ))

        # Not confirmed to 2, which is told the task is no longer its own.
        assert dispatch.status is not TaskDispatchStatus.CONFIRMED
        assert dispatch.assigned_peer != 2
        (ack,) = _sent(self.network, SwarmAckMsg)
        assert (ack.receiver_id, ack.ref_msg_seq, ack.status) == (
            2, 11, ACK_STATUS_RECEIVED,
        )
        # Its accept reported it BUSY, so the owner moved on to peer 3.
        assert dispatch.assigned_peer == 3

    def test_accept_stops_resending(self):
        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            (request,) = _sent(self.network, TaskAssignRequestMsg)
            self.actor.on_message(_ack(2, request, ACK_STATUS_RECEIVED, 10))
            self.actor.on_message(TaskAssignResponseMsg(
                sender_id=2, receiver_id=1, task_id=10, is_accepted=True,
                meta=_meta(11),
            ))
            self.network.reset_mock()
            fired = self.timers.fire_pending()

        assert fired == 0
        assert _sent(self.network, TaskAssignRequestMsg) == []
        assert dispatch.status is TaskDispatchStatus.CONFIRMED


def test_stale_confirmation_timer_cannot_touch_replacement_generation():
    from navpy.modules.swarm.task_auction_coordinator import (
        ASSIGN_REQUEST_SCHEDULE,
    )
    from navpy.modules.swarm.task_auction_models import (
        AssignConfirmationPorts,
        TaskReservation,
    )

    _, auction, rebroadcast, confirmation = create_task_state(
        threading.RLock()
    )
    for peer_id in (2, 3):
        rebroadcast.discover_peer(peer_id, lambda *_: None)
    old, generation = auction.register(_task(7))
    old.assigned_peer = 2
    old.set_status(TaskDispatchStatus.CONFIRMING)
    stale = TaskReservation(7, 2, old.task, generation, attempt=1)

    auction.reset(Mock(spec=ILogger))
    for peer_id in (2, 3):
        rebroadcast.discover_peer(peer_id, lambda *_: None)
    replacement, _ = auction.register(_task(7))
    replacement.on_peer_available(2, TaskHandleMsgData(7, 1.0))
    replacement.assigned_peer = 2
    replacement.set_status(TaskDispatchStatus.CONFIRMING)
    send = Mock(return_value=True)

    outcome = confirmation.due(
        stale,
        AssignConfirmationPorts(send, Mock(), lambda _reservation: None),
        ASSIGN_REQUEST_SCHEDULE,
    )

    assert outcome.kind == "stale"
    send.assert_not_called()
    assert replacement.status is TaskDispatchStatus.CONFIRMING
    assert replacement.assigned_peer == 2
    assert set(replacement.task_handle_by_peer) == {2}


class TestPeerParticipation:
    # Owner boots; an owner's seqs only order its own messages.
    BOOTS = {1: 7, 3: 8}

    def setup_method(self):
        vehicle = Mock(spec=IVehicle)
        vehicle.source_system = 2
        vehicle.location.return_value = Location(12.34, 56.78, 90.0)
        self.network = Mock(spec=NetworkAbc)
        self.timers = FakeTimers()
        for patcher in (
            self.timers.patch(),
            patch(
                "navpy.modules.swarm.task_actor.FlyEstimator.time_to_fly",
                return_value=4.0,
            ),
        ):
            patcher.start()
            self._patchers = [*getattr(self, "_patchers", []), patcher]
        self.actor = TaskActor(vehicle, self.network, Mock(spec=ILogger))
        self.actor.start()
        for peer_id in (1, 3):
            self.actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        self.network.reset_mock()

    def teardown_method(self):
        self.actor.reset()
        for patcher in self._patchers:
            patcher.stop()

    def _request(self, owner_id: int, task_id: int, seq: int) -> None:
        self.actor.on_message(TaskAssignRequestMsg(
            sender_id=owner_id, receiver_id=2, task=_assign_task(task_id),
            meta=_meta(seq, self.BOOTS[owner_id]),
        ))
        self.timers.fire_due_now()

    def _advertise(self, owner_id: int, task_id: int, seq: int) -> None:
        self.actor.on_message(AvailableTaskRequestMsg(
            sender_id=owner_id, tasks=[_task(task_id)],
            meta=_meta(seq, self.BOOTS[owner_id]),
        ))

    def _responses(self) -> list[tuple[int, int, bool]]:
        return [
            (m.receiver_id, m.task_id, m.is_accepted)
            for m in _sent(self.network, TaskAssignResponseMsg)
        ]

    def _held_task_id(self):
        held = self.actor._selection.held()
        return None if held is None else held.task.task_id

    def test_resent_request_for_held_task_is_accepted_again(self):
        self._request(1, 10, 50)
        self._request(1, 10, 52)

        assert self._responses() == [(1, 10, True), (1, 10, True)]
        # Invisible to nav until the owner applies one of the answers.
        assert self.actor.selected_poi() is None
        answer = _sent(self.network, TaskAssignResponseMsg)[-1]
        self.actor.on_message(_ack(1, answer, ACK_STATUS_APPLIED, 60, 7))
        assert self.actor.selected_poi().task_id == 10

    def test_request_for_other_task_or_owner_is_rejected_while_holding(self):
        self._request(1, 10, 50)
        self._request(1, 11, 51)
        self._request(3, 10, 50)

        assert self._responses() == [
            (1, 10, True), (1, 11, False), (3, 10, False),
        ]
        assert self._held_task_id() == 10

    def test_owner_re_advertising_held_task_releases_it_and_bids(self):
        self._request(1, 10, 50)
        self.network.reset_mock()

        self._advertise(1, 10, 51)

        assert self._held_task_id() is None
        bids = _sent(self.network, AvailableTaskResponseMsg)
        assert [(m.receiver_id, [t.task_id for t in m.tasks]) for m in bids] == [
            (1, [10]),
        ]

    def test_other_owner_advertising_same_task_id_does_not_release(self):
        self._request(1, 10, 50)
        self.network.reset_mock()

        self._advertise(3, 10, 60)

        assert self._held_task_id() == 10
        assert _sent(self.network, AvailableTaskResponseMsg) == []

    def test_owner_advertising_other_task_does_not_release(self):
        self._request(1, 10, 50)

        self._advertise(1, 11, 51)

        assert self._held_task_id() == 10


def test_rebroadcast_prepared_before_reservation_is_not_sent_after_it():
    """A stale advertisement would make the assigned peer drop the task."""
    vehicle = Mock(spec=IVehicle)
    vehicle.source_system = 1
    vehicle.location.return_value = Location(12.34, 56.78, 90.0)
    network = Mock(spec=NetworkAbc)
    actor = TaskActor(vehicle, network, Mock(spec=ILogger))
    actor.start()
    try:
        for peer_id in (2, 3):
            actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        dispatch = TaskDispatch(_task(10))
        dispatch.on_peer_available(2, TaskHandleMsgData(10, 1.0))
        assert actor._auction_state.register_dispatch(dispatch) is not None
        generation = actor._auction_state.current_generation()
        rebroadcast_state = actor._rebroadcast._rebroadcast
        original_prepare = rebroadcast_state.prepare

        def prepare_then_complete_auction(task_id, plan_generation):
            plan = original_prepare(task_id, plan_generation)
            # Peer 3's bid lands between prepare and send.
            dispatch.on_peer_available(3, TaskHandleMsgData(10, 2.0))
            actor._auction._select_peer_for_task(10, generation)
            return plan

        network.reset_mock()
        with (
            patch.object(
                rebroadcast_state,
                "prepare",
                side_effect=prepare_then_complete_auction,
            ),
            patch.object(TaskDispatch, "start_rebroadcast", return_value=None),
            patch.object(TaskDispatch, "start_confirm_timer", return_value=None),
        ):
            actor._rebroadcast._rebroadcast_task(10, generation)

        sent = [call.args[0] for call in network.broadcast.call_args_list]
        assert dispatch.status is TaskDispatchStatus.CONFIRMING
        assert [type(message) for message in sent] == [TaskAssignRequestMsg]
    finally:
        actor.reset()


def test_peer_ignores_advertisement_older_than_accepted_request():
    vehicle = Mock(spec=IVehicle)
    vehicle.source_system = 2
    vehicle.location.return_value = Location(12.34, 56.78, 90.0)
    actor = TaskActor(vehicle, Mock(spec=NetworkAbc), Mock(spec=ILogger))
    actor.start()
    try:
        for peer_id in (1, 3):
            actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        with patch(
            "navpy.modules.swarm.task_actor.FlyEstimator.time_to_fly",
            return_value=4.0,
        ):
            actor.on_message(TaskAssignRequestMsg(
                sender_id=1, receiver_id=2, task=_assign_task(10),
                meta=MsgMeta(boot_id=7, msg_seq=50, time_ms=0, ttl_ms=5000),
            ))

            # Reordered on the link: sent before the request.
            actor.on_message(AvailableTaskRequestMsg(
                sender_id=1, tasks=[_task(10)],
                meta=MsgMeta(boot_id=7, msg_seq=49, time_ms=0, ttl_ms=5000),
            ))
            assert actor._selection.held().task.task_id == 10

            actor.on_message(AvailableTaskRequestMsg(
                sender_id=1, tasks=[_task(10)],
                meta=MsgMeta(boot_id=7, msg_seq=51, time_ms=0, ttl_ms=5000),
            ))
        assert actor._selection.held() is None
    finally:
        actor.reset()


def test_failed_rebroadcast_send_keeps_the_rebroadcast_chain():
    vehicle = Mock(spec=IVehicle)
    vehicle.source_system = 1
    vehicle.location.return_value = Location(12.34, 56.78, 90.0)
    network = Mock(spec=NetworkAbc)
    actor = TaskActor(vehicle, network, Mock(spec=ILogger))
    actor.start()
    try:
        for peer_id in (2, 3):
            actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        dispatch = TaskDispatch(_task(10))
        assert actor._auction_state.register_dispatch(dispatch) is not None
        network.broadcast.side_effect = OSError("link down")

        with patch.object(TaskDispatch, "start_rebroadcast") as start:
            actor._rebroadcast._rebroadcast_task(
                10, actor._auction_state.current_generation()
            )

        start.assert_called_once()
    finally:
        network.broadcast.side_effect = None
        actor.reset()
