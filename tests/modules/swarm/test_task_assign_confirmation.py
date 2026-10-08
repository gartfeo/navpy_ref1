"""Lost TASK_ASSIGN_REQUEST / TASK_ASSIGN_RESPONSE must not strand a task.

Transport is best effort with no ack, so the owner resends an unanswered
assign request and finally releases the CONFIRMING reservation.
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
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.modules.comm.messages.types import TaskDispatchStatus, TaskTypeMsgData
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
            ASSIGN_CONFIRMATION,
        )

        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            for _ in range(ASSIGN_CONFIRMATION.max_sends):
                self.timers.fire_pending()
        # The post-release re-advertisement is the only zero-delay timer.
        confirm_timers = [t for t in self.timers.created if t.interval > 0.0]

        requests = _sent(self.network, TaskAssignRequestMsg)
        assert len(requests) == ASSIGN_CONFIRMATION.max_sends
        assert {(m.receiver_id, m.task.task_id) for m in requests} == {(2, 10)}
        assert [t.interval for t in confirm_timers] == [
            ASSIGN_CONFIRMATION.resend_interval_s,
        ] * (ASSIGN_CONFIRMATION.max_sends - 1) + [
            ASSIGN_CONFIRMATION.release_delay_s,
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

        self.actor.on_message(TaskAssignResponseMsg(
            sender_id=2, receiver_id=1, task_id=10, is_accepted=True,
        ))

        assert dispatch.status is TaskDispatchStatus.AVAILABLE
        assert dispatch.assigned_peer is None

    def test_accept_stops_resending(self):
        with self.timers.patch():
            dispatch = self._reserve_task_10_for_peer_2()
            self.actor.on_message(TaskAssignResponseMsg(
                sender_id=2, receiver_id=1, task_id=10, is_accepted=True,
            ))
            self.network.reset_mock()
            fired = self.timers.fire_pending()

        assert fired == 0
        assert _sent(self.network, TaskAssignRequestMsg) == []
        assert dispatch.status is TaskDispatchStatus.CONFIRMED


def test_stale_confirmation_timer_cannot_touch_replacement_generation():
    from navpy.modules.swarm.task_auction_coordinator import (
        ASSIGN_CONFIRMATION,
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
    stale = TaskReservation(7, 2, old.task, generation)

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
        ASSIGN_CONFIRMATION,
    )

    assert outcome.kind == "stale"
    send.assert_not_called()
    assert replacement.status is TaskDispatchStatus.CONFIRMING
    assert replacement.assigned_peer == 2
    assert set(replacement.task_handle_by_peer) == {2}


class TestPeerParticipation:
    def setup_method(self):
        vehicle = Mock(spec=IVehicle)
        vehicle.source_system = 2
        vehicle.location.return_value = Location(12.34, 56.78, 90.0)
        self.network = Mock(spec=NetworkAbc)
        self.actor = TaskActor(vehicle, self.network, Mock(spec=ILogger))
        self.actor.start()
        for peer_id in (1, 3):
            self.actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        self.network.reset_mock()

    def teardown_method(self):
        self.actor.reset()

    def _request(self, owner_id: int, task_id: int) -> None:
        self.actor.on_message(TaskAssignRequestMsg(
            sender_id=owner_id, receiver_id=2, task=_assign_task(task_id),
        ))

    def _responses(self) -> list[tuple[int, int, bool]]:
        return [
            (m.receiver_id, m.task_id, m.is_accepted)
            for m in _sent(self.network, TaskAssignResponseMsg)
        ]

    def test_resent_request_for_held_task_is_accepted_again(self):
        self._request(1, 10)
        self._request(1, 10)

        assert self._responses() == [(1, 10, True), (1, 10, True)]
        assert self.actor.selected_poi().task_id == 10

    def test_request_for_other_task_or_owner_is_rejected_while_holding(self):
        self._request(1, 10)
        self._request(1, 11)
        self._request(3, 10)

        assert self._responses() == [
            (1, 10, True), (1, 11, False), (3, 10, False),
        ]
        assert self.actor.selected_poi().task_id == 10

    def test_owner_re_advertising_held_task_releases_it_and_bids(self):
        self._request(1, 10)
        self.network.reset_mock()

        with patch(
            "navpy.modules.swarm.task_actor.FlyEstimator.time_to_fly",
            return_value=4.0,
        ):
            self.actor.on_message(AvailableTaskRequestMsg(
                sender_id=1, tasks=[_task(10)],
            ))

        assert self.actor.selected_poi() is None
        bids = _sent(self.network, AvailableTaskResponseMsg)
        assert [(m.receiver_id, [t.task_id for t in m.tasks]) for m in bids] == [
            (1, [10]),
        ]

    def test_other_owner_advertising_same_task_id_does_not_release(self):
        self._request(1, 10)
        self.network.reset_mock()

        self.actor.on_message(AvailableTaskRequestMsg(
            sender_id=3, tasks=[_task(10)],
        ))

        assert self.actor.selected_poi().task_id == 10
        assert _sent(self.network, AvailableTaskResponseMsg) == []

    def test_owner_advertising_other_task_does_not_release(self):
        self._request(1, 10)

        self.actor.on_message(AvailableTaskRequestMsg(
            sender_id=1, tasks=[_task(11)],
        ))

        assert self.actor.selected_poi().task_id == 10


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
    from navpy.modules.comm.messages.msg_meta import MsgMeta

    vehicle = Mock(spec=IVehicle)
    vehicle.source_system = 2
    vehicle.location.return_value = Location(12.34, 56.78, 90.0)
    actor = TaskActor(vehicle, Mock(spec=NetworkAbc), Mock(spec=ILogger))
    actor.start()
    try:
        for peer_id in (1, 3):
            actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))
        actor.on_message(TaskAssignRequestMsg(
            sender_id=1, receiver_id=2, task=_assign_task(10),
            meta=MsgMeta(boot_id=7, msg_seq=50, time_ms=0, ttl_ms=5000),
        ))

        # Reordered on the link: sent before the request.
        actor.on_message(AvailableTaskRequestMsg(
            sender_id=1, tasks=[_task(10)],
            meta=MsgMeta(boot_id=7, msg_seq=49, time_ms=0, ttl_ms=5000),
        ))
        assert actor.selected_poi().task_id == 10

        with patch(
            "navpy.modules.swarm.task_actor.FlyEstimator.time_to_fly",
            return_value=4.0,
        ):
            actor.on_message(AvailableTaskRequestMsg(
                sender_id=1, tasks=[_task(10)],
                meta=MsgMeta(boot_id=7, msg_seq=51, time_ms=0, ttl_ms=5000),
            ))
        assert actor.selected_poi() is None
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
