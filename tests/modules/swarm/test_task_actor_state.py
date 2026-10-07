import threading
from unittest.mock import Mock
from unittest.mock import patch

import pytest

from navpy.modules.comm.messages.available_task_msg import (
    TaskAssignMsgData,
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import TaskDispatchStatus, TaskTypeMsgData
from navpy.modules.swarm.task_actor_state import create_task_state
from navpy.modules.swarm.task_assignment_planner import (
    MinimumEtaAssignmentPlanner,
)
from navpy.modules.swarm.task_dispatch import TaskDispatch


def _task(task_id: int) -> TaskMsgData:
    return TaskMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(1.0, 2.0, 3.0),
    )


def test_reset_generation_fences_old_selection_timer_from_reused_task_id():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    old_dispatch, _ = auction.register(_task(7))
    callbacks = []

    with patch.object(
        TaskDispatch,
        "start_peer_select_timer",
        side_effect=lambda callback, _delay: callbacks.append(callback),
    ):
        assert auction.record_offer(
            7,
            2,
            TaskHandleMsgData(task_id=7, time_in_min=1.0),
            lambda _task_id, generation: auction.plan_and_reserve(
                MinimumEtaAssignmentPlanner(),
                generation,
            ),
            2.0,
        )

    auction.reset(logger=type("Logger", (), {"error": lambda *_: None})())
    new_dispatch, _ = auction.register(_task(7))
    new_dispatch.on_peer_available(
        3,
        TaskHandleMsgData(task_id=7, time_in_min=0.5),
    )

    callbacks[0](old_dispatch.task.task_id)

    assert new_dispatch.status == TaskDispatchStatus.AVAILABLE
    assert new_dispatch.assigned_peer is None


def test_complete_bid_matrix_requires_every_free_peer_for_every_available_task():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    first, generation = auction.register(_task(30))
    second, _ = auction.register(_task(31))
    planner = MinimumEtaAssignmentPlanner()
    send = Mock(return_value=True)

    first.on_peer_available(2, TaskHandleMsgData(task_id=30, time_in_min=1.0))
    second.on_peer_available(2, TaskHandleMsgData(task_id=31, time_in_min=2.0))
    assert auction.plan_reserve_and_send_if_complete(
        planner, generation, send
    ) is None
    send.assert_not_called()

    rebroadcast.discover_peer(3, lambda *_: None)
    first.on_peer_available(3, TaskHandleMsgData(task_id=30, time_in_min=3.0))
    assert auction.plan_reserve_and_send_if_complete(
        planner, generation, send
    ) is None
    send.assert_not_called()

    second.on_peer_available(3, TaskHandleMsgData(task_id=31, time_in_min=1.0))
    sent = auction.plan_reserve_and_send_if_complete(planner, generation, send)

    assert sent is True
    assert {
        (call.args[0].task_id, call.args[0].peer_id)
        for call in send.call_args_list
    } == {
        (30, 2),
        (31, 3),
    }


def test_complete_bid_matrix_only_requires_offers_from_free_peers():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    rebroadcast.discover_peer(3, lambda *_: None)
    available, generation = auction.register(_task(40))
    busy, _ = auction.register(_task(41))
    available.on_peer_available(
        2,
        TaskHandleMsgData(task_id=40, time_in_min=1.0),
    )
    busy.assigned_peer = 3
    busy.set_status(TaskDispatchStatus.CONFIRMING)

    send = Mock(return_value=True)
    sent = auction.plan_reserve_and_send_if_complete(
        MinimumEtaAssignmentPlanner(),
        generation,
        send,
    )

    assert sent is True
    assert [
        (call.args[0].task_id, call.args[0].peer_id)
        for call in send.call_args_list
    ] == [(40, 2)]


def test_new_available_task_invalidates_complete_bid_matrix():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    rebroadcast.discover_peer(3, lambda *_: None)
    complete, generation = auction.register(_task(50))
    for peer_id in (2, 3):
        complete.on_peer_available(
            peer_id,
            TaskHandleMsgData(task_id=50, time_in_min=float(peer_id)),
        )
    auction.register(_task(51))

    send = Mock(return_value=True)
    assert auction.plan_reserve_and_send_if_complete(
        MinimumEtaAssignmentPlanner(),
        generation,
        send,
    ) is None
    send.assert_not_called()


@pytest.mark.parametrize(
    "eta",
    [True, -1.0, float("nan"), float("inf"), 1e9, 10**10000],
    ids=["bool", "negative", "nan", "infinite", "sentinel", "huge-int"],
)
def test_record_offer_rejects_invalid_eta_without_mutating_dispatch(eta):
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    dispatch, _ = auction.register(_task(12))
    dispatch.start_peer_select_timer = Mock()

    recorded = auction.record_offer(
        12,
        2,
        TaskHandleMsgData(task_id=12, time_in_min=eta),
        lambda *_: None,
        2.0,
    )

    assert not recorded
    assert dispatch.task_handle_by_peer == {}
    dispatch.start_peer_select_timer.assert_not_called()


def test_record_offer_rejects_late_bid_while_assignment_is_confirming():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    dispatch, _ = auction.register(_task(13))
    dispatch.assigned_peer = 2
    dispatch.set_status(TaskDispatchStatus.CONFIRMING)
    dispatch.start_peer_select_timer = Mock()

    assert not auction.record_offer(
        13,
        2,
        TaskHandleMsgData(task_id=13, time_in_min=0.5),
        lambda *_: None,
        2.0,
    )
    assert dispatch.task_handle_by_peer == {}
    dispatch.start_peer_select_timer.assert_not_called()


def test_complete_assignment_send_is_atomic_against_reset():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    rebroadcast.discover_peer(3, lambda *_: None)
    first, generation = auction.register(_task(60))
    second, _ = auction.register(_task(61))
    for dispatch in (first, second):
        for peer_id in (2, 3):
            dispatch.on_peer_available(
                peer_id,
                TaskHandleMsgData(
                    task_id=dispatch.task.task_id,
                    time_in_min=float(dispatch.task.task_id + peer_id),
                ),
            )
    send_started = threading.Event()
    release_send = threading.Event()
    reset_done = threading.Event()
    sent = []

    def send(reservation):
        sent.append((reservation.task_id, reservation.peer_id))
        if len(sent) == 1:
            send_started.set()
            assert release_send.wait(2.0)
        return True

    assignment_thread = threading.Thread(
        target=lambda: auction.plan_reserve_and_send_if_complete(
            MinimumEtaAssignmentPlanner(),
            generation,
            send,
        )
    )
    assignment_thread.start()
    assert send_started.wait(1.0)
    reset_thread = threading.Thread(
        target=lambda: (auction.reset(Mock()), reset_done.set()),
    )
    reset_thread.start()
    assert not reset_done.wait(0.05)
    release_send.set()
    assignment_thread.join(1.0)
    reset_thread.join(1.0)

    assert len(sent) == 2
    assert reset_done.is_set()
    assert auction.task_ids() == set()


def test_complete_assignment_send_failure_releases_all_unsent_reservations():
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    rebroadcast.discover_peer(3, lambda *_: None)
    first, generation = auction.register(_task(70))
    second, _ = auction.register(_task(71))
    for dispatch in (first, second):
        for peer_id in (2, 3):
            dispatch.on_peer_available(
                peer_id,
                TaskHandleMsgData(
                    task_id=dispatch.task.task_id,
                    time_in_min=float(dispatch.task.task_id + peer_id),
                ),
            )

    with pytest.raises(RuntimeError, match="transport"):
        auction.plan_reserve_and_send_if_complete(
            MinimumEtaAssignmentPlanner(),
            generation,
            lambda _reservation: (_ for _ in ()).throw(
                RuntimeError("transport")
            ),
        )

    assert first.status is TaskDispatchStatus.AVAILABLE
    assert second.status is TaskDispatchStatus.AVAILABLE
    assert first.assigned_peer is None
    assert second.assigned_peer is None


def test_selected_task_slot_accepts_exactly_one_concurrent_assignment():
    selection, _, _ = create_task_state(threading.RLock())
    barrier = threading.Barrier(3)
    accepted: list[bool] = []

    def attempt(task_id: int) -> None:
        barrier.wait()
        accepted.append(selection.try_accept(TaskAssignMsgData(
            task_id=task_id,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(1.0, 2.0, 3.0),
        )))

    threads = [threading.Thread(target=attempt, args=(task_id,)) for task_id in (1, 2)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(timeout=1.0)

    assert accepted.count(True) == 1
    assert accepted.count(False) == 1
    assert selection.selected().task_id in {1, 2}


def test_planner_runs_without_holding_auction_state_lock() -> None:
    _, auction, _ = create_task_state(threading.RLock())
    dispatch, generation = auction.register(_task(20))
    dispatch.on_peer_available(
        2,
        TaskHandleMsgData(task_id=20, time_in_min=1.0),
    )
    planner_entered = threading.Event()
    release_planner = threading.Event()

    class BlockingPlanner:
        def plan(self, _offers, _busy_peers):
            planner_entered.set()
            assert release_planner.wait(2.0)
            return [(20, 2)]

    planning_thread = threading.Thread(
        target=auction.plan_and_reserve,
        args=(BlockingPlanner(), generation),
    )
    planning_thread.start()
    assert planner_entered.wait(1.0)

    generation_read = threading.Event()
    reader = threading.Thread(
        target=lambda: (auction.current_generation(), generation_read.set()),
    )
    reader.start()
    completed_while_planner_blocked = generation_read.wait(0.2)
    release_planner.set()
    reader.join(timeout=1.0)
    planning_thread.join(timeout=1.0)

    assert completed_while_planner_blocked
    assert not planning_thread.is_alive()


def test_generation_change_during_planning_invalidates_snapshot() -> None:
    _, auction, _ = create_task_state(threading.RLock())
    dispatch, generation = auction.register(_task(21))
    dispatch.on_peer_available(
        2,
        TaskHandleMsgData(task_id=21, time_in_min=1.0),
    )
    planner_entered = threading.Event()
    release_planner = threading.Event()
    reservations = []

    class BlockingPlanner:
        def plan(self, _offers, _busy_peers):
            planner_entered.set()
            assert release_planner.wait(2.0)
            return [(21, 2)]

    planning_thread = threading.Thread(
        target=lambda: reservations.extend(
            auction.plan_and_reserve(BlockingPlanner(), generation)
        ),
    )
    planning_thread.start()
    assert planner_entered.wait(1.0)

    reset_finished = threading.Event()
    reset_thread = threading.Thread(
        target=lambda: (
            auction.reset(Mock()),
            reset_finished.set(),
        ),
    )
    reset_thread.start()
    reset_completed_while_planner_blocked = reset_finished.wait(0.2)
    release_planner.set()
    reset_thread.join(timeout=1.0)
    planning_thread.join(timeout=1.0)

    assert reset_completed_while_planner_blocked
    assert reservations == []
    assert auction.task_ids() == set()


def test_shutdown_generation_fences_old_selection_callback() -> None:
    _, auction, rebroadcast = create_task_state(threading.RLock())
    rebroadcast.discover_peer(2, lambda *_: None)
    old_dispatch, generation = auction.register(_task(22))
    callbacks = []

    with patch.object(
        TaskDispatch,
        "start_peer_select_timer",
        side_effect=lambda callback, _delay: callbacks.append(callback),
    ):
        assert auction.record_offer(
            22,
            2,
            TaskHandleMsgData(task_id=22, time_in_min=1.0),
            lambda _task_id, current_generation: auction.plan_and_reserve(
                MinimumEtaAssignmentPlanner(),
                current_generation,
            ),
            2.0,
        )

    auction.shutdown(Mock())
    callbacks[0](old_dispatch.task.task_id)

    assert auction.current_generation() == generation + 1
    assert auction.task_ids() == set()
    assert old_dispatch.status == TaskDispatchStatus.AVAILABLE
    assert old_dispatch.assigned_peer is None


def test_shutdown_rejects_late_registration_and_is_idempotent() -> None:
    _, auction, rebroadcast = create_task_state(threading.RLock())
    dispatch, generation = auction.register(_task(23))
    dispatch.shutdown = Mock(wraps=dispatch.shutdown)

    auction.shutdown(Mock())
    shutdown_generation = auction.current_generation()
    auction.shutdown(Mock())

    assert shutdown_generation == generation + 1
    assert auction.current_generation() == shutdown_generation
    assert dispatch.shutdown.call_count == 1
    assert auction.register(_task(24)) is None
    assert not rebroadcast.discover_peer(2, lambda *_: None)
