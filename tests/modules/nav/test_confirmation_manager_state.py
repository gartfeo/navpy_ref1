from __future__ import annotations

import threading
import time
from unittest.mock import Mock

import pytest

from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.nav.confirmation_round_transaction import ConfirmationRequestRef
from navpy.modules.nav.target_confirmation import ConfirmationRoundRunner
from navpy.modules.nav.confirmation_manager_state import (
    ConfirmationResponseKind,
    ConfirmationWorkerLease,
    ConfirmationManagerState,
    ConfirmationStatus,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from tests.detection_factory import make_detected_target


def _target(task_id: int) -> DetectedObject:
    return make_detected_target(obj_id=task_id, task_id=task_id, class_id=0)


def test_start_review_atomically_registers_status_and_worker_lease() -> None:
    state = ConfirmationManagerState()
    target = _target(7)

    lease = state.start_review(target, threading.Event)

    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMING
    assert state.active_worker_count == 1
    assert state.worker_is_current(lease)


def test_reset_invalidates_not_yet_scheduled_worker() -> None:
    state = ConfirmationManagerState()
    target = _target(8)
    lease = state.start_review(target, threading.Event)

    state.reset()

    assert not state.worker_is_current(lease)
    assert lease.cancel_event.is_set()
    assert state.registry.get_status(target) is None
    assert state.active_worker_count == 0


def test_response_after_reset_stays_late_and_cannot_restore_status() -> None:
    state = ConfirmationManagerState()
    target = _target(81)
    worker = state.start_review(target, threading.Event)
    confirmation = state.begin_round(target, worker, threading.Event)
    assert confirmation is not None

    state.reset()

    assert (
        state.resolve_response(81, True)
        is ConfirmationResponseKind.LATE_OR_DUPLICATE
    )
    assert state.registry.get_status(target) is None


def test_stale_round_cannot_finish_or_remove_replacement_payload() -> None:
    state = ConfirmationManagerState()
    old_target = _target(9)
    new_target = _target(9)
    old_worker = state.start_review(old_target, threading.Event)
    old_round = state.begin_round(old_target, old_worker, threading.Event)
    new_worker = state.start_review(new_target, threading.Event)
    new_round = state.begin_round(new_target, new_worker, threading.Event)
    assert old_round is not None
    assert new_round is not None

    assert not state.complete_round(old_round, ConfirmationStatus.TIMEOUT_REJECTED)
    assert state.pending_target(9) is new_target


def test_replacement_worker_invalidates_older_worker_for_same_target() -> None:
    state = ConfirmationManagerState()
    target = _target(90)
    old_worker = state.start_review(target, threading.Event)
    replacement_worker = state.start_review(target, threading.Event)

    assert not state.worker_is_current(old_worker)
    assert state.worker_is_current(replacement_worker)
    assert not state.set_worker_status(old_worker, ConfirmationStatus.CONFIRMED)
    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMING


def test_response_consumes_exact_round_once() -> None:
    state = ConfirmationManagerState()
    target = _target(10)
    worker = state.start_review(target, threading.Event)
    confirmation = state.begin_round(target, worker, threading.Event)
    assert confirmation is not None

    assert (
        state.resolve_response(10, True)
        is ConfirmationResponseKind.CONFIRMED
    )
    assert confirmation.response_event.is_set()
    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMED
    assert (
        state.resolve_response(10, True)
        is ConfirmationResponseKind.LATE_OR_DUPLICATE
    )


def test_first_use_metadata_less_response_remains_legacy_compatible() -> None:
    state = ConfirmationManagerState()
    target = _target(101)
    worker = state.start_review(target, threading.Event)
    confirmation = state.begin_round(target, worker, threading.Event)
    assert confirmation is not None

    assert (
        state.resolve_response(101, True, response_ref=None)
        is ConfirmationResponseKind.CONFIRMED
    )


def test_correlated_approve_and_recall_share_the_exact_round_token() -> None:
    state = ConfirmationManagerState()
    target = _target(102)
    worker = state.start_review(target, threading.Event)
    request_ref = ConfirmationRequestRef(9001, 12)
    confirmation = state.begin_round(
        target,
        worker,
        threading.Event,
        request_ref,
    )
    assert confirmation is not None

    assert (
        state.resolve_response(102, True, request_ref)
        is ConfirmationResponseKind.CONFIRMED
    )
    assert (
        state.resolve_response(102, True, request_ref)
        is ConfirmationResponseKind.LATE_OR_DUPLICATE
    )
    assert (
        state.resolve_response(102, False, ConfirmationRequestRef(9001, 11))
        is ConfirmationResponseKind.LATE_OR_DUPLICATE
    )
    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMED
    assert (
        state.resolve_response(102, False, request_ref)
        is ConfirmationResponseKind.CANCELLATION_REQUESTED
    )
    assert state.registry.get_status(target) is ConfirmationStatus.REJECTED


def test_reset_clears_retained_confirmation_round_token() -> None:
    state = ConfirmationManagerState()
    target = _target(103)
    worker = state.start_review(target, threading.Event)
    request_ref = ConfirmationRequestRef(9001, 13)
    confirmation = state.begin_round(
        target,
        worker,
        threading.Event,
        request_ref,
    )
    assert confirmation is not None
    assert (
        state.resolve_response(103, True, request_ref)
        is ConfirmationResponseKind.CONFIRMED
    )

    state.reset()

    assert (
        state.resolve_response(103, False, request_ref)
        is ConfirmationResponseKind.LATE_OR_DUPLICATE
    )
    assert state.registry.get_status(target) is None


@pytest.mark.parametrize(
    "meta",
    (
        MsgMeta(True, 1, 0, 0),
        MsgMeta(1, False, 0, 0),
        MsgMeta("1", 1, 0, 0),
        MsgMeta(1, "1", 0, 0),
        MsgMeta(-1, 1, 0, 0),
        MsgMeta(1, 0x1_0000_0000, 0, 0),
    ),
)
def test_malformed_confirmation_metadata_is_not_a_round_reference(meta) -> None:
    with pytest.raises(ValueError, match="confirmation request metadata"):
        ConfirmationRequestRef.from_meta(meta)


def test_confirmation_reference_accepts_uint32_zero_edge_values() -> None:
    assert ConfirmationRequestRef.from_meta(MsgMeta(0, 1, 0, 0)) == (
        ConfirmationRequestRef(0, 1)
    )
    assert ConfirmationRequestRef.from_meta(MsgMeta(1, 0, 0, 0)) == (
        ConfirmationRequestRef(1, 0)
    )
    assert ConfirmationRequestRef.from_meta(MsgMeta(0, 0, 0, 0)) is None


def test_finish_worker_retires_its_unfinished_round_and_wakes_waiter() -> None:
    state = ConfirmationManagerState()
    target = _target(11)
    worker = state.start_review(target, threading.Event)
    confirmation = state.begin_round(target, worker, threading.Event)
    assert confirmation is not None

    state.finish_worker(worker)

    assert state.pending_target(11) is None
    assert confirmation.response_event.is_set()
    assert state.active_worker_count == 0


def test_finishing_old_worker_does_not_remove_replacement_round() -> None:
    state = ConfirmationManagerState()
    old_target = _target(12)
    replacement = _target(12)
    old_worker = state.start_review(old_target, threading.Event)
    old_round = state.begin_round(old_target, old_worker, threading.Event)
    new_worker = state.start_review(replacement, threading.Event)
    new_round = state.begin_round(replacement, new_worker, threading.Event)
    assert old_round is not None
    assert new_round is not None

    state.finish_worker(old_worker)

    assert state.pending_target(12) is replacement
    assert not new_round.response_event.is_set()


def test_response_waits_for_in_flight_resend_transaction() -> None:
    state = ConfirmationManagerState()
    target = _target(13)
    worker = state.start_review(target, threading.Event)
    resend_entered = threading.Event()
    release_resend = threading.Event()
    broadcast_count = 0

    def broadcast(_request: object) -> None:
        nonlocal broadcast_count
        broadcast_count += 1
        if broadcast_count == 2:
            resend_entered.set()
            if not release_resend.wait(2.0):
                raise TimeoutError("test did not release resend")

    network = Mock()
    network.broadcast.side_effect = broadcast
    failure_policy = Mock()
    failure_policy.status.return_value = ConfirmationStatus.TIMEOUT_REJECTED
    runner = ConfirmationRoundRunner(
        sys_id=1,
        state=state,
        network=lambda: network,
        wait_time_s=lambda: 1.0,
        resend_interval_s=lambda: 0.01,
        failure_policy=failure_policy,
        media=Mock(),
        logger=Mock(),
        monotonic=time.monotonic,
        event_factory=threading.Event,
    )
    runner_thread = threading.Thread(target=runner.run, args=(target, worker))
    runner_thread.start()
    assert resend_entered.wait(1.0)

    observed_pending: list[object] = []
    inspection_finished = threading.Event()

    def inspect_state() -> None:
        observed_pending.append(state.pending_target(13))
        inspection_finished.set()

    inspection_thread = threading.Thread(target=inspect_state)
    inspection_thread.start()
    assert inspection_finished.wait(0.2), "broadcast held shared state RLock"
    inspection_thread.join(timeout=1.0)
    assert observed_pending == [target]

    response: list[ConfirmationResponseKind] = []
    response_finished = threading.Event()

    def resolve() -> None:
        response.append(state.resolve_response(13, True))
        response_finished.set()

    response_thread = threading.Thread(target=resolve)
    response_thread.start()
    completed_while_broadcast_blocked = response_finished.wait(0.2)
    release_resend.set()
    response_thread.join(timeout=1.0)
    runner_thread.join(timeout=1.0)

    assert not completed_while_broadcast_blocked
    assert response == [ConfirmationResponseKind.CONFIRMED]
    assert not runner_thread.is_alive()
    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMED


def test_response_winning_gate_window_prevents_resend_from_starting() -> None:
    state = ConfirmationManagerState()
    target = _target(15)
    worker = state.start_review(target, threading.Event)
    resend_network_lookup = threading.Event()
    release_network_lookup = threading.Event()
    network = Mock()
    network_lookups = 0

    def current_network() -> Mock:
        nonlocal network_lookups
        network_lookups += 1
        if network_lookups == 2:
            resend_network_lookup.set()
            if not release_network_lookup.wait(2.0):
                raise TimeoutError("test did not release network lookup")
        return network

    failure_policy = Mock()
    failure_policy.status.return_value = ConfirmationStatus.TIMEOUT_REJECTED
    runner = ConfirmationRoundRunner(
        sys_id=1,
        state=state,
        network=current_network,
        wait_time_s=lambda: 1.0,
        resend_interval_s=lambda: 0.01,
        failure_policy=failure_policy,
        media=Mock(),
        logger=Mock(),
        monotonic=time.monotonic,
        event_factory=threading.Event,
    )
    runner_thread = threading.Thread(target=runner.run, args=(target, worker))
    runner_thread.start()
    assert resend_network_lookup.wait(1.0)

    result = state.resolve_response(15, True)
    release_network_lookup.set()
    runner_thread.join(timeout=1.0)

    assert result is ConfirmationResponseKind.CONFIRMED
    assert not runner_thread.is_alive()
    assert network.broadcast.call_count == 1
    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMED


@pytest.mark.parametrize(
    "resolution",
    ("complete", "retire", "finish", "reset", "replace"),
)
def test_every_exact_round_resolution_waits_for_outbound_transaction(
    resolution: str,
) -> None:
    state = ConfirmationManagerState()
    target = _target(20)
    worker = state.start_review(target, threading.Event)
    confirmation = state.begin_round(target, worker, threading.Event)
    assert confirmation is not None
    send_entered = threading.Event()
    release_send = threading.Event()

    def blocked_send() -> None:
        send_entered.set()
        if not release_send.wait(2.0):
            raise TimeoutError("test did not release outbound transaction")

    send_thread = threading.Thread(
        target=lambda: state.send_if_current(confirmation, blocked_send),
    )
    send_thread.start()
    assert send_entered.wait(1.0)
    resolution_finished = threading.Event()

    def resolve() -> None:
        if resolution == "complete":
            state.complete_round(confirmation, ConfirmationStatus.TIMEOUT_REJECTED)
        elif resolution == "retire":
            state.retire_round(confirmation)
        elif resolution == "finish":
            state.finish_worker(worker)
        elif resolution == "reset":
            state.reset()
        else:
            replacement = _target(20)
            replacement_worker = state.start_review(replacement, threading.Event)
            state.begin_round(replacement, replacement_worker, threading.Event)
        resolution_finished.set()

    resolution_thread = threading.Thread(target=resolve)
    resolution_thread.start()
    resolved_while_send_blocked = resolution_finished.wait(0.2)
    release_send.set()
    send_thread.join(timeout=1.0)
    resolution_thread.join(timeout=1.0)

    assert not resolved_while_send_blocked
    assert resolution_finished.is_set()
    assert confirmation.response_event.is_set()


def test_late_network_loss_leaves_no_pending_confirmation_round() -> None:
    state = ConfirmationManagerState()
    target = _target(14)
    worker = state.start_review(target, threading.Event)
    failure_policy = Mock()
    failure_policy.status.return_value = ConfirmationStatus.TIMEOUT_REJECTED
    runner = ConfirmationRoundRunner(
        sys_id=1,
        state=state,
        network=lambda: None,
        wait_time_s=lambda: 1.0,
        resend_interval_s=lambda: 0.01,
        failure_policy=failure_policy,
        media=Mock(),
        logger=Mock(),
        monotonic=time.monotonic,
        event_factory=threading.Event,
    )

    runner.run(target, worker)

    assert state.pending_target(14) is None
    assert state.registry.get_status(target) is ConfirmationStatus.CONFIRMED


def _runner_for_outbound_failure(
    error: Exception,
) -> tuple[
    ConfirmationRoundRunner,
    ConfirmationManagerState,
    Mock,
    ConfirmationWorkerLease,
]:
    state = ConfirmationManagerState()
    target = _target(91)
    worker = state.start_review(target, threading.Event)
    network = Mock()
    network.broadcast.side_effect = error
    failure_policy = Mock()
    failure_policy.status.return_value = ConfirmationStatus.TIMEOUT_REJECTED
    runner = ConfirmationRoundRunner(
        sys_id=1,
        state=state,
        network=lambda: network,
        wait_time_s=lambda: 1.0,
        resend_interval_s=lambda: 0.01,
        failure_policy=failure_policy,
        media=Mock(),
        logger=Mock(),
        monotonic=time.monotonic,
        event_factory=threading.Event,
    )
    return runner, state, target, worker


def test_transport_oserror_is_contained_by_confirmation_round() -> None:
    runner, state, target, worker = _runner_for_outbound_failure(
        OSError("radio unavailable"),
    )

    runner.run(target, worker)

    assert state.registry.get_status(target) is ConfirmationStatus.TIMEOUT_REJECTED
    assert state.pending_target(91) is None


def test_programmer_typeerror_propagates_from_confirmation_send() -> None:
    runner, _state, target, worker = _runner_for_outbound_failure(
        TypeError("broadcast signature mismatch"),
    )

    with pytest.raises(TypeError, match="signature mismatch"):
        runner.run(target, worker)
