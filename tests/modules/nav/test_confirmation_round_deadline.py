"""An operator approval counts only before its round's deadline.

State-level cases: real ConfirmationManagerState, a hand-moved clock, and the
worker's steps (complete, retire, replace) called directly.
"""

from __future__ import annotations

import dataclasses
import math
import threading
from typing import Optional

import pytest

from navpy.core.ports.detect_data import DetectedObject
from navpy.core.review.confirmation_manager_state import ConfirmationManagerState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.review.confirmation_round_transaction import (
    ConfirmationDeadline,
    ConfirmationRequestRef,
    ConfirmationResponseKind,
    ConfirmationRound,
    ConfirmationWorkerLease,
)
from tests.confirmation_doubles import SettableClock
from tests.detection_factory import make_detected_poi

Kind = ConfirmationResponseKind
START_S = 1.0
WAIT_S = 1000.0
DEADLINE_S = START_S + WAIT_S


def _poi(task_id: int) -> DetectedObject:
    return make_detected_poi(obj_id=task_id, task_id=task_id, class_id=0)


def _open_round(
    state: ConfirmationManagerState,
    poi: DetectedObject,
    clock: SettableClock,
    wait_s: float = WAIT_S,
    request_ref: Optional[ConfirmationRequestRef] = None,
) -> tuple[ConfirmationWorkerLease, ConfirmationRound]:
    worker = state.start_review(poi, threading.Event)
    confirmation = state.begin_round(
        poi,
        worker,
        threading.Event,
        request_ref,
        deadline=ConfirmationDeadline(clock, wait_s),
    )
    assert confirmation is not None
    return worker, confirmation


def test_approval_just_before_the_deadline_confirms() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(31), SettableClock(START_S)
    _worker, confirmation = _open_round(state, poi, clock)

    clock.now_s = math.nextafter(DEADLINE_S, 0.0)

    assert state.resolve_response(31, True) is Kind.CONFIRMED
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED
    assert state.pending_poi(31) is None
    assert confirmation.response_event.is_set()


def test_approval_exactly_at_the_deadline_is_refused_and_the_round_stays_open() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(32), SettableClock(START_S)
    _worker, confirmation = _open_round(state, poi, clock)

    clock.now_s = DEADLINE_S

    assert state.resolve_response(32, True) is Kind.DEADLINE_PASSED
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING
    assert state.pending_poi(32) is poi
    # A set event would stop the round's sends and strand it in CONFIRMING.
    assert not confirmation.response_event.is_set()


@pytest.mark.parametrize(
    "policy_status",
    [ConfirmationStatus.TIMEOUT_REJECTED, ConfirmationStatus.CONFIRMED],
)
def test_late_approval_leaves_the_timeout_outcome_to_the_worker(
    policy_status: ConfirmationStatus,
) -> None:
    """The reproduced case: built at 1 s, answered at 1001 s with the worker late."""
    state, poi, clock = ConfirmationManagerState(), _poi(33), SettableClock(START_S)
    ref = ConfirmationRequestRef(9001, 33)
    _worker, confirmation = _open_round(state, poi, clock, request_ref=ref)

    clock.now_s = 1001.0
    assert state.resolve_response(33, True, ref) is Kind.DEADLINE_PASSED

    assert state.complete_round(confirmation, policy_status)
    assert state.registry.get_status(poi) is policy_status
    assert state.pending_poi(33) is None
    if policy_status is ConfirmationStatus.CONFIRMED:
        # The automatic confirm is still the operator's to recall.
        assert state.resolve_response(33, False, ref) is Kind.CANCELLATION_REQUESTED
        assert state.registry.get_status(poi) is ConfirmationStatus.REJECTED


def test_every_repeat_of_a_late_approval_is_refused() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(34), SettableClock(START_S)
    ref = ConfirmationRequestRef(9001, 34)
    _open_round(state, poi, clock, request_ref=ref)
    clock.now_s = DEADLINE_S + 5.0

    assert state.resolve_response(34, True, ref) is Kind.DEADLINE_PASSED
    assert state.resolve_response(34, True, ref) is Kind.DEADLINE_PASSED
    assert state.resolve_response(34, True) is Kind.DEADLINE_PASSED  # no-token sender
    # Another round's token is still told apart from a late one.
    assert state.resolve_response(34, True, ConfirmationRequestRef(9001, 99)) is Kind.LATE_OR_DUPLICATE

    assert state.pending_poi(34) is poi
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING


@pytest.mark.parametrize("now_s", [START_S, DEADLINE_S - 1.0, DEADLINE_S, DEADLINE_S + 4000.0])
def test_reject_is_accepted_before_at_and_after_the_deadline(now_s: float) -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(35), SettableClock(START_S)
    _worker, confirmation = _open_round(state, poi, clock)
    clock.now_s = now_s
    reads = clock.reads

    assert state.resolve_response(35, False) is Kind.REJECTED

    assert state.registry.get_status(poi) is ConfirmationStatus.REJECTED
    assert clock.reads == reads, "a reject does not depend on the clock"
    # The worker's late timeout cannot overwrite the operator's reject.
    assert not state.complete_round(confirmation, ConfirmationStatus.CONFIRMED)
    assert state.registry.get_status(poi) is ConfirmationStatus.REJECTED


def test_a_clock_that_steps_back_does_not_reopen_the_round() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(36), SettableClock(START_S)
    _open_round(state, poi, clock)
    clock.now_s = DEADLINE_S
    assert state.resolve_response(36, True) is Kind.DEADLINE_PASSED

    clock.now_s = START_S

    assert state.resolve_response(36, True) is Kind.DEADLINE_PASSED
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING


@pytest.mark.parametrize("wait_s", [0.0, -3.0])
def test_zero_or_negative_wait_refuses_approval_and_accepts_reject(wait_s: float) -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(37), SettableClock(START_S)
    _worker, confirmation = _open_round(state, poi, clock, wait_s=wait_s)
    clock.now_s = START_S - 50.0  # even with the clock behind the start

    assert state.resolve_response(37, True) is Kind.DEADLINE_PASSED
    assert state.pending_poi(37) is poi
    assert state.resolve_response(37, False) is Kind.REJECTED
    assert not state.complete_round(confirmation, ConfirmationStatus.CONFIRMED)


def test_unmeasurable_window_refuses_approval_and_infinite_wait_accepts_it() -> None:
    state = ConfirmationManagerState()
    nan_wait, nan_clock, forever = _poi(41), _poi(42), _poi(43)
    _open_round(state, nan_wait, SettableClock(START_S), wait_s=math.nan)
    _open_round(state, nan_clock, SettableClock(math.nan))
    late_clock = SettableClock(START_S)
    _open_round(state, forever, late_clock, wait_s=math.inf)
    late_clock.now_s = 1e9

    assert state.resolve_response(41, True) is Kind.DEADLINE_PASSED
    assert state.resolve_response(42, True) is Kind.DEADLINE_PASSED
    assert state.resolve_response(43, True) is Kind.CONFIRMED


def test_a_round_cannot_exist_without_a_deadline() -> None:
    state, poi = ConfirmationManagerState(), _poi(44)
    worker = state.start_review(poi, threading.Event)

    with pytest.raises(TypeError):
        state.begin_round(poi, worker, threading.Event)  # type: ignore[call-arg]

    field = {f.name: f for f in dataclasses.fields(ConfirmationRound)}["deadline"]
    assert field.default is dataclasses.MISSING
    assert field.default_factory is dataclasses.MISSING


def test_refused_round_is_dropped_by_reset_and_the_next_review_has_its_own_window() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(51), SettableClock(START_S)
    _worker, old_round = _open_round(state, poi, clock)
    clock.now_s = DEADLINE_S
    assert state.resolve_response(51, True) is Kind.DEADLINE_PASSED

    state.reset()

    assert state.pending_poi(51) is None
    assert old_round.response_event.is_set()
    assert state.registry.get_status(poi) is None
    assert not state.complete_round(old_round, ConfirmationStatus.CONFIRMED)

    # The old round's expiry does not carry over: the clock is already past it.
    new_ref = ConfirmationRequestRef(9001, 2)
    _new_worker, new_round = _open_round(state, poi, clock, request_ref=new_ref)
    clock.now_s = DEADLINE_S + WAIT_S - 1.0
    assert state.resolve_response(51, True, new_ref) is Kind.CONFIRMED
    assert new_round.response_event.is_set()


def test_refused_round_is_dropped_when_the_acquisition_gates_withdraw_the_review() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(52), SettableClock(START_S)
    state.acquisition.publish(52, True)
    _worker, confirmation = _open_round(state, poi, clock)
    clock.now_s = DEADLINE_S
    assert state.resolve_response(52, True) is Kind.DEADLINE_PASSED

    assert state.acquisition.publish(52, False)

    assert state.pending_poi(52) is None
    assert confirmation.response_event.is_set()
    assert state.registry.get_status(poi) is None
    assert not state.complete_round(confirmation, ConfirmationStatus.CONFIRMED)
    assert state.resolve_response(52, True) is Kind.LATE_OR_DUPLICATE


def test_refused_round_is_dropped_when_its_worker_finishes() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(53), SettableClock(START_S)
    worker, confirmation = _open_round(state, poi, clock)
    clock.now_s = DEADLINE_S
    assert state.resolve_response(53, True) is Kind.DEADLINE_PASSED

    state.finish_worker(worker)

    assert state.pending_poi(53) is None
    assert confirmation.response_event.is_set()
    assert state.active_worker_count == 0
    assert state.resolve_response(53, True) is Kind.LATE_OR_DUPLICATE


def test_replacement_after_a_refusal_owns_the_poi_with_a_fresh_window() -> None:
    state, poi, clock = ConfirmationManagerState(), _poi(54), SettableClock(START_S)
    old_ref, new_ref = ConfirmationRequestRef(9001, 1), ConfirmationRequestRef(9001, 2)
    _old_worker, old_round = _open_round(state, poi, clock, request_ref=old_ref)
    clock.now_s = DEADLINE_S
    assert state.resolve_response(54, True, old_ref) is Kind.DEADLINE_PASSED

    # A second review of the same POI starts with the clock at the old deadline.
    _new_worker, new_round = _open_round(state, poi, clock, request_ref=new_ref)

    assert old_round.response_event.is_set()
    assert not new_round.response_event.is_set()
    # The old worker can neither write its timeout nor retire the new round.
    assert not state.complete_round(old_round, ConfirmationStatus.CONFIRMED)
    assert not state.retire_round(old_round)
    assert state.pending_poi(54) is poi
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING

    clock.now_s = DEADLINE_S + WAIT_S - 1.0  # inside the new window only
    assert state.resolve_response(54, True, old_ref) is Kind.LATE_OR_DUPLICATE
    assert state.resolve_response(54, True) is Kind.LATE_OR_DUPLICATE  # no-token input is closed
    assert state.pending_poi(54) is poi
    assert state.resolve_response(54, True, new_ref) is Kind.CONFIRMED
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


def test_approval_queued_behind_a_send_is_judged_when_it_is_resolved() -> None:
    """The approval arrives in time but waits on the round's send lock past the
    deadline: the clock is read inside the transaction, so it is refused."""
    state, poi, clock = ConfirmationManagerState(), _poi(55), SettableClock(START_S)
    _worker, confirmation = _open_round(state, poi, clock)
    send_entered = threading.Event()
    release_send = threading.Event()

    def slow_send() -> None:
        send_entered.set()
        if not release_send.wait(5.0):
            raise TimeoutError("test did not release the send")

    sender = threading.Thread(
        target=state.send_if_current,
        args=(confirmation, slow_send),
    )
    sender.start()
    assert send_entered.wait(2.0)

    results: list[ConfirmationResponseKind] = []
    resolver = threading.Thread(
        target=lambda: results.append(state.resolve_response(55, True)),
    )
    clock.now_s = DEADLINE_S - 1.0  # in time when it arrives
    resolver.start()
    resolver.join(timeout=0.2)
    assert resolver.is_alive(), "the response must wait for the send in flight"

    clock.now_s = DEADLINE_S + 1.0
    release_send.set()
    resolver.join(timeout=2.0)
    sender.join(timeout=2.0)

    assert not resolver.is_alive() and not sender.is_alive()
    assert results == [Kind.DEADLINE_PASSED]
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING
    assert state.pending_poi(55) is poi
