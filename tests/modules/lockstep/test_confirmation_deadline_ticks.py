"""The companion's confirmation deadline on the lockstep baton.

The real round runner and state run as managed threads on a BatonRuntime:
its clock is the tick time and its locks and events park on the baton. An
operator answer is an input of one tick, taken by an inbound thread. A tick
runs inputs before due timers by default, and timers first with the
``timers_first`` scheduling model, so the deadline tick is where the answer
and the worker's own timeout meet. Whatever the order, an approval counts
only on a tick whose time is below the deadline.

The deadline is the product's float sum, start + wait, the same value the
baton uses for the worker's timed wait. For some start ticks that sum is one
float step above the tick that lies exactly ``wait`` later, and that tick is
then still inside the window for the worker and the resolver alike (the last
test pins it).
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass, field
from typing import Optional
from unittest.mock import Mock, patch

import pytest

from navpy.core.review import confirmation_manager_state as manager_state_module
from navpy.core.review import confirmation_round_transaction as transaction_module
from navpy.core.review.confirmation_manager_state import ConfirmationManagerState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.review.confirmation_round_runner import ConfirmationRoundRunner
from navpy.core.review.confirmation_round_transaction import ConfirmationResponseKind
from navpy.sim.lockstep.baton import BatonRuntime, check_no_deadlock
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import sleep
from navpy.sim.lockstep.baton_threads import LockstepThreads
from tests.detection_factory import make_detected_poi
from tests.runtime_overlay import RuntimeOverlay

Kind = ConfirmationResponseKind
TICK_US = 20_000
WAIT_S = 0.1          # the round starts at tick 0, so its deadline is tick 5
DEADLINE_TICK = 5
LAST_TICK = 12
POI_ID = 71
TIMEOUT_LABEL = f"POI {POI_ID} auto-rejected (timeout) after waiting {WAIT_S}s."


@dataclass
class Outcome:
    answers: list[tuple[int, ConfirmationResponseKind]] = field(default_factory=list)
    status: Optional[ConfirmationStatus] = None
    round_ended_us: Optional[int] = None
    info_logs: list[str] = field(default_factory=list)


def _run(
    *,
    approve_at_tick: int,
    timers_first: bool = False,
    host_delays: bool = False,
    first_send_s: float = 0.0,
    start_tick: int = 0,
) -> Outcome:
    runtime = BatonRuntime()
    runtime.runnable.scheduling.timers_first = timers_first
    threads = LockstepThreads(runtime)
    overlay = RuntimeOverlay(
        Lock=threads.Lock,
        RLock=threads.RLock,
        Event=threads.Event,
        monotonic=runtime.monotonic,
    )
    outcome = Outcome()
    inbox = BatonQueue(runtime)
    poi = make_detected_poi(obj_id=POI_ID, task_id=POI_ID, class_id=0)
    jitter = random.Random(7)

    def host_delay() -> None:
        if host_delays:
            time.sleep(jitter.random() / 500)  # wall time inside the baton: not sim time

    def broadcast(_request: object) -> None:
        host_delay()
        if first_send_s > 0.0 and network.broadcast.call_count == 1:
            sleep(runtime, first_send_s)  # parked across ticks, holding the round's send lock

    network = Mock()
    network.broadcast.side_effect = broadcast
    failure_policy = Mock()
    failure_policy.status.return_value = ConfirmationStatus.TIMEOUT_REJECTED
    media = Mock()
    media.send.return_value = True
    logger = Mock()

    with patch.object(transaction_module, "rt", overlay), \
            patch.object(manager_state_module, "rt", overlay):
        state = ConfirmationManagerState()
        runner = ConfirmationRoundRunner(
            sys_id=1,
            is_simulation=False,
            state=state,
            network=lambda: network,
            wait_time_s=lambda: WAIT_S,
            resend_interval_s=lambda: 1.0,
            failure_policy=failure_policy,
            media=media,
            logger=logger,
            monotonic=runtime.monotonic,
            event_factory=threads.Event,
        )

        def worker() -> None:
            if start_tick:
                sleep(runtime, start_tick * TICK_US / 1_000_000)
            lease = state.start_review(poi, threads.Event)
            runner.run(poi, lease)
            outcome.round_ended_us = runtime.now_us
            state.finish_worker(lease)

        def inbound() -> None:
            while True:
                answer = inbox.get()
                if answer is None:
                    outcome.status = state.registry.get_status(poi)
                    return
                host_delay()
                result = state.resolve_response(POI_ID, answer)  # may park behind a send
                outcome.answers.append((runtime.now_us, result))

        runtime.spawn(worker, name="confirm-worker")
        runtime.spawn(inbound, name="inbound")
        runtime.run_until_quiescent()  # tick 0: the round is built and its request sent
        for k in range(1, LAST_TICK + 1):
            inputs: list[Optional[bool]] = []
            if k == approve_at_tick:
                inputs.append(True)
            if k == LAST_TICK:
                inputs.append(None)
            runtime.tick(k * TICK_US, lambda inputs=inputs: [inbox.deliver(item) for item in inputs])
        check_no_deadlock(runtime)

    outcome.info_logs = [str(entry.args[0]) for entry in logger.info.call_args_list]
    return outcome


def _us(tick: int) -> int:
    return tick * TICK_US


@pytest.mark.parametrize("timers_first", [False, True])
@pytest.mark.parametrize("host_delays", [False, True])
def test_approval_on_the_tick_before_the_deadline_confirms(
    timers_first: bool,
    host_delays: bool,
) -> None:
    outcome = _run(
        approve_at_tick=DEADLINE_TICK - 1,
        timers_first=timers_first,
        host_delays=host_delays,
    )

    assert outcome.answers == [(_us(DEADLINE_TICK - 1), Kind.CONFIRMED)]
    assert outcome.status is ConfirmationStatus.CONFIRMED
    assert outcome.round_ended_us == _us(DEADLINE_TICK - 1)
    assert TIMEOUT_LABEL not in outcome.info_logs


@pytest.mark.parametrize(
    ("timers_first", "answer"),
    [
        # Inputs first: the answer meets the open round and is refused by the deadline.
        (False, Kind.DEADLINE_PASSED),
        # Timers first: the worker's timeout has already closed the round.
        (True, Kind.LATE_OR_DUPLICATE),
    ],
)
@pytest.mark.parametrize("host_delays", [False, True])
def test_approval_on_the_deadline_tick_loses_to_the_timeout_in_either_order(
    timers_first: bool,
    answer: ConfirmationResponseKind,
    host_delays: bool,
) -> None:
    outcome = _run(
        approve_at_tick=DEADLINE_TICK,
        timers_first=timers_first,
        host_delays=host_delays,
    )

    assert outcome.answers == [(_us(DEADLINE_TICK), answer)]
    assert outcome.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert outcome.round_ended_us == _us(DEADLINE_TICK)
    assert TIMEOUT_LABEL in outcome.info_logs


@pytest.mark.parametrize("timers_first", [False, True])
@pytest.mark.parametrize("host_delays", [False, True])
def test_approval_on_the_tick_after_the_deadline_is_late(
    timers_first: bool,
    host_delays: bool,
) -> None:
    outcome = _run(
        approve_at_tick=DEADLINE_TICK + 1,
        timers_first=timers_first,
        host_delays=host_delays,
    )

    assert outcome.answers == [(_us(DEADLINE_TICK + 1), Kind.LATE_OR_DUPLICATE)]
    assert outcome.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert outcome.round_ended_us == _us(DEADLINE_TICK)
    assert TIMEOUT_LABEL in outcome.info_logs


@pytest.mark.parametrize("timers_first", [False, True])
def test_a_first_send_that_parks_across_ticks_does_not_move_the_deadline(
    timers_first: bool,
) -> None:
    """The first send takes 60 ms of sim time. The window still ends at tick 5
    (100 ms after the round was built), not 100 ms after the send returned."""
    outcome = _run(approve_at_tick=DEADLINE_TICK + 1, timers_first=timers_first, first_send_s=0.06)

    assert outcome.answers == [(_us(DEADLINE_TICK + 1), Kind.LATE_OR_DUPLICATE)]
    assert outcome.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert outcome.round_ended_us == _us(DEADLINE_TICK)
    assert TIMEOUT_LABEL in outcome.info_logs


@pytest.mark.parametrize("timers_first", [False, True])
def test_approval_held_behind_the_first_send_is_judged_when_it_resolves_in_time(
    timers_first: bool,
) -> None:
    """Delivered at tick 2 while the send holds the round; resolved at tick 3 (60 ms)."""
    outcome = _run(approve_at_tick=2, timers_first=timers_first, first_send_s=0.06)

    assert outcome.answers == [(_us(3), Kind.CONFIRMED)]
    assert outcome.status is ConfirmationStatus.CONFIRMED
    assert TIMEOUT_LABEL not in outcome.info_logs


@pytest.mark.parametrize("timers_first", [False, True])
def test_approval_held_behind_a_send_past_the_deadline_is_refused(
    timers_first: bool,
) -> None:
    """Delivered at tick 4, in time, but the send holds the round until tick 7:
    the clock is read when the answer is resolved, so it is refused and the
    worker then writes its timeout."""
    outcome = _run(approve_at_tick=DEADLINE_TICK - 1, timers_first=timers_first, first_send_s=0.14)

    assert outcome.answers == [(_us(7), Kind.DEADLINE_PASSED)]
    assert outcome.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert outcome.round_ended_us == _us(7)
    assert TIMEOUT_LABEL in outcome.info_logs


@pytest.mark.parametrize("timers_first", [False, True])
def test_the_deadline_is_the_float_sum_for_the_worker_and_the_resolver_alike(
    timers_first: bool,
) -> None:
    """A round built at tick 1 (0.02 s) with a 0.1 s wait has the deadline
    0.02 + 0.1 = 0.12000000000000001, one float step above tick 6 (0.12 s).
    Tick 6 is therefore inside the window: an approval there confirms, and an
    unanswered round times out on tick 7. Worker and resolver never disagree."""
    assert 0.02 + WAIT_S > 6 * TICK_US / 1_000_000

    on_tick_6 = _run(approve_at_tick=6, timers_first=timers_first, start_tick=1)
    assert on_tick_6.answers == [(_us(6), Kind.CONFIRMED)]
    assert on_tick_6.status is ConfirmationStatus.CONFIRMED

    on_tick_7 = _run(approve_at_tick=7, timers_first=timers_first, start_tick=1)
    assert on_tick_7.answers[0][1] in (Kind.DEADLINE_PASSED, Kind.LATE_OR_DUPLICATE)
    assert on_tick_7.status is ConfirmationStatus.TIMEOUT_REJECTED
    assert on_tick_7.round_ended_us == _us(7)
    assert TIMEOUT_LABEL in on_tick_7.info_logs
