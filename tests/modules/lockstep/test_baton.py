"""The scheduling contract of the lockstep baton runtime."""

from __future__ import annotations

from collections.abc import Callable
from queue import Empty
import random
import threading
import time

import pytest

from navpy.sim.lockstep.baton import (
    MAX_TIMER_ROUNDS_PER_TICK,
    BatonRuntime,
    LockstepFault,
    audit,
    check_no_deadlock,
)
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import (
    BatonCondition,
    BatonEvent,
    BatonLock,
    join,
    sleep,
    yield_now,
)

TICK_US = 20_000


def _run_ticks(runtime: BatonRuntime, ticks: int, inputs_at=None) -> None:
    runtime.run_until_quiescent()  # tick 0: start every thread
    for k in range(1, ticks + 1):
        tick(runtime, k * TICK_US, (inputs_at or {}).get(k, ()))


def tick(runtime: BatonRuntime, now_us: int, inputs=()) -> None:
    """One coordinator tick: time, inputs to quiescence, then timers."""
    def deliver() -> None:
        for queue, item in inputs:
            queue.deliver(item)

    runtime.tick(now_us, deliver)


def _trace_run(jitter: bool) -> list[tuple[str, int]]:
    runtime = BatonRuntime()
    trace: list[tuple[str, int]] = []
    lock = BatonLock(runtime)

    def worker(name: str, period_s: float) -> None:
        for _ in range(5):
            with lock:
                if jitter:
                    time.sleep(random.random() / 2000)  # host delay inside the baton
                trace.append((name, runtime.now_us))
            sleep(runtime, period_s)

    for index, period in enumerate((0.02, 0.04, 0.03)):
        runtime.spawn(lambda n=f"w{index}", p=period: worker(n, p), name=f"w{index}")
    _run_ticks(runtime, 12)
    return trace


def test_order_is_deterministic_under_host_delays():
    reference = _trace_run(jitter=False)
    assert _trace_run(jitter=True) == reference
    assert reference[:3] == [("w0", 0), ("w1", 0), ("w2", 0)]


def test_a_timed_wait_fires_at_the_first_tick_at_or_after_its_due_time():
    runtime = BatonRuntime()
    woke_at = []

    def waiter() -> None:
        sleep(runtime, 0.025)  # due at 25 ms: between ticks 1 and 2
        woke_at.append(runtime.now_us)

    runtime.spawn(waiter, name="waiter")
    _run_ticks(runtime, 3)
    assert woke_at == [2 * TICK_US]


def test_an_input_at_the_deadline_tick_wins_over_the_timeout():
    for arrival_tick, expected in ((2, "input"), (3, "timeout")):
        runtime = BatonRuntime()
        inbox = BatonQueue(runtime)
        outcome = []

        def receiver() -> None:
            try:
                outcome.append(inbox.get(timeout=0.04))  # deadline = tick 2
            except Empty:
                outcome.append("timeout")

        runtime.spawn(receiver, name="receiver")
        _run_ticks(runtime, 3, {arrival_tick: [(inbox, "input")]})
        assert outcome == [expected], arrival_tick


def test_lock_ownership_hands_off_first_in_first_out():
    runtime = BatonRuntime()
    lock = BatonLock(runtime)
    order = []

    def holder() -> None:
        with lock:
            sleep(runtime, 0.02)  # holding across a tick parks the others
            order.append("holder")

    def contender(name: str) -> None:
        with lock:
            order.append(name)

    runtime.spawn(holder, name="holder")
    runtime.spawn(lambda: contender("a"), name="a")
    runtime.spawn(lambda: contender("b"), name="b")
    _run_ticks(runtime, 2)
    assert order == ["holder", "a", "b"]


def test_condition_and_event_wake_in_order_and_join_waits():
    runtime = BatonRuntime()
    condition = BatonCondition(runtime)
    event = BatonEvent(runtime)
    seen = []

    def waiter(name: str) -> None:
        with condition:
            condition.wait()
            seen.append(name)
        event.wait()
        seen.append(name + "-event")

    first = runtime.spawn(lambda: waiter("x"), name="x")
    runtime.spawn(lambda: waiter("y"), name="y")

    def notifier() -> None:
        sleep(runtime, 0.02)
        with condition:
            condition.notify_all()
        event.set()
        join(runtime, first)
        seen.append("joined")

    runtime.spawn(notifier, name="notifier")
    _run_ticks(runtime, 2)
    # x runs first and finds the event already set; then y.
    assert seen == ["x", "x-event", "y", "y-event", "joined"]


def test_misuse_is_a_fault_not_an_outcome():
    runtime = BatonRuntime()
    lock = BatonLock(runtime)
    with pytest.raises(LockstepFault, match="off the baton"):
        lock.acquire()
    # The fault is latched: the run is over even if a caller swallowed it.
    with pytest.raises(LockstepFault, match="off the baton"):
        runtime.tick(TICK_US)

    runtime = BatonRuntime()
    lock = BatonLock(runtime)

    def twice() -> None:
        lock.acquire()
        lock.acquire()

    runtime.spawn(twice, name="twice")
    with pytest.raises(LockstepFault, match="re-acquired"):
        runtime.run_until_quiescent()


def test_the_thread_audit_catches_an_unmanaged_thread():
    runtime = BatonRuntime()
    baseline = set(threading.enumerate())
    stop = threading.Event()
    stray = threading.Thread(target=stop.wait, name="stray", daemon=True)
    stray.start()
    try:
        with pytest.raises(LockstepFault, match="stray"):
            audit(runtime, baseline)
    finally:
        stop.set()
        stray.join()
    audit(runtime, baseline)


def test_a_plain_sleep_never_wakes_early_and_float_drift_waits_a_tick():
    runtime = BatonRuntime()
    woke = {}

    def sleeper() -> None:
        sleep(runtime, 0.0200004)  # just over one tick: due at tick 2
        woke["sleep"] = runtime.now_us
        deadline = 0.2 + 0.1  # 0.30000000000000004, as product code computes it
        while (remaining := deadline - runtime.monotonic()) > 0:
            sleep(runtime, remaining)
        woke["drift"] = runtime.now_us

    runtime.spawn(sleeper, name="sleeper")
    _run_ticks(runtime, 20)
    assert woke == {"sleep": 2 * TICK_US, "drift": 16 * TICK_US}


def test_an_event_set_then_cleared_still_releases_its_waiter():
    runtime = BatonRuntime()
    event = BatonEvent(runtime)
    result = []
    runtime.spawn(lambda: result.append(event.wait(1.0)), name="waiter")

    def pulse() -> None:
        event.set()
        event.clear()

    runtime.spawn(pulse, name="pulse")
    _run_ticks(runtime, 1)
    assert result == [True]


def test_queue_contention_does_not_restart_the_timeout():
    runtime = BatonRuntime()
    queue, go = BatonQueue(runtime), BatonQueue(runtime)
    outcome = {}

    def slow_getter() -> None:
        try:
            queue.get(timeout=0.04)  # deadline tick 2, whatever happens before
        except Empty:
            outcome["slow"] = runtime.now_us

    def thief() -> None:
        go.get()
        outcome["thief"] = queue.get()

    runtime.spawn(slow_getter, name="slow")
    runtime.spawn(thief, name="thief")
    # At tick 1 "go" wakes the thief, then the item wakes the slow getter;
    # the thief runs first and takes the item, and the slow getter parks
    # again for only what is left of its timeout.
    _run_ticks(runtime, 4, {1: [(go, "go"), (queue, "item")]})
    assert outcome == {"thief": "item", "slow": 2 * TICK_US}


def test_only_the_coordinator_schedules_and_scheduling_cannot_nest():
    runtime = BatonRuntime()
    errors = []
    other = threading.Thread(target=lambda: errors.append(_try(runtime.run_until_quiescent)))
    other.start()
    other.join()
    assert "only the coordinator" in str(errors[0])

    runtime.spawn(runtime.run_until_quiescent, name="nested")
    with pytest.raises(LockstepFault, match="only the coordinator"):
        runtime.run_until_quiescent()


def _try(action):
    try:
        action()
    except LockstepFault as error:
        return error
    return None


def test_a_finished_thread_is_reaped_before_the_audit():
    runtime = BatonRuntime()
    baseline = set(threading.enumerate())
    runtime.spawn(lambda: None, name="short")
    runtime.run_until_quiescent()
    audit(runtime, baseline)
    assert not [t for t in threading.enumerate() if t.name == "lockstep-short"]


def test_self_join_and_a_lock_cycle_are_faults():
    runtime = BatonRuntime()
    holder = {}

    def selfish() -> None:
        join(runtime, holder["me"])

    holder["me"] = runtime.spawn(selfish, name="selfish")
    with pytest.raises(LockstepFault, match="joined itself"):
        runtime.run_until_quiescent()

    runtime = BatonRuntime()
    first, second = BatonLock(runtime), BatonLock(runtime)

    def one() -> None:
        with first:
            sleep(runtime, 0.02)
            second.acquire()

    def two() -> None:
        with second:
            sleep(runtime, 0.02)
            first.acquire()

    runtime.spawn(one, name="one")
    runtime.spawn(two, name="two")
    _run_ticks(runtime, 2)
    with pytest.raises(LockstepFault, match="deadlocked"):
        check_no_deadlock(runtime)
    runtime.abort()
    assert runtime.threads.alive() == []


def test_a_swallowed_fault_still_fails_the_tick():
    runtime = BatonRuntime()
    lock = BatonLock(runtime)

    def swallow() -> None:
        lock.acquire()
        try:
            lock.acquire()  # plain lock re-acquired: a fault
        except LockstepFault:
            pass

    runtime.spawn(swallow, name="swallow")
    with pytest.raises(LockstepFault, match="re-acquired"):
        runtime.run_until_quiescent()
    runtime.abort()


def test_after_a_wall_budget_breach_the_runtime_refuses_to_continue(monkeypatch):
    from navpy.sim.lockstep import baton

    monkeypatch.setattr(baton, "BATON_WALL_BUDGET_S", 0.2)
    runtime = BatonRuntime()
    host_lock = threading.Lock()
    host_lock.acquire()
    runtime.spawn(host_lock.acquire, name="host-blocked")
    with pytest.raises(LockstepFault, match="kept the baton"):
        runtime.run_until_quiescent()
    with pytest.raises(LockstepFault, match="kept the baton"):
        runtime.tick(TICK_US)
    with pytest.raises(LockstepFault, match="stop by ownership"):
        runtime.abort()
    host_lock.release()


def test_a_condition_needs_its_lock_for_notify_and_wait_for():
    runtime = BatonRuntime()
    condition = BatonCondition(runtime)
    runtime.spawn(condition.notify, name="no-lock")
    with pytest.raises(LockstepFault, match="without holding its lock"):
        runtime.run_until_quiescent()


def test_finished_threads_are_retired():
    runtime = BatonRuntime()
    for index in range(50):
        runtime.spawn(lambda: None, name=f"t{index}")
    runtime.run_until_quiescent()
    assert len(runtime.threads) == 0


def test_abort_unwinds_a_condition_waiter_with_a_contended_recursive_lock():
    runtime = BatonRuntime()
    condition = BatonCondition(runtime)  # recursive lock by default

    def waiter() -> None:
        with condition:
            with condition:  # recursion depth 2
                condition.wait()

    def holder() -> None:
        sleep(runtime, 0.01)
        with condition:
            condition.notify_all()
            sleep(runtime, 1.0)  # keeps the lock: the waiter parks re-acquiring it

    runtime.spawn(waiter, name="waiter")
    runtime.spawn(holder, name="holder")
    _run_ticks(runtime, 2)
    runtime.abort()
    assert runtime.threads.alive() == []


def test_abort_unwinds_a_rollback_that_parks_again_and_groups_the_aborts():
    """Construction rollback: cleanup joins the reader while unwinding, the
    reader's exit wakes it into the abort again, and the product groups both.
    That is an abort, not a failure; anything else in the group is a failure."""

    def run(last_error: BaseException | None) -> BatonRuntime:
        runtime = BatonRuntime()

        def construction() -> None:
            reader = runtime.spawn(lambda: sleep(runtime, 60.0), name="reader")
            errors: list[BaseException] = []
            try:
                sleep(runtime, 30.0)  # waiting for a heartbeat
            except BaseException as error:
                errors.append(error)
            try:
                join(runtime, reader, 2.0)  # rollback joins the reader
            except BaseException as error:
                errors.append(error)
            raise BaseExceptionGroup("rollback", errors + [last_error] * (last_error is not None))

        runtime.spawn(construction, name="construction")
        _run_ticks(runtime, 2)
        return runtime

    runtime = run(None)
    runtime.abort()
    assert runtime.threads.alive() == []
    with pytest.raises(BaseExceptionGroup):
        run(RuntimeError("not an abort")).abort()


def test_yield_now_requeues_first_in_first_out_at_the_same_time_without_a_timer():
    runtime = BatonRuntime()
    order: list[tuple[str, int, int]] = []

    def worker(name: str, yields: int) -> Callable[[], None]:
        def run() -> None:
            for index in range(yields):
                order.append((name, index, runtime.now_us))
                yield_now(runtime)
        return run

    runtime.spawn(worker("a", 3), name="a")
    runtime.spawn(worker("b", 2), name="b")
    # More yields than timer rounds a tick allows: none of them is a timer round.
    runtime.spawn(worker("c", MAX_TIMER_ROUNDS_PER_TICK + 1), name="c")
    runtime.tick(TICK_US)
    assert runtime.threads.alive() == []
    assert not runtime.timers.any_due(float("inf"))
    assert order[:6] == [
        ("a", 0, TICK_US), ("b", 0, TICK_US), ("c", 0, TICK_US),
        ("a", 1, TICK_US), ("b", 1, TICK_US), ("c", 1, TICK_US),
    ]
    assert {now_us for _name, _index, now_us in order} == {TICK_US}


def test_a_tick_takes_at_most_its_handoffs_and_due_timers_wait_for_one_left():
    """A capped tick (step 4: a CPU budget, or 0 in a process freeze) leaves the rest runnable."""
    runtime = BatonRuntime()
    inbox = BatonQueue(runtime)
    order: list[tuple[str, int]] = []

    def reader() -> None:
        while True:
            order.append((inbox.get(), runtime.now_us))

    def sleeper() -> None:
        sleep(runtime, 0.01)
        order.append(("timer", runtime.now_us))

    runtime.spawn(reader, name="reader")
    runtime.spawn(sleeper, name="sleeper")
    taken = [runtime.tick(TICK_US, handoffs=1),  # the reader starts; the sleeper waits
             runtime.tick(2 * TICK_US, lambda: inbox.deliver("x")),  # the sleeper first, then the read
             runtime.tick(3 * TICK_US, handoffs=0)]  # frozen: its timer (due 50 ms) stays due
    assert taken == [1, 2, 0] and order == [("x", 2 * TICK_US)]
    assert runtime.timers.any_due(runtime.monotonic())
    assert runtime.tick(4 * TICK_US, handoffs=5) == 1 and order[-1] == ("timer", 4 * TICK_US)
