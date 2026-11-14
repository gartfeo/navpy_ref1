"""ConfirmationDeadline: one round's response window, fixed when built."""

from __future__ import annotations

import math
import threading

import pytest

from navpy.core.review import confirmation_round_transaction as transaction_module
from navpy.core.review.confirmation_round_transaction import ConfirmationDeadline
from tests.confirmation_doubles import SettableClock
from tests.runtime_overlay import RuntimeOverlay


def test_open_until_just_before_the_deadline() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, 30.0)

    assert deadline.remaining_s() == 30.0
    assert not deadline.expired()

    clock.now_s = math.nextafter(40.0, 0.0)
    assert deadline.remaining_s() > 0.0
    assert not deadline.expired()


def test_expired_exactly_at_the_deadline() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, 30.0)

    clock.now_s = 40.0

    assert deadline.expired()
    assert deadline.remaining_s() == 0.0


def test_expired_after_the_deadline() -> None:
    clock = SettableClock(1.0)
    deadline = ConfirmationDeadline(clock, 30.0)

    clock.now_s = 1001.0

    assert deadline.remaining_s() == 0.0
    assert deadline.expired()


def test_expiry_is_one_way_when_the_clock_steps_back() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, 30.0)
    clock.now_s = 40.0
    assert deadline.expired()
    reads = clock.reads

    clock.now_s = 5.0

    assert deadline.expired()
    assert deadline.remaining_s() == 0.0
    assert clock.reads == reads, "an expired deadline must not consult the clock again"


@pytest.mark.parametrize(
    ("start_s", "wait_s"),
    [
        (math.nan, 30.0),   # the clock is not a number
        (1.0, math.nan),    # the wait parameter is not a number
        (1e300, 30.0),      # start + wait rounds back to start
        (math.inf, 30.0),
    ],
)
def test_a_window_that_cannot_be_measured_is_closed_from_the_start(
    start_s: float,
    wait_s: float,
) -> None:
    clock = SettableClock(start_s)
    deadline = ConfirmationDeadline(clock, wait_s)

    clock.now_s = 0.0  # even a later sane reading does not open it

    assert deadline.expired()
    assert deadline.remaining_s() == 0.0
    assert clock.reads == 1


def test_a_clock_that_stops_being_a_number_closes_the_window_for_good() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, 30.0)
    assert not deadline.expired()

    clock.now_s = math.nan
    assert deadline.expired()

    clock.now_s = 11.0
    assert deadline.expired()


@pytest.mark.parametrize("wait_s", [0.0, -0.0, -5.0, -math.inf])
def test_zero_or_negative_wait_is_expired_before_the_first_check(wait_s: float) -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, wait_s)

    # The clock steps back before anything asked: start + wait is now ahead.
    clock.now_s = -100.0

    assert deadline.expired()
    assert deadline.remaining_s() == 0.0
    assert clock.reads == 1


def test_infinite_wait_never_expires() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, math.inf)

    clock.now_s = 1e12

    assert not deadline.expired()
    assert deadline.remaining_s() == math.inf


def test_the_window_cannot_be_changed_after_construction() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, 30.0)

    assert clock.reads == 1, "the start is sampled once, when the round is built"
    assert deadline.wait_s == 30.0
    with pytest.raises(AttributeError):
        deadline.wait_s = 60.0  # type: ignore[misc]
    assert deadline.remaining_s() == 30.0


def test_a_frozen_clock_below_the_deadline_stays_open() -> None:
    clock = SettableClock(10.0)
    deadline = ConfirmationDeadline(clock, 30.0)

    for _ in range(1000):
        assert not deadline.expired()

    assert deadline.remaining_s() == 30.0


def test_every_check_reads_the_clock_while_holding_the_deadline_lock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    locks: list[threading.Lock] = []

    def make_lock() -> threading.Lock:
        locks.append(threading.Lock())
        return locks[-1]

    monkeypatch.setattr(transaction_module, "rt", RuntimeOverlay(Lock=make_lock))
    held_at_read: list[bool] = []

    def clock() -> float:
        held_at_read.append(bool(locks) and locks[0].locked())
        return 10.0

    deadline = ConfirmationDeadline(clock, 30.0)
    assert deadline.remaining_s() == 30.0
    assert not deadline.expired()

    assert len(locks) == 1
    # The start sample needs no lock: nothing else can see the object yet.
    assert held_at_read == [False, True, True]
    assert not locks[0].locked()


def test_expiry_cannot_be_recorded_while_another_thread_is_mid_check() -> None:
    """A check is one step: latch, clock read and update under one lock.

    The resolver thread is paused inside its clock read. Without the lock the
    worker thread would record expiry meanwhile and the resolver would then
    answer "open" from a clock that stepped back, reopening the round.
    """
    resolver_in_clock = threading.Event()
    worker_answered = threading.Event()
    recorded_during_check: list[bool] = []
    resolver = threading.current_thread()
    started = False

    def clock() -> float:
        nonlocal started
        if not started:
            started = True
            return 10.0  # construction
        if threading.current_thread() is resolver:
            resolver_in_clock.set()
            recorded_during_check.append(worker_answered.wait(0.3))
            return 5.0   # a stepped-back reading, below the deadline
        return 50.0      # the worker's reading, past the deadline

    deadline = ConfirmationDeadline(clock, 30.0)
    worker_remaining: list[float] = []

    def worker() -> None:
        assert resolver_in_clock.wait(2.0)
        worker_remaining.append(deadline.remaining_s())
        worker_answered.set()

    worker_thread = threading.Thread(target=worker)
    worker_thread.start()
    resolver_open = not deadline.expired()
    worker_thread.join(timeout=2.0)

    assert not worker_thread.is_alive()
    assert recorded_during_check == [False], "expiry was recorded during another thread's check"
    assert resolver_open, "the resolver's own reading (5 s) was below the deadline"
    assert worker_remaining == [0.0]
    assert deadline.expired()
