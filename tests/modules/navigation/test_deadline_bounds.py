"""Todo 291: a nonsense clock or period never raises or strands a deadline wait."""

from __future__ import annotations

import math
import threading
import time
from contextlib import nullcontext
from unittest.mock import Mock

import pytest

from navpy.core.approach.navigation_command_worker import (
    AUTOPILOT_NAVIGATION_PERIOD_S,
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.core.flow.nav_clock import NavClock
from navpy.core.flow.nav_constants import NAV_LOOP_PERIOD_S
from navpy.core.flow.navigation_loop import NavigationLoop
from navpy.core.kernel.high_resolution_deadline import (
    HIGH_RESOLUTION_TAIL_S,
    MAX_SCHEDULER_WALL_PERIOD_S,
    MAX_WAIT_SLICE_S,
    bounded_scheduler_wall_period_s,
    next_fixed_deadline,
    wait_for_slot,
    wait_until_deadline,
)
from navpy.core.kernel.scheduler_cadence import SchedulerCadence


class FakeClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, duration_s: float) -> None:
        if math.isnan(duration_s):
            raise ValueError("sleep length must be a number")
        self.now_s += duration_s


class FakeStopEvent:
    """Advances the fake clock by each wait and stops after ``limit`` waits."""

    def __init__(self, clock: FakeClock, limit: int = 1_000_000) -> None:
        self._clock = clock
        self._limit = limit
        self.waits: list[float] = []
        self.flag = False

    def is_set(self) -> bool:
        return self.flag

    def set(self) -> None:
        self.flag = True

    def wait(self, timeout: float) -> bool:
        if timeout > threading.TIMEOUT_MAX:
            raise OverflowError("timeout value is too large")
        self.waits.append(timeout)
        # A real clock always moves; a rounding-sized wait must not freeze it.
        self._clock.now_s += max(timeout, 1e-6)
        if len(self.waits) >= self._limit:
            self.flag = True
        return self.flag


def test_huge_finite_deadline_waits_in_bounded_slices_until_stop() -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock, limit=3)

    reached = wait_until_deadline(
        1e300, stop_event=stop, monotonic_s=clock.monotonic, sleep_s=clock.sleep
    )

    assert reached is False
    assert stop.waits == pytest.approx([MAX_WAIT_SLICE_S] * 3)


@pytest.mark.parametrize("deadline_s", [math.inf, math.nan])
def test_non_finite_deadline_returns_without_waiting(deadline_s: float) -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)

    assert wait_until_deadline(
        deadline_s, stop_event=stop, monotonic_s=clock.monotonic, sleep_s=clock.sleep
    )
    assert stop.waits == []
    assert clock.now_s == 0.0


@pytest.mark.parametrize("clock_s", [math.inf, -math.inf, math.nan])
def test_non_finite_clock_reading_returns_without_waiting(clock_s: float) -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)

    assert wait_until_deadline(
        1.0, stop_event=stop, monotonic_s=lambda: clock_s, sleep_s=clock.sleep
    )
    assert stop.waits == []


def test_finite_deadline_still_ends_on_the_high_resolution_tail() -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)

    assert wait_until_deadline(
        2.5, stop_event=stop, monotonic_s=clock.monotonic, sleep_s=clock.sleep
    )
    assert stop.waits == pytest.approx([1.0, 1.0, 0.5 - HIGH_RESOLUTION_TAIL_S])
    assert clock.now_s == pytest.approx(2.5)


def test_deadline_beyond_timeout_max_stays_stop_responsive_on_real_event() -> None:
    stop = threading.Event()
    result: list[bool] = []
    errors: list[BaseException] = []

    def wait() -> None:
        try:
            result.append(wait_until_deadline(
                time.perf_counter() + threading.TIMEOUT_MAX * 4,
                stop_event=stop,
                monotonic_s=time.perf_counter,
                sleep_s=time.sleep,
            ))
        except BaseException as error:  # noqa: BLE001 - the test reports it
            errors.append(error)

    waiter = threading.Thread(target=wait, daemon=True)
    waiter.start()
    time.sleep(0.05)
    started = time.monotonic()
    stop.set()
    waiter.join(timeout=2.0)

    assert errors == []
    assert not waiter.is_alive()
    assert result == [False]
    assert time.monotonic() - started < 1.0


@pytest.mark.parametrize(
    ("period_s", "fallback_s", "expected_s"),
    [
        (0.02, 0.04, 0.02),
        (1e300, 0.04, MAX_SCHEDULER_WALL_PERIOD_S),
        (math.inf, 0.04, MAX_SCHEDULER_WALL_PERIOD_S),
        (math.nan, 0.04, 0.04),
        ("not a number", 0.04, 0.04),
        (None, 0.04, 0.04),
        (math.nan, math.nan, 0.0),
        (math.nan, math.inf, MAX_SCHEDULER_WALL_PERIOD_S),
        (-math.inf, 0.04, 0.04),
        (-1.0, 0.04, 0.04),
        (-1.0, -1.0, 0.0),
        (0.0, 0.04, 0.0),
    ],
)
def test_bounded_scheduler_wall_period(
    period_s: object, fallback_s: float, expected_s: float
) -> None:
    assert bounded_scheduler_wall_period_s(
        period_s, fallback_s=fallback_s
    ) == pytest.approx(expected_s)


def test_max_wall_period_matches_the_speedup_poll_interval() -> None:
    # A slot longer than the poll could not follow a corrected SIM_SPEEDUP.
    assert MAX_SCHEDULER_WALL_PERIOD_S == SchedulerCadence.SPEED_POLL_WALL_S


def _nav_loop_on_fake_clock(clock_source: NavClock, cycles: int):
    clock = FakeClock()
    cycle = Mock()
    cycle_times: list[float] = []
    loop = NavigationLoop(
        cycle, clock_source, Mock(), monotonic_s=clock.monotonic, sleep_s=clock.sleep
    )
    stop = FakeStopEvent(clock)
    loop.stop_event = stop  # type: ignore[assignment]

    def run_cycle() -> None:
        cycle_times.append(clock.now_s)
        if len(cycle_times) == cycles:
            stop.set()

    cycle.run.side_effect = run_cycle
    loop._run()
    return cycle_times, stop


def test_tiny_sim_speedup_cannot_strand_the_nav_loop() -> None:
    cadence = SchedulerCadence(lambda: 1e-300, high_resolution_timer=False)
    try:
        # The cause: period / speedup is a years-long (here infinite) slot.
        assert cadence.wall_period_for_scheduler_period(NAV_LOOP_PERIOD_S) > 1e290

        cycle_times, stop = _nav_loop_on_fake_clock(NavClock(cadence), cycles=3)
    finally:
        cadence.close()

    assert cycle_times == pytest.approx([0.0, 1.0, 2.0])
    assert all(wait <= MAX_WAIT_SLICE_S for wait in stop.waits)


@pytest.mark.parametrize("wall_period_s", [math.inf, math.nan])
def test_non_finite_wall_period_keeps_the_nav_loop_running(
    wall_period_s: float,
) -> None:
    clock_source = Mock()
    clock_source.scheduler_wall_period.return_value = wall_period_s

    cycle_times, _stop = _nav_loop_on_fake_clock(clock_source, cycles=3)

    step_s = MAX_SCHEDULER_WALL_PERIOD_S if math.isinf(wall_period_s) else NAV_LOOP_PERIOD_S
    assert cycle_times == pytest.approx([0.0, step_s, 2 * step_s])


@pytest.mark.parametrize(
    ("loop_rate_hz", "expected_period_s"),
    [
        (1e-300, 1e300),  # bounded per cycle, after the speedup conversion
        (5e-324, NAV_LOOP_PERIOD_S),  # 1 / rate overflows
        (0.5, 2.0),
        (math.nan, NAV_LOOP_PERIOD_S),
        (math.inf, NAV_LOOP_PERIOD_S),
        (-5.0, NAV_LOOP_PERIOD_S),
        (50.0, 0.02),
    ],
)
def test_nav_loop_start_bounds_its_period(
    loop_rate_hz: float, expected_period_s: float
) -> None:
    clock_source = Mock()
    clock_source.scheduler_wall_period.side_effect = lambda period_s: period_s
    loop = NavigationLoop(Mock(), clock_source, Mock())
    try:
        loop.start(loop_rate_hz)
        assert loop.period_s == pytest.approx(expected_period_s)
    finally:
        started = time.monotonic()
        loop.stop()
        assert time.monotonic() - started < 1.0


@pytest.mark.parametrize(
    ("wall_period_s", "expected_step_s"),
    [
        (math.inf, MAX_SCHEDULER_WALL_PERIOD_S),
        (1e300, MAX_SCHEDULER_WALL_PERIOD_S),
        (math.nan, AUTOPILOT_NAVIGATION_PERIOD_S),
    ],
)
def test_command_worker_grid_survives_a_nonsense_wall_period(
    wall_period_s: float, expected_step_s: float
) -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)
    ticks: list[float] = []

    class IdleCommandEvent:
        def wait(self, timeout: float) -> bool:
            ticks.append(clock.now_s)
            if len(ticks) == 3:
                stop.set()
            return False

    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = False
    NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop,  # type: ignore[arg-type]
        command_event=IdleCommandEvent(),  # type: ignore[arg-type]
        wall_period_s=lambda _period_s: wall_period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )).run()

    assert ticks == pytest.approx(
        [expected_step_s, 2 * expected_step_s, 3 * expected_step_s]
    )
    assert all(wait <= MAX_WAIT_SLICE_S for wait in stop.waits)


def _nav_loop_with_clock(clock_source, monotonic_s, cycles: int):
    clock = FakeClock()
    cycle = Mock()
    loop = NavigationLoop(cycle, clock_source, Mock(), monotonic_s=monotonic_s, sleep_s=clock.sleep)
    stop = FakeStopEvent(clock)
    loop.stop_event = stop  # type: ignore[assignment]
    count = [0]

    def run_cycle() -> None:
        count[0] += 1
        if count[0] == cycles:
            stop.set()

    cycle.run.side_effect = run_cycle
    loop._run()
    return count[0], stop


def test_slow_legitimate_rate_is_converted_before_the_bound() -> None:
    cadence = SchedulerCadence(lambda: 10.0, high_resolution_timer=False)
    try:
        loop = NavigationLoop(Mock(), NavClock(cadence), Mock())
        loop.period_s = 2.0  # a 0.5 Hz loop
        clock = FakeClock()
        loop._monotonic_s = clock.monotonic
        loop._sleep_s = clock.sleep
        stop = FakeStopEvent(clock)
        loop.stop_event = stop  # type: ignore[assignment]
        times: list[float] = []

        def run_cycle() -> None:
            times.append(clock.now_s)
            if len(times) == 3:
                stop.set()

        loop._cycle.run.side_effect = run_cycle
        loop._run()
    finally:
        cadence.close()

    assert times == pytest.approx([0.0, 0.2, 0.4])


@pytest.mark.parametrize("clock_s", [math.nan, math.inf, -math.inf, 1e300])
def test_nav_loop_on_a_broken_clock_is_paced_by_its_period(clock_s: float) -> None:
    clock_source = Mock()
    clock_source.scheduler_wall_period.return_value = NAV_LOOP_PERIOD_S

    cycles, stop = _nav_loop_with_clock(clock_source, lambda: clock_s, cycles=4)

    assert cycles == 4
    assert stop.waits == pytest.approx([NAV_LOOP_PERIOD_S] * 3)


def test_slot_deadline_never_lies_more_than_one_period_ahead() -> None:
    clock = FakeClock()
    clock.now_s = 100.0  # the clock stepped back from 1e300
    stop = FakeStopEvent(clock)

    assert wait_for_slot(
        1e300, 0.5, stop_event=stop, monotonic_s=clock.monotonic, sleep_s=clock.sleep
    )
    assert clock.now_s == pytest.approx(100.5)


@pytest.mark.parametrize(
    ("scheduled_s", "completed_s"),
    [
        (1.0, math.nan),
        (math.nan, 1.0),
        (1.0, math.inf),
        (-math.inf, 1.0),
        (0.0, 1e308),
    ],
)
def test_next_fixed_deadline_never_raises_on_a_broken_clock(
    scheduled_s: float, completed_s: float
) -> None:
    deadline_s = next_fixed_deadline(scheduled_s, completed_s, 0.02)

    if math.isfinite(completed_s):
        assert deadline_s == pytest.approx(completed_s + 0.02)
    else:
        assert not math.isfinite(deadline_s)


@pytest.mark.parametrize("clock_s", [math.nan, math.inf, 1e300])
def test_command_worker_on_a_broken_clock_is_paced_by_its_period(
    clock_s: float,
) -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)
    ticks: list[int] = []

    class IdleCommandEvent:
        def wait(self, timeout: float) -> bool:
            ticks.append(len(stop.waits))
            if len(ticks) == 3:
                stop.set()
            return False

    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = False
    NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop,  # type: ignore[arg-type]
        command_event=IdleCommandEvent(),  # type: ignore[arg-type]
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=lambda: clock_s,
        sleep_s=clock.sleep,
    )).run()

    assert ticks == [1, 2, 3]
    assert stop.waits == pytest.approx([AUTOPILOT_NAVIGATION_PERIOD_S] * 3)


def test_clock_stepping_back_mid_wait_cannot_stretch_the_slot() -> None:
    clock = FakeClock()
    clock.now_s = 100.0
    stop = FakeStopEvent(clock)
    readings = iter([100.0, -1e300])
    real_s: list[float] = []

    def monotonic() -> float:
        real_s.append(clock.now_s)
        return next(readings, -1e300)

    assert wait_for_slot(
        100.5, 0.5, stop_event=stop, monotonic_s=monotonic, sleep_s=clock.sleep
    )
    # Wall time spent stays one slot although the clock reads -1e300.
    assert clock.now_s == pytest.approx(100.5)


def test_early_wakeup_still_waits_the_rest_of_the_slot() -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)
    sleeps: list[float] = []

    def early_sleep(duration_s: float) -> None:
        sleeps.append(duration_s)
        clock.now_s += duration_s / 2  # returns early every time

    assert wait_for_slot(
        0.04, 0.04, stop_event=stop, monotonic_s=clock.monotonic, sleep_s=early_sleep
    )
    assert clock.now_s == pytest.approx(0.04, abs=1e-9)
    assert len(sleeps) > 1


def test_nav_loop_with_an_infinite_completion_reading_still_waits() -> None:
    clock_source = Mock()
    clock_source.scheduler_wall_period.return_value = NAV_LOOP_PERIOD_S
    readings = iter([0.0, math.inf] * 100)

    cycles, stop = _nav_loop_with_clock(clock_source, lambda: next(readings), cycles=4)

    assert cycles == 4
    assert len(stop.waits) == 3


def test_frozen_clock_cannot_strand_the_slot() -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)

    assert wait_for_slot(
        5.5, 0.5, stop_event=stop, monotonic_s=lambda: 5.0, sleep_s=clock.sleep
    )
    assert clock.now_s == pytest.approx(0.5, abs=1e-5)


def test_frozen_clock_under_the_sleep_tail_cannot_strand_the_slot() -> None:
    clock = FakeClock()
    stop = FakeStopEvent(clock)

    assert wait_for_slot(
        5.02, 0.02, stop_event=stop, monotonic_s=lambda: 5.0, sleep_s=clock.sleep
    )
    assert stop.waits == []
    assert clock.now_s == pytest.approx(0.04)  # one early return, then the budget
