from __future__ import annotations

import threading
import time
from unittest.mock import Mock

import pytest

from navpy.modules.nav.nav_application import NavApplication
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.navigation_loop import NavigationLoop, StateActionDispatcher
from navpy.modules.nav.navigation_transitions import TransitionOutcome


class FatalNavigationFailure(BaseException):
    """Fatal worker failure outside the operational OSError retry contract."""


@pytest.fixture
def action_map() -> dict[NavState, Mock]:
    return {state: Mock(name=state.name) for state in NavState}


@pytest.mark.parametrize("state", list(NavState))
def test_state_action_dispatcher_dispatches_every_state_from_map(
    state: NavState,
    action_map: dict[NavState, Mock],
) -> None:
    phase = NavPhaseState(current=state, previous=state)
    transitions = Mock()

    StateActionDispatcher(phase, transitions, action_map).act()

    action_map[state].assert_called_once_with()
    for candidate, action in action_map.items():
        assert action.call_count == (candidate is state)
    transitions.on_change.assert_not_called()


def test_state_action_dispatcher_requires_complete_nav_state_map(
    action_map: dict[NavState, Mock],
) -> None:
    del action_map[NavState.RECOVERY]

    with pytest.raises(ValueError, match="RECOVERY"):
        StateActionDispatcher(NavPhaseState(), Mock(), action_map)


def test_nav_redirect_recomputes_dispatch_before_atomic_commit(
    action_map: dict[NavState, Mock],
) -> None:
    phase = NavPhaseState(
        current=NavState.RESET,
        previous=NavState.NAV,
    )
    transitions = Mock()
    transitions.on_change.return_value = TransitionOutcome(
        effective_state=NavState.ONHOLD,
        oneshot_completed=True,
    )

    StateActionDispatcher(phase, transitions, action_map).act()

    transitions.on_change.assert_called_once_with(
        NavState.NAV,
        NavState.RESET,
    )
    assert phase.current is NavState.ONHOLD
    assert phase.previous is NavState.ONHOLD
    assert phase.oneshot_completed
    action_map[NavState.ONHOLD].assert_called_once_with()
    action_map[NavState.RESET].assert_not_called()


def test_navigation_loop_stop_interrupts_long_scheduler_wait() -> None:
    cycle_entered = threading.Event()
    cycle = Mock()
    cycle.run.side_effect = cycle_entered.set
    clock = Mock()
    clock.scheduler_wall_period.return_value = 30.0
    loop = NavigationLoop(cycle, clock, Mock())
    loop.start(loop_rate_hz=0.0)
    assert cycle_entered.wait(1.0)

    started = time.monotonic()
    loop.stop()

    assert time.monotonic() - started < 0.5
    assert not loop.thread.is_alive()


def test_navigation_loop_uses_high_resolution_elapsed_time_and_sleep() -> None:
    class FakeClock:
        now_s = 0.0

        def monotonic(self) -> float:
            return self.now_s

        def sleep(self, duration_s: float) -> None:
            sleeps.append(duration_s)
            self.now_s += duration_s

    fake_clock = FakeClock()
    sleeps: list[float] = []
    cycle = Mock()
    clock = Mock()
    clock.scheduler_wall_period.return_value = 0.004
    loop = NavigationLoop(
        cycle,
        clock,
        Mock(),
        monotonic_s=fake_clock.monotonic,
        sleep_s=fake_clock.sleep,
    )

    def run_cycle() -> None:
        fake_clock.now_s += 0.001
        if cycle.run.call_count == 2:
            loop.stop_event.set()

    cycle.run.side_effect = run_cycle

    loop._run()

    assert sleeps == pytest.approx([0.003])
    assert fake_clock.now_s == pytest.approx(0.005)


def test_navigation_loop_stop_rejects_non_quiescent_thread() -> None:
    loop = NavigationLoop(Mock(), Mock(), Mock())
    stuck_thread = Mock()
    stuck_thread.is_alive.return_value = True
    loop.thread = stuck_thread

    with pytest.raises(RuntimeError, match="failed to terminate"):
        loop.stop()

    assert loop.stop_event.is_set()
    stuck_thread.join.assert_called_once()


def test_navigation_loop_recovers_from_operational_oserror() -> None:
    cycle = Mock()
    cycle_calls = 0
    loop: NavigationLoop

    def run_cycle() -> None:
        nonlocal cycle_calls
        cycle_calls += 1
        if cycle_calls == 1:
            raise OSError("sensor temporarily unavailable")
        loop.stop_event.set()

    cycle.run.side_effect = run_cycle
    clock = Mock()
    clock.scheduler_wall_period.return_value = 0.0
    logger = Mock()
    loop = NavigationLoop(cycle, clock, logger)

    loop._run()

    assert cycle.run.call_count == 2
    logger.error.assert_called_once_with(
        "nav loop error: sensor temporarily unavailable"
    )


def test_navigation_loop_propagates_programmer_typeerror_without_retry() -> None:
    cycle = Mock()
    logger = Mock()
    clock = Mock()
    clock.scheduler_wall_period.return_value = 0.0
    loop = NavigationLoop(cycle, clock, logger)

    def invalid_cycle() -> None:
        loop.stop_event.set()
        raise TypeError("bad cycle input")

    cycle.run.side_effect = invalid_cycle

    with pytest.raises(TypeError, match="bad cycle input"):
        loop._run()

    cycle.run.assert_called_once_with()
    logger.error.assert_not_called()


def test_navigation_loop_retains_fatal_thread_failure_after_clean_stop() -> None:
    cycle_entered = threading.Event()
    cycle = Mock()
    clock = Mock()
    logger = Mock()
    failure = FatalNavigationFailure("navigation worker failed")
    loop = NavigationLoop(cycle, clock, logger)

    def fail_while_stopping() -> None:
        cycle_entered.set()
        assert loop.stop_event.wait(1.0)
        raise failure

    cycle.run.side_effect = fail_while_stopping
    loop.start(loop_rate_hz=0.0)
    assert cycle_entered.wait(1.0)

    loop.stop()

    assert loop.stop_event.is_set()
    assert not loop.thread.is_alive()
    cycle.run.assert_called_once_with()
    logger.error.assert_not_called()
    for _ in range(2):
        with pytest.raises(FatalNavigationFailure) as raised:
            loop.raise_if_failed()
        assert raised.value is failure


def test_navigation_loop_fatal_thread_failure_fences_itself() -> None:
    cycle = Mock()
    failure = FatalNavigationFailure("navigation worker failed")
    cycle.run.side_effect = failure
    loop = NavigationLoop(cycle, Mock(), Mock())

    loop.start(loop_rate_hz=0.0)
    loop.thread.join(timeout=1.0)

    assert not loop.thread.is_alive()
    assert loop.stop_event.is_set()
    with pytest.raises(FatalNavigationFailure) as raised:
        loop.raise_if_failed()
    assert raised.value is failure


def test_nav_application_delegates_all_runtime_failure_health() -> None:
    loop = Mock()
    network = Mock()
    application = NavApplication(loop, network, Mock(), Mock())

    application.raise_if_failed()

    loop.raise_if_failed.assert_called_once_with()
    network.raise_if_failed.assert_called_once_with()
