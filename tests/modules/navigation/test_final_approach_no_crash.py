"""No-crash regressions: final-approach failures end the episode, never NavPy.

Owner ruling 2026-11-05: an approach whose commands went stale must fail
closed and recover; no scenario may crash the companion.
"""

from __future__ import annotations

import threading
from unittest.mock import Mock

import pytest

from navpy.app.flow.nav_runtime_composition import _fail_active_approach
from navpy.core.approach.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.core.approach.vision_nav import nav_fail_reason as fail_reason
from navpy.core.approach.vision_nav.command_executor import (
    FinalApproachCommandExecutor,
    FinalApproachExecutorPorts,
)
from navpy.core.approach.vision_nav.command_freshness import (
    FinalApproachCommandFreshness,
    FinalApproachCommandTiming,
)
from navpy.core.approach.vision_nav.command_liveness import (
    FinalApproachEpisodeFailures,
)
from navpy.core.approach.vision_nav.command_reset import (
    FinalApproachCommandReset,
    FinalApproachCommandResetPorts,
)
from navpy.core.flow.nav_state import NavPhaseState, NavState
from navpy.core.flow.navigation_loop import NavigationLoop


# --- current-command stale (todo 230) ----------------------------------------


def _executor(report_stale) -> tuple[FinalApproachCommandExecutor, Mock]:
    ports = Mock()
    ports.transaction.terminal_result.return_value = None
    executor = FinalApproachCommandExecutor(FinalApproachExecutorPorts(
        slot=ports.slot,
        transaction=ports.transaction,
        mailbox=ports.mailbox,
        diagnostics=ports.diagnostics,
        status=ports.status,
        source_time=ports.source_time,
        hold=ports.hold,
        failures=ports.failures,
        freshness=FinalApproachCommandFreshness(lambda source_s: source_s / 10.0),
        postprocess_fence=ports.postprocess_fence,
        report_stale=report_stale,
    ))
    return executor, ports


STALE_SOURCE = FinalApproachCommandTiming(
    source_timestamp_s=100.0,
    source_now_s=lambda: 101.06,
    receipt_timestamp_s=5.0,
    receipt_now_s=lambda: 5.01,
)


def test_stale_fresh_frame_names_the_failed_bound_and_fails_closed() -> None:
    reported = []
    executor, ports = _executor(reported.append)

    assert executor._execute_current(Mock(), STALE_SOURCE) is None

    assert [check.source.problem for check in reported] == ["too_old"]
    assert "source: too_old age_s=1.0600" in reported[0].describe()
    ports.failures.mark_failed.assert_called_once_with()
    ports.transaction.execute.assert_not_called()


def test_raising_stale_report_still_fails_closed() -> None:
    executor, ports = _executor(Mock(side_effect=RuntimeError("log down")))

    assert executor._execute_current(Mock(), STALE_SOURCE) is None

    ports.failures.mark_failed.assert_called_once_with()
    ports.transaction.execute.assert_not_called()


# --- terminal observer raising ------------------------------------------------


def test_raising_terminal_observer_still_signals_the_failure() -> None:
    episode = Mock()
    episode.latch_failure.side_effect = RuntimeError("handoff failed")
    liveness = Mock()
    failures = FinalApproachEpisodeFailures(threading.RLock(), episode, liveness)

    with pytest.raises(RuntimeError, match="handoff failed"):
        failures.mark_failed()

    # Without the signal the approach would neither command nor end.
    liveness.mark_failed.assert_called_once_with()


def test_failure_after_docking_is_still_dropped() -> None:
    episode = Mock()
    episode.latch_failure.return_value = False
    liveness = Mock()

    FinalApproachEpisodeFailures(threading.RLock(), episode, liveness).mark_failed()

    liveness.mark_failed.assert_not_called()


# --- reset with a failing drain ---------------------------------------------


def test_failed_drain_still_resets_control_and_the_episode() -> None:
    ports = Mock()
    ports.slot.invalidate.return_value = None
    ports.docked_evidence.take_staged.return_value = None
    ports.postprocess_fence.drain.side_effect = TimeoutError("fence stranded")
    reset = FinalApproachCommandReset(FinalApproachCommandResetPorts(
        lock=threading.RLock(),
        slot=ports.slot,
        mailbox=ports.mailbox,
        hold=ports.hold,
        law=ports.law,
        visual_pass=ports.visual_pass,
        status=ports.status,
        liveness=ports.liveness,
        postprocess_fence=ports.postprocess_fence,
        docked_evidence=ports.docked_evidence,
        episode=ports.episode,
    ))

    with pytest.raises(TimeoutError):
        reset.reset_phase()

    ports.hold.clear.assert_called_once_with()
    for owner in (ports.law, ports.visual_pass, ports.status,
                  ports.liveness, ports.episode):
        owner.reset.assert_called_once_with()


# --- command worker -----------------------------------------------------------


def _worker(stop: threading.Event, **overrides) -> NavigationCommandWorker:
    fields = dict(
        stop_event=stop,
        command_event=threading.Event(),
        wall_period_s=lambda period_s: 0.0,
        runtime_session=Mock(),
        logger=Mock(),
    )
    fields.update(overrides)
    return NavigationCommandWorker(NavigationCommandWorkerPorts(**fields))


def _runs_failing_once(stop: threading.Event) -> tuple[list[int], object]:
    runs: list[int] = []

    def run() -> None:
        runs.append(len(runs))
        if len(runs) == 1:
            raise RuntimeError("runtime_session failed")
        stop.set()

    return runs, run


def test_command_worker_restarts_after_an_escaped_error() -> None:
    stop = threading.Event()
    logger = Mock()
    worker = _worker(stop, logger=logger)
    runs, run = _runs_failing_once(stop)
    worker.run = run  # type: ignore[method-assign]

    worker.run_until_stopped()

    assert runs == [0, 1]
    assert "restarting" in logger.error.call_args.args[0]


def test_command_worker_restart_survives_a_raising_logger() -> None:
    stop = threading.Event()
    worker = _worker(
        stop,
        logger=Mock(error=Mock(side_effect=OSError("log down"))),
        scheduler_period_s=0.0,
    )
    runs, run = _runs_failing_once(stop)
    worker.run = run  # type: ignore[method-assign]

    worker.run_until_stopped()

    assert runs == [0, 1]


def test_command_worker_stops_without_running_once_stopped() -> None:
    stop = threading.Event()
    stop.set()
    worker = _worker(stop)
    worker.run = Mock()  # type: ignore[method-assign]

    worker.run_until_stopped()

    worker.run.assert_not_called()


# --- NAV loop -----------------------------------------------------------------


def _loop(cycle_errors: list[Exception | None]) -> tuple[NavigationLoop, Mock, Mock]:
    cycle = Mock()
    logger = Mock()
    on_error = Mock()
    clock = Mock()
    clock.scheduler_wall_period.return_value = 0.0
    loop = NavigationLoop(cycle, clock, logger, on_cycle_error=on_error)
    remaining = list(cycle_errors)

    def run_cycle() -> None:
        error = remaining.pop(0)
        if not remaining:
            loop.stop_event.set()
        if error is not None:
            raise error

    cycle.run.side_effect = run_cycle
    return loop, logger, on_error


def test_nav_loop_survives_an_unexpected_cycle_error_and_fails_closed() -> None:
    error = KeyError("missing mailbox token")
    loop, logger, on_error = _loop([error, None])

    loop._run()

    on_error.assert_called_once_with(error)
    assert "KeyError" in logger.error.call_args.args[0]
    operator = logger.warning.call_args
    assert "internal error" in operator.args[0]
    assert operator.kwargs["key"] == "nav"
    loop.raise_if_failed()


def test_nav_loop_reports_a_repeating_error_once() -> None:
    loop, logger, on_error = _loop([ValueError("x"), ValueError("x"), ValueError("x")])

    loop._run()

    assert on_error.call_count == 3
    assert logger.warning.call_count == 1
    assert logger.error.call_count == 1


def test_nav_loop_survives_failing_recovery_and_logging() -> None:
    loop, logger, on_error = _loop([RuntimeError("a"), RuntimeError("b"), None])
    logger.error.side_effect = RuntimeError("log down")
    on_error.side_effect = RuntimeError("latch down")

    loop._run()

    assert on_error.call_count == 2
    loop.raise_if_failed()


def test_nav_loop_still_stops_on_base_exception() -> None:
    loop, _logger, on_error = _loop([KeyboardInterrupt(), None])  # type: ignore[list-item]

    loop._run_thread()

    on_error.assert_not_called()
    with pytest.raises(KeyboardInterrupt):
        loop.raise_if_failed()


@pytest.mark.parametrize(
    ("phase", "failed"),
    [(NavState.NAV, True), (NavState.DETECT, False), (NavState.RESET, False)],
)
def test_cycle_error_fails_only_an_active_approach(
    phase: NavState, failed: bool,
) -> None:
    state = Mock()
    state.phase = NavPhaseState(current=phase, previous=phase)

    _fail_active_approach(state, KeyError("x"))

    if failed:
        state.navigation_failures.mark_failed.assert_called_once_with(
            fail_reason.INTERNAL_ERROR,
        )
    else:
        state.navigation_failures.mark_failed.assert_not_called()
