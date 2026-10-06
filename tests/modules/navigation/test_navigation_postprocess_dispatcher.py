"""Command-cadence, FIFO, and failure contracts for evidence dispatch."""

from __future__ import annotations

import threading
from contextlib import nullcontext
from unittest.mock import Mock

import pytest

from navpy.logger.navigation_logger import NAVIGATION_EVIDENCE_FAILURE_MARKER
from navpy.modules.navigation.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.modules.navigation.navigation_postprocess_dispatcher import (
    NavigationPostprocessDispatcher,
    NavigationPostprocessFailure,
)


class _Logger:
    def __init__(self) -> None:
        self.errors: list[tuple[str, BaseException]] = []

    def error(self, message: str, error: BaseException) -> None:
        self.errors.append((message, error))


def test_submit_is_nonblocking_and_jobs_remain_fifo():
    dispatcher = NavigationPostprocessDispatcher(_Logger())
    entered = threading.Event()
    release = threading.Event()
    order: list[str] = []

    def first() -> None:
        order.append("first-enter")
        entered.set()
        assert release.wait(timeout=2.0)
        order.append("first-exit")

    dispatcher.submit(first)
    assert entered.wait(timeout=2.0)
    dispatcher.submit(lambda: order.append("second"))
    assert order == ["first-enter"]
    release.set()
    dispatcher.close()

    assert order == ["first-enter", "first-exit", "second"]


def test_failure_is_latched_reported_and_surfaced_after_drain():
    logger = _Logger()
    dispatcher = NavigationPostprocessDispatcher(logger)
    entered = threading.Event()
    release = threading.Event()
    continued = threading.Event()

    def fail() -> None:
        entered.set()
        assert release.wait(timeout=2.0)
        raise ValueError("diagnostics broke")

    dispatcher.submit(fail)
    assert entered.wait(timeout=2.0)
    dispatcher.submit(continued.set)
    release.set()

    with pytest.raises(NavigationPostprocessFailure) as raised:
        dispatcher.close()

    assert isinstance(raised.value.__cause__, ValueError)
    assert continued.is_set()
    assert len(logger.errors) == 1
    assert NAVIGATION_EVIDENCE_FAILURE_MARKER in logger.errors[0][0]


def test_blocked_diagnostics_do_not_delay_the_next_command_slot():
    class FakeClock:
        now_s = 0.0

        def monotonic(self) -> float:
            return self.now_s

        def sleep(self, duration_s: float) -> None:
            self.now_s += duration_s

    class HotCommandEvent:
        @staticmethod
        def wait(timeout: float) -> bool:
            del timeout
            return True

    clock = FakeClock()
    stop_event = threading.Event()
    diagnostic_entered = threading.Event()
    diagnostic_release = threading.Event()
    second_issued = threading.Event()
    starts: list[float] = []
    dispatcher = NavigationPostprocessDispatcher(_Logger())
    dispatcher.start()
    runtime = Mock()
    runtime.take_work.side_effect = ("first", "second")

    def execute(_work: str) -> None:
        starts.append(clock.monotonic())
        if len(starts) == 2:
            second_issued.set()
            stop_event.set()

    def blocked_diagnostic() -> None:
        diagnostic_entered.set()
        assert diagnostic_release.wait(timeout=2.0)

    runtime.execute_work.side_effect = execute
    runtime.postprocess_job.side_effect = (
        blocked_diagnostic,
        lambda: None,
    )
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=HotCommandEvent(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        postprocess_submit=dispatcher.submit,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    ))
    thread = threading.Thread(target=worker.run)
    thread.start()
    try:
        assert diagnostic_entered.wait(timeout=2.0)
        assert not diagnostic_release.is_set()
        assert second_issued.wait(timeout=2.0)
    finally:
        diagnostic_release.set()
        thread.join(timeout=2.0)
        dispatcher.close()

    assert not thread.is_alive()
    assert starts == pytest.approx([0.02, 0.04])
