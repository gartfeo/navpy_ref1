import threading
import time
from dataclasses import dataclass, field
from unittest.mock import patch

import pytest

from navpy.modules.vision import gimbal_cadence
from navpy.modules.vision.gimbal_cadence import FixedCadenceRunner


@dataclass
class _Step:
    calls: int = 0

    def advance(self) -> None:
        self.calls += 1


@dataclass
class _Cadence:
    speedup: float
    requested_periods: list[float] = field(default_factory=list)

    def wall_period_for_scheduler_period(
        self,
        scheduler_period_s: float,
    ) -> float:
        self.requested_periods.append(scheduler_period_s)
        return scheduler_period_s / self.speedup


class _OneCycleEvent:
    def __init__(self) -> None:
        self._checks = 0
        self.waits: list[float] = []

    def is_set(self) -> bool:
        self._checks += 1
        return self._checks > 1

    def wait(self, timeout_s: float) -> bool:
        self.waits.append(timeout_s)
        return False


class _Clock:
    def __init__(self, *values: float) -> None:
        self._values = iter(values)

    def monotonic(self) -> float:
        return next(self._values)


@pytest.mark.parametrize("speedup", [0.5, 1.0, 3.0, 10.0])
def test_speedup_changes_only_wall_period(monkeypatch, speedup: float):
    step = _Step()
    cadence = _Cadence(speedup)
    runner = FixedCadenceRunner(step, 0.02, cadence)
    event = _OneCycleEvent()
    runner._stop_event = event  # type: ignore[assignment]
    monkeypatch.setattr(gimbal_cadence, "time", _Clock(10.0, 10.0))

    runner._run()

    assert step.calls == 1
    assert cadence.requested_periods == [0.02]
    assert event.waits == pytest.approx([0.02 / speedup])


def test_slow_step_does_not_create_catch_up_steps(monkeypatch):
    step = _Step()
    runner = FixedCadenceRunner(step, 0.02, None)
    event = _OneCycleEvent()
    runner._stop_event = event  # type: ignore[assignment]
    monkeypatch.setattr(gimbal_cadence, "time", _Clock(10.0, 10.08))

    runner._run()

    assert step.calls == 1
    assert event.waits == [0.0]


def test_stop_before_start_is_safe():
    runner = FixedCadenceRunner(_Step(), 0.02, None)
    assert runner.stop() is True
    assert not runner.is_running


def test_stop_retains_live_worker_after_join_timeout(monkeypatch):
    entered = threading.Event()
    release = threading.Event()

    class _BlockedStep:
        def advance(self) -> None:
            entered.set()
            release.wait(1.0)

    runner = FixedCadenceRunner(_BlockedStep(), 0.02, None)
    original_join = threading.Thread.join
    runner.start()
    assert entered.wait(0.5)
    monkeypatch.setattr(threading.Thread, "join", lambda self, timeout=None: None)

    assert runner.stop() is False

    assert runner.is_running
    assert runner.start() is False
    release.set()
    monkeypatch.setattr(threading.Thread, "join", original_join)
    deadline_s = time.monotonic() + 0.5
    while runner.is_running and time.monotonic() < deadline_s:
        time.sleep(0.005)
    assert runner.stop() is True
    assert not runner.is_running


def test_runner_can_restart_after_prior_worker_exits():
    step = _Step()
    runner = FixedCadenceRunner(step, 0.01, None)

    assert runner.start()
    time.sleep(0.03)
    runner.stop()
    first_count = step.calls
    assert runner.start()
    time.sleep(0.03)
    runner.stop()

    assert first_count > 0
    assert step.calls > first_count


def test_delayed_bootstrap_cannot_revive_retired_generation_after_restart():
    release_first = threading.Event()
    second_started = threading.Event()
    thread_ids: set[int] = set()

    class _ThreadRecordingStep:
        def advance(self) -> None:
            thread_ids.add(threading.get_ident())
            second_started.set()

    runner = FixedCadenceRunner(_ThreadRecordingStep(), 0.005, None)
    original_start = threading.Thread.start
    start_count = 0
    launcher = None

    def delay_first_start(thread):
        nonlocal start_count, launcher
        start_count += 1
        if start_count != 1:
            return original_start(thread)

        def launch_later():
            assert release_first.wait(timeout=1.0)
            original_start(thread)

        launcher = threading.Thread(target=launch_later, daemon=True)
        original_start(launcher)
        return None

    with patch.object(threading.Thread, "start", delay_first_start):
        assert runner.start() is True
        assert runner.stop() is True
        assert runner.start() is True
        assert second_started.wait(timeout=1.0)
        release_first.set()
        launcher.join(timeout=1.0)
        time.sleep(0.03)
        assert runner.stop() is True

    assert len(thread_ids) == 1


def test_step_failure_is_persistent_health_failure() -> None:
    failure = RuntimeError("simulated gimbal step failed")
    attempted = threading.Event()

    class _FailingStep:
        def advance(self) -> None:
            attempted.set()
            raise failure

    runner = FixedCadenceRunner(_FailingStep(), 0.02, None)
    runner.start()
    assert attempted.wait(timeout=1.0)

    for _ in range(2):
        with pytest.raises(RuntimeError) as raised:
            runner.raise_if_failed()
        assert raised.value is failure
    assert runner.stop() is True


def test_start_failure_after_native_launch_retains_worker_owner() -> None:
    runner = FixedCadenceRunner(_Step(), 0.02, None)
    original_start = threading.Thread.start

    def launch_then_fail(thread):
        original_start(thread)
        raise RuntimeError("runner launch acknowledgement failed")

    with patch.object(threading.Thread, "start", launch_then_fail):
        with pytest.raises(RuntimeError, match="launch acknowledgement"):
            runner.start()

    assert runner._thread is not None
    if not runner.stop():
        assert runner.stop()


@pytest.mark.parametrize("period_s", [0.0, -0.1])
def test_nonpositive_period_is_rejected(period_s: float):
    with pytest.raises(ValueError, match="scheduler_period_s"):
        FixedCadenceRunner(_Step(), period_s, None)
