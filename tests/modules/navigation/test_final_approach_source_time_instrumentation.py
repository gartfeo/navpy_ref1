import threading
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.exception_groups import BaseExceptionGroup
from navpy.modules.common.resource_cleanup import CleanupStack
from scripts import eval_certificate as certificate
from navpy.modules.navigation.nav.vision_nav import command_executor as command_executor_module
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.command_executor import (
    FinalApproachCommandExecutor,
    FinalApproachExecutorPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    FinalApproachCommandFreshness,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    FinalApproachCommandHold,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    FinalApproachCommandLiveness,
)
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.command_reset import (
    FinalApproachCommandReset,
    FinalApproachCommandResetPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    FinalApproachCommandOutcome,
    FinalApproachCommandResult,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    FinalApproachDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.ingress import (
    FinalApproachIngress,
    FinalApproachIngressPorts,
)
from navpy.modules.navigation.nav.vision_nav.runtime import (
    FinalApproachCommandWorkRuntime,
    FinalApproachSessionRuntime,
    VisionNavRuntime,
)
from navpy.modules.navigation.nav.vision_nav.runtime_state import FinalApproachRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.navigation.nav.vision_nav.source_time_adapter import (
    PoseCadenceFinalApproachSourceTimeObserver,
)
from navpy.modules.vehicle import pose_cadence_debug


@dataclass
class _Poi:
    frame: FinalApproachVisionFrame | None
    pixel: object
    timing: object
    source_name: str = "camera"

    def visual_detection(self):
        return self


class _Projector:
    @staticmethod
    def source_name(poi):
        return poi.source_name if poi.source_name else None

    @staticmethod
    def project(poi, generation, air_speed_mps=None):
        frame = poi.frame
        if frame is None:
            return None
        return FinalApproachVisionFrame(
            frame.source_name,
            generation,
            frame.task_id,
            frame.obj_id,
            frame.source_timestamp_s,
            *frame.body_ray,
            *frame.control_ray,
        )


class _IssuedTransaction:
    @staticmethod
    def execute(_frame):
        return FinalApproachCommandResult(
            CalcData(0.0, 0.0, 1.0, 2.0, 0.5),
            outcome=FinalApproachCommandOutcome.ISSUED,
            passed=False,
        )


class _UnissuedTransaction:
    def __init__(self, outcome):
        self._outcome = outcome

    def execute(self, _frame):
        return FinalApproachCommandResult(None, self._outcome, passed=False)


class _ActuatorFailureTransaction:
    @staticmethod
    def execute(_frame):
        raise RuntimeError("actuator failed")


class _Diagnostics:
    @staticmethod
    def capture(frame, poi, result):
        return frame, poi, result

    @staticmethod
    def record(_diagnostic):
        return None


class _HeldActuator:
    def __init__(self):
        self.calls = []
        self.fail = False

    def issue(self, roll_deg, pitch_deg, throttle):
        if self.fail:
            raise RuntimeError("held actuator failed")
        self.calls.append((roll_deg, pitch_deg, throttle))


def _runtime(
    *,
    source_time=None,
    transaction=None,
    diagnostics=None,
    status=None,
    actuator=None,
):
    lock = threading.RLock()
    slot = NavigationCommandSlot(lock, threading.Event())
    mailbox = FinalApproachDiagnosticMailbox()
    epochs = SourceEpochLedger()
    status = status or FinalApproachRuntimeStatus()
    source_time = source_time or PoseCadenceFinalApproachSourceTimeObserver(121)
    liveness = FinalApproachCommandLiveness()
    postprocess_fence = FinalApproachPostprocessFence()
    freshness = FinalApproachCommandFreshness(lambda period_s: period_s)
    hold = FinalApproachCommandHold(
        actuator or _HeldActuator(),
        source_time,
        liveness,
        freshness,
    )
    law = Mock()
    visual_pass = Mock()
    command_reset = FinalApproachCommandReset(FinalApproachCommandResetPorts(
        lock,
        slot,
        mailbox,
        hold,
        law,
        visual_pass,
        status,
        liveness,
        postprocess_fence,
    ))
    ingress = FinalApproachIngress(
        FinalApproachIngressPorts(
            lock,
            slot,
            _Projector(),
            epochs,
            mailbox,
            source_time,
            command_reset,
        )
    )
    executor_ports = FinalApproachExecutorPorts(
            slot,
            transaction or _IssuedTransaction(),
            mailbox,
            diagnostics or _Diagnostics(),
            status,
            source_time,
            hold,
            liveness,
            freshness,
            postprocess_fence,
        )
    executor = FinalApproachCommandExecutor(executor_ports)
    runtime = VisionNavRuntime(
        FinalApproachCommandWorkRuntime(slot, ingress, executor, hold),
        FinalApproachSessionRuntime(
            lock,
            epochs,
            command_reset,
            visual_pass,
            status,
            liveness,
        ),
    )
    return runtime, lock


def _frame(timestamp_s, *, body_x=1.0):
    return FinalApproachVisionFrame(
        "camera",
        0,
        1,
        2,
        timestamp_s,
        body_x,
        0.0,
        0.0,
        body_x,
        0.0,
        0.0,
    )


def _poi(frame, now_s, *, source_name="camera", source_timestamp_s=None):
    timestamp = (
        None if frame is None else frame.source_timestamp_s
    ) if source_timestamp_s is None else source_timestamp_s
    return _Poi(
        frame,
        SimpleNamespace(source_timestamp_s=timestamp),
        SimpleNamespace(
            detection_now_s=now_s,
            source_receipt_timestamp_s=timestamp,
            source_receipt_now_s=lambda: timestamp,
        ),
        source_name,
    )


def _execute_one(runtime):
    work = runtime.take_work()
    assert work is not None
    with CleanupStack() as cleanup:
        cleanup.push(lambda: _run_postprocess(runtime, work))
        cleanup.push(lambda: runtime.finish_work(work))
        return runtime.execute_work(work)


def _run_postprocess(runtime, work):
    job = runtime.postprocess_job(work)
    if job is not None:
        job()


def test_fresh_ingress_and_issued_command_emit_source_time_rows(monkeypatch):
    observation_record = Mock()
    worker_record = Mock()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(
        pose_cadence_debug, "record_observation", observation_record
    )
    monkeypatch.setattr(pose_cadence_debug, "record_worker", worker_record)
    runtime, lock = _runtime()
    provider_calls = []

    def now():
        provider_calls.append(lock._is_owned())
        return 5.03

    poi = _poi(_frame(5.0), now)

    assert runtime.nav(poi)
    assert _execute_one(runtime) is not None

    observation_record.assert_called_once()
    worker_record.assert_called_once()
    assert observation_record.call_args.args[2:] == (5.0, 5.03, "fresh")
    assert worker_record.call_args.args[3:] == (
        5.0,
        5.03,
        "measured",
        "fresh",
    )
    assert provider_calls == [False, True, False]


def test_held_command_reuses_raw_timestamp_without_rerunning_transaction(
    monkeypatch,
):
    worker_record = Mock()
    transaction = _IssuedTransaction()
    transaction.execute = Mock(wraps=transaction.execute)
    actuator = _HeldActuator()
    provider_lock_states = []
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "record_worker", worker_record)
    runtime, lock = _runtime(transaction=transaction, actuator=actuator)

    def source_now():
        provider_lock_states.append(lock._is_owned())
        return 5.03

    assert runtime.nav(_poi(_frame(5.0), source_now))
    assert _execute_one(runtime) is not None
    assert _execute_one(runtime) is not None

    assert transaction.execute.call_count == 1
    assert actuator.calls == [(1.0, 2.0, 0.5)]
    assert [call.args[3] for call in worker_record.call_args_list] == [5.0, 5.0]
    assert [call.args[-1] for call in worker_record.call_args_list] == [
        "fresh",
        "held",
    ]
    assert provider_lock_states == [False, True, False, True, False]


def test_all_ingress_outcomes_are_recorded_after_unlock(monkeypatch):
    records = Mock()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "record_observation", records)
    runtime, lock = _runtime()

    def now():
        assert not lock._is_owned()
        return 10.1

    assert not runtime.nav(_poi(_frame(1.0), now, source_name=""))
    assert not runtime.nav(
        _poi(None, now, source_timestamp_s=1.5)
    )
    assert runtime.nav(_poi(_frame(2.0), now))
    assert runtime.nav(_poi(_frame(2.0), now))
    assert not runtime.nav(_poi(_frame(1.0), now))

    assert [call.args[-1] for call in records.call_args_list] == [
        "invalid_source",
        "invalid_frame",
        "fresh",
        "duplicate",
        "regression",
    ]


def test_newest_observation_replaces_pending_work_without_changing_timestamps(
    monkeypatch,
):
    records = Mock()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "record_observation", records)
    runtime, _lock = _runtime()

    assert runtime.nav(_poi(_frame(3.0), lambda: 3.01))
    assert runtime.nav(_poi(_frame(3.02), lambda: 3.03))
    assert runtime.nav(_poi(_frame(3.04), lambda: 3.05))

    assert [call.args[-1] for call in records.call_args_list] == [
        "fresh",
        "fresh",
        "fresh",
    ]
    work = runtime.take_work()
    assert work is not None
    assert work.frame.source_timestamp_s == 3.04
    runtime.finish_work(work)


def test_disabled_observer_never_calls_source_clock_provider(monkeypatch):
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", False)
    provider = Mock(return_value=1.1)
    observer = PoseCadenceFinalApproachSourceTimeObserver(121)

    observer.record_observation(
        source_timestamp_s=1.0,
        source_now_s=provider,
        outcome="fresh",
    )
    observer.record_command(
        wall_start_s=0.5,
        source_timestamp_s=1.0,
        source_now_s=provider,
        execution_ms=2.0,
    )

    provider.assert_not_called()


def test_disabled_runtime_samples_source_clock_only_for_freshness(monkeypatch):
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", False)
    runtime, lock = _runtime()
    lock_states = []

    def source_now():
        lock_states.append(lock._is_owned())
        return 1.02

    assert runtime.nav(_poi(_frame(1.0), source_now))
    assert _execute_one(runtime) is not None

    assert lock_states == [True]


def test_source_clock_diagnostic_failure_cannot_change_issued_control(
    monkeypatch,
    caplog,
):
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    caplog.set_level("WARNING")
    runtime, _lock = _runtime()
    calls = 0

    def source_now():
        nonlocal calls
        calls += 1
        if calls == 3:
            raise OSError("diagnostic source clock failed")
        return 1.02

    assert runtime.nav(_poi(_frame(1.0), source_now))
    assert _execute_one(runtime) is not None

    assert calls == 3
    assert "diagnostic source clock failed" in caplog.text


def test_observer_failures_are_logged_and_do_not_change_control(
    monkeypatch, caplog
):
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(
        pose_cadence_debug,
        "record_observation",
        Mock(side_effect=OSError("observation sink failed")),
    )
    monkeypatch.setattr(
        pose_cadence_debug,
        "record_worker",
        Mock(side_effect=OSError("worker sink failed")),
    )
    caplog.set_level("WARNING")
    runtime, _lock = _runtime()

    assert runtime.nav(_poi(_frame(1.0), lambda: 1.02))
    assert _execute_one(runtime) is not None

    assert "observation sink failed" in caplog.text
    assert "worker sink failed" in caplog.text


def test_stale_lease_duplicate_and_regression_emit_no_worker_rows(monkeypatch):
    workers = Mock()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "record_worker", workers)
    runtime, _lock = _runtime()

    assert runtime.nav(_poi(_frame(1.0), lambda: 1.01))
    stale = runtime.take_work()
    assert stale is not None
    runtime.reset_phase()
    assert runtime.execute_work(stale) is None
    runtime.finish_work(stale)
    _run_postprocess(runtime, stale)
    assert runtime.nav(_poi(_frame(2.0), lambda: 2.01))
    assert _execute_one(runtime) is not None
    workers.reset_mock()

    assert runtime.nav(_poi(_frame(2.0), lambda: 2.01))
    assert not runtime.nav(_poi(_frame(1.5), lambda: 2.01))
    assert runtime.take_work() is None
    workers.assert_not_called()


@pytest.mark.parametrize(
    ("outcome", "liveness_failed"),
    [
        (FinalApproachCommandOutcome.LAW_UNAVAILABLE, True),
        (FinalApproachCommandOutcome.PASS_SUPPRESSED, False),
    ],
)
def test_unissued_results_emit_no_worker_row(
    monkeypatch,
    outcome,
    liveness_failed,
):
    workers = Mock()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "record_worker", workers)
    runtime, _lock = _runtime(transaction=_UnissuedTransaction(outcome))

    assert runtime.nav(_poi(_frame(1.0), lambda: 1.01))
    assert _execute_one(runtime) is None
    workers.assert_not_called()
    assert runtime.consume_command_liveness_failure() is liveness_failed


def test_actuator_failure_emits_no_worker_row(monkeypatch):
    workers = Mock()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "record_worker", workers)
    runtime, _lock = _runtime(transaction=_ActuatorFailureTransaction())

    assert runtime.nav(_poi(_frame(1.0), lambda: 1.01))
    with pytest.raises(RuntimeError, match="actuator failed"):
        _execute_one(runtime)
    workers.assert_not_called()
    assert runtime.consume_command_liveness_failure()


def test_multiple_post_fence_diagnostic_failures_are_grouped(monkeypatch):
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", False)

    class FailingStatus:
        @staticmethod
        def record(_frame):
            raise LookupError("status failed")

    class FailingDiagnostics:
        @staticmethod
        def capture(_frame, _poi, _result):
            raise ValueError("rich diagnostics failed")

        @staticmethod
        def record(_diagnostic):
            return None

    runtime, _lock = _runtime(
        status=FailingStatus(),
        diagnostics=FailingDiagnostics(),
    )
    assert runtime.nav(_poi(_frame(1.0), lambda: 1.01))

    with pytest.raises(BaseExceptionGroup) as raised:
        _execute_one(runtime)

    assert [str(error) for error in raised.value.exceptions] == [
        "status failed",
        "rich diagnostics failed",
    ]


def _source_time_run(monkeypatch, output_dir, wall_times):
    pose_cadence_debug._buffers.clear()
    pose_cadence_debug._written.clear()
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", True)
    monkeypatch.setattr(pose_cadence_debug, "_OUTPUT_DIR", str(output_dir))
    wall = iter(wall_times[::2])
    command_times = iter(
        value
        for start in wall_times[1::2]
        for value in (start, start + 0.0001)
    )
    observer = PoseCadenceFinalApproachSourceTimeObserver(
        121,
        wall_time_s=lambda: next(wall),
    )
    monkeypatch.setattr(
        command_executor_module,
        "time",
        SimpleNamespace(perf_counter=lambda: next(command_times)),
    )
    runtime, _lock = _runtime(source_time=observer)
    for timestamp in (5.0, 5.2):
        assert runtime.nav(
            _poi(_frame(timestamp), lambda value=timestamp: value + 0.03)
        )
        assert _execute_one(runtime) is not None
    pose_cadence_debug.dump_all()
    return certificate.source_time_summary(output_dir)


def test_raw_rows_reach_summary_independent_of_wall_cadence(monkeypatch, tmp_path):
    summaries = [
        _source_time_run(
            monkeypatch,
            tmp_path / label,
            wall_times,
        )
        for label, wall_times in (
            ("realtime", (100.0, 100.01, 100.2, 100.21)),
            ("ten-x", (200.0, 200.001, 200.02, 200.021)),
        )
    ]

    for summary in summaries:
        observation = summary["streams"]["observation"]
        worker = summary["streams"]["worker"]
        assert observation["source_gap_ms"]["mean"] == pytest.approx(200.0)
        assert worker["source_gap_ms"]["mean"] == pytest.approx(200.0)
        assert observation["age_ms"]["mean"] == pytest.approx(30.0)
        assert worker["age_ms"]["mean"] == pytest.approx(30.0)
        assert "sources" not in worker
    assert summaries[0]["streams"]["worker"]["wall_gap_ms"]["mean"] == pytest.approx(
        200.0
    )
    assert summaries[1]["streams"]["worker"]["wall_gap_ms"]["mean"] == pytest.approx(
        20.0
    )
