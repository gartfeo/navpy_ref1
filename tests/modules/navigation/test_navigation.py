import math
import threading
from contextlib import nullcontext
from dataclasses import dataclass, field
from io import StringIO
from unittest.mock import Mock

import numpy as np
import pytest

from navpy.exception_groups import BaseExceptionGroup
from navpy.modules.common.resource_cleanup import CleanupStack
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.logger.navigation_log_streams import NavigationLogStreams
from navpy.logger.navigation_logger import NavigationLogger
from navpy.logger.log_events import LogEvent
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.modules.navigation.nav.nav_law_factory import (
    VehicleTerminalLawConfigProvider,
)
from navpy.modules.navigation.nav.vision_nav.command_executor import (
    TerminalCommandExecutor,
    TerminalExecutorPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_anchor import (
    TerminalLimits,
)
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    TERMINAL_COMMAND_MAX_SOURCE_AGE_S,
    TerminalCommandFreshness,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import TerminalCommandHold
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    TerminalCommandLiveness,
)
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    TerminalPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.command_reset import (
    TerminalCommandReset,
    TerminalCommandResetPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    TerminalCommandOutcome,
    TerminalCommandResult,
    TerminalCommandTransaction,
    TerminalLawEvidence,
)
from navpy.modules.navigation.nav.vision_nav.confirmation import (
    TerminalConfirmation,
    TerminalConfirmationPorts,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    TerminalDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.diagnostics import (
    NavigationTerminalDiagnostics,
    TerminalDiagnosticSnapshot,
)
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.ingress import (
    TerminalIngress,
    TerminalIngressPorts,
)
from navpy.modules.navigation.nav.vision_nav.law import (
    FixedTerminalLawConfigProvider,
    ROLL_LIMIT_CAP_DEG,
    TerminalLawConfig,
    VisionNavLaw,
)
from navpy.modules.navigation.nav.vision_nav.law_plan import TerminalPlanOrigin
from navpy.modules.navigation.nav.vision_nav.runtime import (
    TerminalCommandWorkRuntime,
    TerminalSessionRuntime,
    VisionNavRuntime,
)
from navpy.modules.navigation.nav.vision_nav.runtime_composition import (
    TerminalVehicleDiagnosticReader,
    compose_terminal_runtime,
)
from navpy.modules.navigation.nav.vision_nav.runtime_state import TerminalRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassDetector


@dataclass
class RichTarget:
    frame: TerminalVisionFrame
    pixel: object | None = None
    optics: object = field(default_factory=lambda: _DiagnosticOptics())
    geo: object = field(default_factory=lambda: _DiagnosticGeo())
    timing: object = field(default_factory=lambda: _DiagnosticTiming())

    def __post_init__(self):
        if self.pixel is None:
            self.pixel = _DiagnosticPixel(
                source_timestamp_s=self.frame.source_timestamp_s
            )
        detection_now_s = self.timing.detection_now_s
        receipt_timestamp_s = self.timing.source_receipt_timestamp_s
        receipt_now_s = self.timing.source_receipt_now_s
        self.timing = _DiagnosticTiming(
            detection_now_s=(
                detection_now_s
                if detection_now_s is not None
                else lambda: self.frame.source_timestamp_s
            ),
            source_receipt_timestamp_s=(
                self.frame.source_timestamp_s
                if receipt_timestamp_s is None
                else receipt_timestamp_s
            ),
            source_receipt_now_s=(
                receipt_now_s
                if receipt_now_s is not None
                else lambda: self.frame.source_timestamp_s
            ),
        )

    def visual_detection(self):
        return self


@dataclass(frozen=True)
class _DiagnosticPixel:
    u_px: int | None = None
    v_px: int | None = None
    source_timestamp_s: float | None = None


@dataclass(frozen=True)
class _DiagnosticTiming:
    detection_now_s: object | None = None
    source_receipt_timestamp_s: float | None = None
    source_receipt_now_s: object | None = None
    source_air_speed_mps: float | None = 25.0


class _DiagnosticOptics:
    @staticmethod
    def camera_matrix():
        return None


@dataclass(frozen=True)
class _DiagnosticGeo:
    is_simulation: bool = False
    truth_target_location: object | None = None
    camera_location: object | None = None


class Projector:
    @staticmethod
    def source_name(target):
        return target.frame.source_name

    @staticmethod
    def project(target, generation, air_speed_mps=None):
        frame = target.frame
        return TerminalVisionFrame(
            frame.source_name, generation, frame.task_id, frame.obj_id,
            frame.source_timestamp_s, *frame.body_ray, *frame.control_ray,
            frame.aircraft_roll_deg,
            frame.air_speed_mps if air_speed_mps is None else air_speed_mps,
        )


class Actuator:
    def __init__(self):
        self.calls = []
        self.fail = False

    def issue(self, roll_deg, pitch_deg, throttle):
        if self.fail:
            raise RuntimeError("actuator failed")
        self.calls.append((roll_deg, pitch_deg, throttle))


class Diagnostics:
    def __init__(self, lock):
        self.lock = lock
        self.records = []
        self.fail = False

    def capture(self, frame, target, result):
        assert not self.lock._is_owned()
        if self.fail:
            raise ValueError("diagnostics failed")
        return frame, target, result

    def record(self, diagnostic):
        self.records.append(diagnostic)


class _SourceTimeObserver:
    def __init__(self):
        self.command_outcomes = []

    def record_observation(self, **_kwargs):
        return None

    def record_command(self, **kwargs):
        self.command_outcomes.append(kwargs.get("outcome", "fresh"))


def _ray(yaw_deg=0.0, down_deg=0.0):
    y = math.tan(math.radians(yaw_deg))
    z = math.tan(math.radians(down_deg))
    norm = math.sqrt(1.0 + y * y + z * z)
    return 1.0 / norm, y / norm, z / norm


def _frame(ts, *, body=None, control=None, source="cam", generation=0):
    body = body or _ray()
    control = control or body
    return TerminalVisionFrame(
        source, generation, 1, 2, ts, *body, *control,
    )


_DEFAULT_CONFIG = TerminalLawConfig(-55.0, 25.0, 45.0, 0.5, None)


def _runtime(
    config=_DEFAULT_CONFIG,
    diagnostics_factory=None,
    law_override=None,
    command_event=None,
    source_time=None,
    wall_period_s=lambda period_s: period_s,
):
    lock = threading.RLock()
    slot = NavigationCommandSlot(lock, command_event or threading.Event())
    mailbox = TerminalDiagnosticMailbox()
    epochs = SourceEpochLedger()
    law = law_override or VisionNavLaw(
        FixedTerminalLawConfigProvider(config)
    )
    visual_pass = VisualPassDetector()
    status = TerminalRuntimeStatus()
    actuator = Actuator()
    source_time = source_time or _SourceTimeObserver()
    liveness = TerminalCommandLiveness()
    postprocess_fence = TerminalPostprocessFence()
    freshness = TerminalCommandFreshness(wall_period_s)
    hold = TerminalCommandHold(actuator, source_time, liveness, freshness)
    diagnostics = (
        Diagnostics(lock)
        if diagnostics_factory is None
        else diagnostics_factory(lock)
    )
    command_reset = TerminalCommandReset(TerminalCommandResetPorts(
        lock, slot, mailbox, hold, law, visual_pass, status, liveness,
        postprocess_fence,
    ))
    ingress = TerminalIngress(TerminalIngressPorts(
        lock, slot, Projector(), epochs, mailbox, source_time, command_reset,
    ))
    confirmation = TerminalConfirmation(TerminalConfirmationPorts(
        lock, Projector(), epochs, law,
    ))
    transaction = TerminalCommandTransaction(law, visual_pass, actuator)
    executor = TerminalCommandExecutor(TerminalExecutorPorts(
        slot, transaction, mailbox, diagnostics, status, source_time, hold,
        liveness, freshness, postprocess_fence,
    ))
    runtime = VisionNavRuntime(
        TerminalCommandWorkRuntime(slot, ingress, executor, hold),
        TerminalSessionRuntime(
            lock, epochs, command_reset, visual_pass, status, liveness,
        ),
    )
    runtime._test_confirmation = confirmation
    return runtime, actuator, diagnostics, mailbox


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


def test_production_composition_shares_one_hold_across_command_owners():
    lock = threading.RLock()
    slot = NavigationCommandSlot(lock, threading.Event())
    built = compose_terminal_runtime(
        lock=lock,
        slot=slot,
        sys_id=121,
        actuator=Actuator(),
        diagnostic_reader=Mock(),
        law=Mock(),
        aircraft_roll_deg=lambda: 0.0,
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        navigation_logger=Mock(),
        wall_period_s=lambda period_s: period_s,
    )
    runtime = built.runtime

    commands = runtime._commands
    session = runtime._session
    assert commands._hold is commands._executor._ports.hold
    assert (
        commands._hold._freshness
        is commands._executor._ports.freshness
    )
    assert commands._hold is session._command_reset._ports.hold
    assert (
        session._command_reset
        is commands._ingress._ports.command_reset
    )


def test_empty_autopilot_tick_reissues_cached_primitive_command():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(
        RichTarget(_frame(1.0, control=_ray(yaw_deg=4.0, down_deg=6.0)))
    )
    assert _execute_one(runtime) is not None
    fresh_command = actuator.calls[-1]

    held_work = runtime.take_work()
    assert held_work is not None
    try:
        runtime.execute_work(held_work)
    finally:
        runtime.finish_work(held_work)

    assert actuator.calls == [fresh_command, fresh_command]


def test_held_command_copies_primitives_from_mutable_fresh_result():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(RichTarget(_frame(1.0, control=_ray(yaw_deg=4.0))))
    fresh = _execute_one(runtime)
    assert fresh is not None
    issued = actuator.calls[-1]

    fresh.cmd_roll = 999.0
    fresh.cmd_pitch = 999.0
    fresh.cmd_thr = 0.0
    held = _execute_one(runtime)

    assert held is not None
    assert actuator.calls == [issued, issued]
    assert held is not fresh


def test_raw_source_age_expires_hold_and_latches_navigation_failure():
    source_now_s = [1.0]
    target = RichTarget(
        _frame(1.0),
        timing=_DiagnosticTiming(detection_now_s=lambda: source_now_s[0]),
    )
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(target)
    assert _execute_one(runtime) is not None

    source_now_s[0] = 1.0 + TERMINAL_COMMAND_MAX_SOURCE_AGE_S + 0.001
    assert _execute_one(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()
    assert not runtime.consume_command_liveness_failure()


def test_delayed_fresh_frame_fails_closed_before_first_actuation():
    source_now_s = 1.0 + TERMINAL_COMMAND_MAX_SOURCE_AGE_S + 0.001
    target = RichTarget(
        _frame(1.0),
        timing=_DiagnosticTiming(detection_now_s=lambda: source_now_s),
    )
    runtime, actuator, _diagnostics, _mailbox = _runtime()

    assert runtime.nav(target)
    assert _execute_one(runtime) is None

    assert actuator.calls == []
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_each_continuously_stale_fresh_frame_relatches_liveness():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    for timestamp_s in (1.0, 2.0):
        target = RichTarget(
            _frame(timestamp_s),
            timing=_DiagnosticTiming(
                detection_now_s=lambda ts=timestamp_s: (
                    ts + TERMINAL_COMMAND_MAX_SOURCE_AGE_S + 0.001
                )
            ),
        )
        assert runtime.nav(target)
        assert _execute_one(runtime) is None
        assert runtime.consume_command_liveness_failure()

    assert actuator.calls == []


@pytest.mark.parametrize("speedup", [0.5, 1.0, 3.0, 10.0])
def test_frozen_source_clock_expires_on_speedup_scaled_receipt_age(speedup):
    receipt_now_s = [100.0]
    target = RichTarget(
        _frame(5.0),
        timing=_DiagnosticTiming(
            detection_now_s=lambda: 5.0,
            source_receipt_timestamp_s=100.0,
            source_receipt_now_s=lambda: receipt_now_s[0],
        ),
    )
    runtime, actuator, _diagnostics, _mailbox = _runtime(
        wall_period_s=lambda period_s: period_s / speedup,
    )
    assert runtime.nav(target)
    assert _execute_one(runtime) is not None

    receipt_now_s[0] += (
        TERMINAL_COMMAND_MAX_SOURCE_AGE_S / speedup + 0.001
    )
    assert _execute_one(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_hold_is_cleared_by_reset_discontinuity_and_stale_lease():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(RichTarget(_frame(1.0)))
    assert _execute_one(runtime) is not None

    stale_hold = runtime.take_work()
    assert stale_hold is not None
    runtime.clear_source_discontinuity_state("cam")
    assert runtime.execute_work(stale_hold) is None
    runtime.finish_work(stale_hold)
    assert len(actuator.calls) == 1
    assert runtime.take_work() is None


def test_held_actuator_failure_clears_the_cached_command():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(RichTarget(_frame(1.0)))
    assert _execute_one(runtime) is not None

    actuator.fail = True
    with pytest.raises(RuntimeError, match="actuator failed"):
        _execute_one(runtime)
    actuator.fail = False

    assert runtime.take_work() is None
    assert len(actuator.calls) == 1
    runtime.clear_source_discontinuity_state("cam")
    assert runtime.consume_command_liveness_failure()
    assert not runtime.consume_command_liveness_failure()


def test_reset_waits_for_held_issue_then_prevents_later_actuation():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(RichTarget(_frame(1.0)))
    assert _execute_one(runtime) is not None
    entered = threading.Event()
    release = threading.Event()
    original_issue = actuator.issue

    def blocked_issue(roll_deg, pitch_deg, throttle):
        entered.set()
        assert release.wait(1.0)
        original_issue(roll_deg, pitch_deg, throttle)

    actuator.issue = blocked_issue
    held = runtime.take_work()
    assert held is not None
    issue_thread = threading.Thread(
        target=lambda: (
            runtime.execute_work(held),
            runtime.finish_work(held),
        ),
    )
    issue_thread.start()
    assert entered.wait(1.0)
    reset_done = threading.Event()
    reset_thread = threading.Thread(
        target=lambda: (
            runtime.reset_phase(),
            reset_done.set(),
        ),
    )
    reset_thread.start()
    assert not reset_done.wait(0.05)

    release.set()
    issue_thread.join(1.0)
    reset_thread.join(1.0)

    assert not issue_thread.is_alive()
    assert not reset_thread.is_alive()
    assert reset_done.is_set()
    assert len(actuator.calls) == 2
    assert runtime.take_work() is None


@pytest.mark.parametrize("speedup", [0.5, 1.0, 3.0, 10.0])
def test_slower_source_still_calls_actuator_once_per_autopilot_slot(speedup):
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

    class FakeStopEvent:
        def __init__(self):
            self._set = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        def wait(self, timeout):
            clock.now_s = math.nextafter(clock.now_s + timeout, math.inf)
            return self._set

    command_event = threading.Event()
    source_time = _SourceTimeObserver()
    runtime, actuator, _diagnostics, _mailbox = _runtime(
        command_event=command_event,
        source_time=source_time,
    )
    clock = FakeClock()
    stop_event = FakeStopEvent()
    next_source_s = 0.02
    source_period_s = 0.06
    issue_wall_times = []
    original_issue = actuator.issue

    def issue_and_stop(roll_deg, pitch_deg, throttle):
        original_issue(roll_deg, pitch_deg, throttle)
        issue_wall_times.append(clock.now_s)
        if len(issue_wall_times) == 12:
            stop_event.set()

    actuator.issue = issue_and_stop

    def sleep_and_publish(duration_s):
        nonlocal next_source_s
        clock.now_s = math.nextafter(clock.now_s + duration_s, math.inf)
        source_now_s = clock.now_s * speedup
        while next_source_s <= source_now_s + 1e-12:
            assert runtime.nav(RichTarget(_frame(next_source_s)))
            next_source_s += source_period_s

    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=lambda period_s: period_s / speedup,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=sleep_and_publish,
    ))

    worker.run()

    assert len(actuator.calls) == 12
    assert issue_wall_times == pytest.approx(
        [0.02 * tick / speedup for tick in range(1, 13)]
    )
    assert source_time.command_outcomes.count("fresh") == 4
    assert source_time.command_outcomes.count("held") == 8


def test_initial_aft_does_not_invent_an_acquisition_turn():
    runtime, actuator, _, _ = _runtime()
    assert runtime.nav(RichTarget(_frame(1.0, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))

    result = _execute_one(runtime)

    assert result is not None
    assert actuator.calls[0][0] == 0.0
    assert not runtime.target_passed_override()


def test_duplicate_and_regressed_aft_frames_never_add_pass_evidence():
    runtime, actuator, _, _ = _runtime()
    runtime.nav(RichTarget(_frame(1.0)))
    _execute_one(runtime)
    aft = RichTarget(_frame(2.0, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0)))
    runtime.nav(aft)
    assert _execute_one(runtime) is None
    assert runtime.nav(aft)
    assert runtime.take_work() is None
    assert not runtime.nav(RichTarget(_frame(1.5, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))
    assert runtime.take_work() is None
    assert not runtime.target_passed_override()
    assert len(actuator.calls) == 1


def test_second_fresh_aft_latches_pass_without_command():
    runtime, _, _, _ = _runtime()
    runtime.nav(RichTarget(_frame(1.0)))
    _execute_one(runtime)
    for ts in (2.0, 3.0):
        runtime.nav(RichTarget(_frame(ts, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))
        assert _execute_one(runtime) is None
    assert runtime.target_passed_override()


def test_actuator_failure_mutates_neither_law_anchor_nor_pass_arm():
    runtime, actuator, _, mailbox = _runtime()
    actuator.fail = True
    runtime.nav(RichTarget(_frame(1.0, control=_ray(down_deg=5.0))))
    with pytest.raises(RuntimeError, match="actuator failed"):
        _execute_one(runtime)
    assert mailbox.size == 0
    assert runtime.consume_command_liveness_failure()

    actuator.fail = False
    runtime.nav(RichTarget(_frame(2.0, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))
    assert _execute_one(runtime) is not None
    assert len(actuator.calls) == 1


def test_diagnostic_failure_does_not_mask_actuator_failure():
    runtime, actuator, diagnostics, mailbox = _runtime()
    actuator.fail = True
    diagnostics.fail = True
    runtime.nav(RichTarget(_frame(1.0)))

    with pytest.raises(BaseExceptionGroup) as raised:
        _execute_one(runtime)

    assert [type(error) for error in raised.value.exceptions] == [
        RuntimeError,
        ValueError,
    ]
    assert "actuator failed" in str(raised.value.exceptions[0])
    assert "diagnostics failed" in str(raised.value.exceptions[1])
    assert mailbox.size == 0
    assert runtime.consume_command_liveness_failure()


def test_missing_attitude_is_nonfatal_after_successful_command():
    class Reader:
        @staticmethod
        def read():
            return TerminalDiagnosticSnapshot(attitude=None, location=None)

    compact = StringIO()
    debug = StringIO()
    cache = Mock()
    navigation_logger = NavigationLogger(
        1,
        cache,
        wall_time=lambda: 1000.0,
        timestamp=lambda: "12:00:00.000",
        streams=NavigationLogStreams(compact, debug),
    )
    runtime, actuator, _, mailbox = _runtime(
        diagnostics_factory=lambda _lock: NavigationTerminalDiagnostics(
            navigation_logger,
            Reader(),
        )
    )
    assert runtime.nav(RichTarget(_frame(1.0)))

    result = _execute_one(runtime)

    assert result is not None
    assert len(actuator.calls) == 1
    assert mailbox.size == 0
    status_text = " ".join(
        str(call.args[0]) for call in cache.info.call_args_list if call.args
    )
    assert "ar: N/A" in status_text
    assert "ap: N/A" in status_text
    navigation_logger.drain()
    row = compact.getvalue().strip().split(",")
    assert row[8:10] == ["", ""]
    navigation_logger.close()


def test_telemetry_read_failures_are_independent_and_nonfatal_after_command():
    telemetry_reads = []

    class Vehicle:
        @property
        def attitude(self):
            telemetry_reads.append("attitude")
            raise RuntimeError("attitude unavailable")

        def location(self, is_relative):
            assert is_relative is False
            telemetry_reads.append("location")
            raise OSError("location unavailable")

    compact = StringIO()
    debug = StringIO()
    cache = Mock()
    navigation_logger = NavigationLogger(
        1,
        cache,
        wall_time=lambda: 1000.0,
        timestamp=lambda: "12:00:00.000",
        streams=NavigationLogStreams(compact, debug),
    )
    runtime, actuator, _, mailbox = _runtime(
        diagnostics_factory=lambda _lock: NavigationTerminalDiagnostics(
            navigation_logger,
            TerminalVehicleDiagnosticReader(
                lambda: Vehicle().attitude,
                lambda: Vehicle().location(False),
            ),
        )
    )
    assert runtime.nav(RichTarget(_frame(1.0)))

    result = _execute_one(runtime)

    assert result is not None
    assert len(actuator.calls) == 1
    assert telemetry_reads == ["attitude", "location"]
    assert mailbox.size == 0
    status_text = " ".join(
        str(call.args[0]) for call in cache.info.call_args_list if call.args
    )
    assert "ar: N/A" in status_text
    assert "ap: N/A" in status_text
    navigation_logger.drain()
    assert compact.getvalue().strip().split(",")[8:10] == ["", ""]
    navigation_logger.close()


def test_diagnostic_failure_propagates_without_command_failure():
    runtime, actuator, diagnostics, mailbox = _runtime()
    diagnostics.fail = True
    runtime.nav(RichTarget(_frame(1.0)))

    with pytest.raises(ValueError, match="diagnostics failed"):
        _execute_one(runtime)

    assert mailbox.size == 0
    assert _execute_one(runtime) is not None
    assert len(actuator.calls) == 2


def test_terminal_diagnostics_sample_every_command_but_rate_limit_compact_rows():
    target_location = Location(40.0, 44.0, 100.0, is_absolute=True)
    locations = iter((
        Location(40.01, 44.0, 100.0, is_absolute=True),
        Location(40.001, 44.0, 100.0, is_absolute=True),
    ))

    class Reader:
        @staticmethod
        def read():
            return TerminalDiagnosticSnapshot(
                attitude=None,
                location=next(locations),
            )

    compact = StringIO()
    debug = StringIO()
    navigation_logger = NavigationLogger(
        1,
        Mock(),
        wall_time=lambda: 1000.0,
        timestamp=lambda: "12:00:00.000",
        streams=NavigationLogStreams(compact, debug),
    )
    diagnostics = NavigationTerminalDiagnostics(navigation_logger, Reader())
    result = TerminalCommandResult(
        CalcData(0.0, 0.0, 1.0, 2.0, None),
        TerminalCommandOutcome.ISSUED,
        False,
    )
    target = RichTarget(
        _frame(1.0),
        geo=_DiagnosticGeo(
            is_simulation=True,
            truth_target_location=target_location,
        ),
    )

    diagnostics.record(diagnostics.capture(target.frame, target, result))
    first_snap = navigation_logger.get_snap()
    diagnostics.record(diagnostics.capture(target.frame, target, result))
    second_snap = navigation_logger.get_snap()

    assert second_snap.dist < first_snap.dist
    navigation_logger.drain()
    assert len(compact.getvalue().strip().splitlines()) == 1
    assert debug.getvalue().count("EVENT:TERMINAL_CMD") == 1
    navigation_logger.close()


def test_terminal_diagnostic_capture_is_immutable_before_async_record():
    attitude = Attitude(-5.0, 12.0, 7.0)
    location = Location(40.0, 44.0, 100.0, is_absolute=True)
    target_location = Location(40.001, 44.002, 90.0, is_absolute=True)
    camera_location = Location(40.0, 44.0, 101.0, is_absolute=True)
    matrix = np.eye(3)

    class Reader:
        @staticmethod
        def read():
            return TerminalDiagnosticSnapshot(attitude, location)

    class Optics:
        @staticmethod
        def camera_matrix():
            return matrix

    logger = Mock()
    logger.capture_event_timestamp.return_value = "12:00:00.000"
    logger.log.return_value = True
    target = RichTarget(
        _frame(1.0),
        pixel=_DiagnosticPixel(12, 34, 1.0),
        optics=Optics(),
        geo=_DiagnosticGeo(True, target_location, camera_location),
    )
    result = TerminalCommandResult(
        CalcData(0.0, 0.0, 3.0, -9.0, 0.55),
        TerminalCommandOutcome.ISSUED,
        False,
    )
    diagnostics = NavigationTerminalDiagnostics(logger, Reader())

    captured = diagnostics.capture(target.frame, target, result)
    attitude.roll = 99.0
    location.lat = 50.0
    target_location.lat = 51.0
    camera_location.lat = 52.0
    matrix[0, 0] = 99.0
    target.pixel = _DiagnosticPixel(98, 97, 1.0)
    diagnostics.record(captured)

    sample = logger.log.call_args.kwargs
    assert sample["actual_roll"] == 7.0
    assert sample["c_loc"].lat == 40.0
    assert sample["t_loc"].lat == 40.001
    assert sample["detect_c_loc"].lat == 40.0
    assert sample["x_error"] == 12.0
    assert sample["y_error"] == 34.0
    assert sample["k"][0, 0] == 1.0
    assert sample["k"].flags.writeable is False


def test_terminal_diagnostics_record_separate_command_causality_event():
    logger = Mock()
    logger.capture_event_timestamp.return_value = "12:00:00.000"
    logger.log.return_value = True
    diagnostics = NavigationTerminalDiagnostics(logger, Mock())
    evidence = TerminalLawEvidence(
        control_bearing_deg=3.0,
        lateral_rate_deg_s=0.0,
        aircraft_turn_rate_deg_s=0.0,
        raw_inertial_los_rate_deg_s=0.0,
        inertial_los_rate_deg_s=0.0,
        aircraft_roll_deg=0.0,
        aircraft_pitch_deg=-5.0,
        air_speed_mps=25.0,
        control_elevation_deg=7.0,
        vertical_rate_deg_s=0.4,
        pitch_time_constant_s=0.5,
        configured_limits=TerminalLimits(45.0, -55.0, 25.0),
        effective_limits=TerminalLimits(35.0, -55.0, 3.0),
        raw_roll_deg=12.0,
        raw_pitch_deg=-7.6,
        origin=TerminalPlanOrigin(
            reason="normal",
            anchor_cmd_roll_deg=11.0,
            anchor_cmd_pitch_deg=-7.0,
            dt_s=0.1,
            lateral_nav_constant=4.0,
            vertical_nav_constant=4.0,
        ),
    )
    result = TerminalCommandResult(
        CalcData(0.0, 7.0, 12.0, -7.6, None),
        TerminalCommandOutcome.ISSUED,
        False,
        evidence,
    )

    captured = diagnostics.capture(_frame(1.0), None, result)
    diagnostics.record(captured)

    assert logger.log_event.call_args_list[-1].args[0] is LogEvent.TERMINAL_RESPONSE_STATE
    payload = logger.log_event.call_args_list[-1].args[1]
    assert payload["control_bearing_deg"] == 3.0
    assert payload["raw_roll_deg"] == 12.0
    assert payload["vertical_rate_deg_s"] == 0.4
    assert payload["cmd_roll_deg"] == 12.0


def test_terminal_reset_waits_for_owned_postprocess_before_state_reset():
    lock = threading.RLock()
    fence = TerminalPostprocessFence()
    fence.begin()
    slot = NavigationCommandSlot(lock, threading.Event())
    reset_observed = threading.Event()
    hold = Mock()
    law = Mock()
    law.reset.side_effect = reset_observed.set
    reset = TerminalCommandReset(TerminalCommandResetPorts(
        lock,
        slot,
        TerminalDiagnosticMailbox(),
        hold,
        law,
        Mock(),
        Mock(),
        Mock(),
        fence,
    ))
    thread = threading.Thread(target=reset.invalidate_commands)
    thread.start()

    assert not reset_observed.wait(timeout=0.05)
    fence.finish()
    thread.join(timeout=2.0)

    assert not thread.is_alive()
    assert reset_observed.is_set()
    hold.clear.assert_called_once_with()


def test_pass_suppressed_frame_finishes_the_snap_segment_after_crossing():
    target_location = Location(40.0, 44.0, 100.0, is_absolute=True)
    locations = iter((
        Location(39.99999, 44.0, 100.0, is_absolute=True),
        Location(40.00001, 44.0, 100.0, is_absolute=True),
    ))

    class Reader:
        @staticmethod
        def read():
            return TerminalDiagnosticSnapshot(
                attitude=None,
                location=next(locations),
            )

    navigation_logger = NavigationLogger(
        1,
        Mock(),
        wall_time=lambda: 1000.0,
        timestamp=lambda: "12:00:00.000",
        streams=NavigationLogStreams(StringIO(), StringIO()),
    )
    # NavigationLogger owns a NavigationStreamWorker daemon (started on first
    # write); close it so the worker thread does not outlive the test.
    try:
        diagnostics = NavigationTerminalDiagnostics(navigation_logger, Reader())
        target = RichTarget(
            _frame(1.0),
            geo=_DiagnosticGeo(
                is_simulation=True,
                truth_target_location=target_location,
            ),
        )
        issued = TerminalCommandResult(
            CalcData(0.0, 0.0, 1.0, 2.0, None),
            TerminalCommandOutcome.ISSUED,
            False,
        )
        suppressed = TerminalCommandResult(
            None,
            TerminalCommandOutcome.PASS_SUPPRESSED,
            False,
        )

        diagnostics.record(diagnostics.capture(target.frame, target, issued))
        before_crossing = navigation_logger.get_snap()
        diagnostics.record(diagnostics.capture(target.frame, target, suppressed))
        after_crossing = navigation_logger.get_snap()

        assert before_crossing.dist > 1.0
        assert after_crossing.dist < 0.01
    finally:
        navigation_logger.close()


def test_failed_second_command_preserves_first_raw_rate_anchor():
    """A command that threw must not advance the rate anchor.

    The frame after a failure has to be differentiated against the last
    SUCCESSFUL command's frame -- here 5 deg at t=10.0 -- not against the 30 deg
    frame that threw. Otherwise one actuator error silently injects a huge
    spurious rate into the next command.

    Asserted BY CONSTRUCTION: fly the same two frames through a clean runtime
    with no failure in between and require an identical pitch. An earlier
    version hardcoded `tau * N * rate` from a superseded vertical law -- the
    command now integrates on the previous command and no longer reads
    `pitch_time_constant_s` at all (`law.py:24-28`) -- so it broke when the law
    was retuned even though the property it exists to protect still held. The
    formulation below cannot break that way.
    """

    def pitch_after(*, with_failure: bool) -> float:
        runtime, actuator, _, _ = _runtime()
        runtime.nav(RichTarget(_frame(10.0, control=_ray(down_deg=5.0))))
        _execute_one(runtime)
        if with_failure:
            actuator.fail = True
            runtime.nav(RichTarget(_frame(10.5, control=_ray(down_deg=30.0))))
            with pytest.raises(RuntimeError):
                _execute_one(runtime)
            assert runtime.take_work() is None
            actuator.fail = False
        runtime.nav(RichTarget(_frame(11.0, control=_ray(down_deg=7.0))))
        _execute_one(runtime)
        return actuator.calls[-1][1]

    assert pitch_after(with_failure=True) == pytest.approx(
        pitch_after(with_failure=False)
    )


def test_mailbox_has_no_leaks_on_replace_reset_or_stale_inflight():
    runtime, _, _, mailbox = _runtime()
    assert runtime.nav(RichTarget(_frame(1.0)))
    assert runtime.nav(RichTarget(_frame(2.0)))
    assert mailbox.size == 1
    work = runtime.take_work()
    assert work is not None
    assert work.frame.source_timestamp_s == 2.0
    runtime.nav(RichTarget(_frame(3.0)))
    assert mailbox.size == 2
    runtime.reset_phase()
    assert mailbox.size == 1
    assert runtime.execute_work(work) is None
    runtime.finish_work(work)
    assert mailbox.size == 0


def test_runtime_discontinuity_restarts_only_named_source_epoch():
    runtime, _, _, _ = _runtime()
    runtime.nav(RichTarget(_frame(100.0, source="a")))
    _execute_one(runtime)
    runtime.nav(RichTarget(_frame(100.0, source="b")))
    _execute_one(runtime)
    runtime.clear_source_discontinuity_state("a")

    assert runtime.nav(RichTarget(_frame(1.0, source="a")))
    assert _execute_one(runtime) is not None
    assert not runtime.nav(RichTarget(_frame(1.0, source="b")))


def test_runtime_batch_discontinuity_is_unique_atomic_and_source_local():
    runtime, _, _, _ = _runtime()
    for source in ("a", "b", "c"):
        assert runtime.nav(RichTarget(_frame(100.0, source=source)))
        assert _execute_one(runtime) is not None
    assert runtime.nav(RichTarget(_frame(101.0, source="a")))
    stale_work = runtime.take_work()
    assert stale_work is not None

    runtime.clear_source_discontinuity_state(("a", "b", "a"))

    assert runtime.execute_work(stale_work) is None
    runtime.finish_work(stale_work)
    _run_postprocess(runtime, stale_work)
    for source in ("a", "b"):
        assert runtime.nav(RichTarget(_frame(1.0, source=source)))
        assert _execute_one(runtime) is not None
    assert not runtime.nav(RichTarget(_frame(1.0, source="c")))


def test_unavailable_limits_suppress_command_and_confirmation_closed():
    runtime, actuator, _, mailbox = _runtime(config=None)
    target = RichTarget(_frame(1.0))

    assert runtime.nav(target)
    assert _execute_one(runtime) is None
    assert not runtime._test_confirmation.can_confirm_detection(target)
    assert not runtime._test_confirmation.record_terminal_confirmed_detection(target)
    assert actuator.calls == []
    assert mailbox.size == 0
    assert runtime.consume_command_liveness_failure()


def test_unavailable_limits_cannot_arm_or_latch_visual_pass():
    runtime, actuator, _, mailbox = _runtime(config=None)
    frames = (
        _frame(1.0),
        _frame(
            2.0,
            body=(-1.0, 0.0, 0.0),
            control=(-1.0, 0.0, 0.0),
        ),
        _frame(
            3.0,
            body=(-1.0, 0.0, 0.0),
            control=(-1.0, 0.0, 0.0),
        ),
    )

    for frame in frames:
        assert runtime.nav(RichTarget(frame))
        assert _execute_one(runtime) is None

    assert not runtime.target_passed_override()
    assert actuator.calls == []
    assert mailbox.size == 0
    assert runtime.consume_command_liveness_failure()


@pytest.mark.parametrize(
    "failed_name",
    (
        "PTCH_LIM_MIN_DEG",
        "PTCH_LIM_MAX_DEG",
        "ROLL_LIMIT_DEG",
        "PTCH2SRV_TCONST",
        "TRIM_THROTTLE",
    ),
)
def test_parameter_read_exception_fails_closed_without_actuator(failed_name):
    params = {
        "PTCH_LIM_MIN_DEG": -55.0,
        "PTCH_LIM_MAX_DEG": 25.0,
        "ROLL_LIMIT_DEG": 45.0,
        "PTCH2SRV_TCONST": 0.5,
        "TRIM_THROTTLE": 40.0,
    }

    class Vehicle:
        def get_parameter(self, name, quiet=False):
            assert quiet is True
            if name == failed_name:
                raise RuntimeError(f"{name} unavailable")
            return params[name]

    args = type("Args", (), {"delivery_throttle": None})()
    law = VisionNavLaw(
        VehicleTerminalLawConfigProvider(Vehicle(), args)
    )
    runtime, actuator, _, mailbox = _runtime(law_override=law)
    target = RichTarget(_frame(1.0))

    assert not law.available
    assert runtime.nav(target)
    assert _execute_one(runtime) is None
    assert not runtime._test_confirmation.can_confirm_detection(target)
    assert actuator.calls == []
    assert mailbox.size == 0
    assert runtime.consume_command_liveness_failure()


def test_mid_navigation_task_law_loss_clears_hold_and_latches_failure():
    class MutableConfigProvider:
        config = _DEFAULT_CONFIG

        def read(self):
            return self.config

    provider = MutableConfigProvider()
    law = VisionNavLaw(provider)
    runtime, actuator, _, _ = _runtime(law_override=law)
    assert runtime.nav(RichTarget(_frame(1.0)))
    assert _execute_one(runtime) is not None

    provider.config = None
    runtime.reset_phase()
    assert runtime.nav(RichTarget(_frame(2.0)))
    assert _execute_one(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_valid_pass_suppression_does_not_require_available_limits():
    from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassState

    frame = _frame(1.0)
    law = VisionNavLaw(FixedTerminalLawConfigProvider(None))
    visual_pass = VisualPassDetector()
    visual_pass._state = VisualPassState(
        frame.continuity_key,
        armed=True,
        aft_pending=True,
        passed=True,
    )
    actuator = Actuator()
    transaction = TerminalCommandTransaction(law, visual_pass, actuator)

    result = transaction.execute(frame)

    assert not result.issued
    assert result.outcome is TerminalCommandOutcome.PASS_SUPPRESSED
    assert result.passed
    assert actuator.calls == []


def test_issued_command_carries_exact_frame_local_law_evidence():
    """The bearing MOVES here, 3 deg to 5 deg over one second.

    PN acts on the LOS RATE, so a constant bearing commands zero roll and the
    evidence assertion would hold vacuously. An earlier version held the bearing
    at 3 deg and expected `degrees(sin(bearing))` -- a superseded
    bearing-proportional law. The current one is atan(N*V*rate/g)
    (`law.py:224-231`), which is why the frames now differ.
    """
    frame = _frame(2.0, control=_ray(yaw_deg=5.0, down_deg=7.0))
    law = VisionNavLaw(FixedTerminalLawConfigProvider(_DEFAULT_CONFIG))
    law.seed(_frame(1.0, control=_ray(yaw_deg=3.0, down_deg=5.0)))
    transaction = TerminalCommandTransaction(law, VisualPassDetector(), Actuator())

    result = transaction.execute(frame)

    assert result.evidence is not None
    assert result.evidence.control_bearing_deg == pytest.approx(5.0)
    assert result.evidence.control_elevation_deg == pytest.approx(
        result.calc_data.pitch
    )
    # 2 deg of bearing in 1 s at 25 m/s: atan(4 * 25 * radians(2) / g) = 19.59
    # deg, comfortably under the 35 deg cap, so the raw and issued values
    # coincide and a clamp cannot mask a wrong rate.
    assert result.evidence.raw_roll_deg == pytest.approx(19.5931, abs=1e-3)
    assert result.evidence.raw_roll_deg == pytest.approx(result.calc_data.cmd_roll)
    assert result.evidence.raw_pitch_deg == pytest.approx(
        result.calc_data.cmd_pitch
    )


def test_command_postprocess_preserves_frame_local_law_evidence():
    """Two frames, because the evidence under test is a RATE-derived command.

    A single frame leaves PN nothing to differentiate, so the roll assertion
    would pass at zero whatever postprocessing did to it. The earlier version
    used one frame and expected `degrees(sin(bearing))` from a superseded
    bearing-proportional law.
    """
    runtime, _, diagnostics, _ = _runtime()

    assert runtime.nav(
        RichTarget(_frame(1.0, control=_ray(yaw_deg=3.0, down_deg=5.0)))
    )
    _execute_one(runtime)
    assert runtime.nav(
        RichTarget(_frame(2.0, control=_ray(yaw_deg=5.0, down_deg=7.0)))
    )
    _execute_one(runtime)

    result = diagnostics.records[-1][2]
    assert result.evidence is not None
    assert result.evidence.control_bearing_deg == pytest.approx(5.0)
    # Same geometry as the transaction-level test above, so the two pin the
    # same number through different paths: 19.59 deg.
    assert result.evidence.raw_roll_deg == pytest.approx(19.5931, abs=1e-3)


def test_confirmation_records_fresh_frame_without_command_then_nav_is_duplicate():
    runtime, actuator, _, _ = _runtime()
    target = RichTarget(_frame(1.0, control=_ray(yaw_deg=5.0, down_deg=5.0)))
    confirmation = runtime._test_confirmation
    assert confirmation.can_confirm_detection(target)
    assert confirmation.record_terminal_confirmed_detection(target)
    assert not confirmation.can_confirm_detection(target)
    assert not confirmation.record_terminal_confirmed_detection(target)
    assert runtime.nav(target)
    assert runtime.take_work() is None
    assert actuator.calls == []


def test_delayed_confirmation_uses_the_newer_visual_command_immediately():
    runtime, actuator, _, _ = _runtime()
    confirmation = runtime._test_confirmation
    reviewed = RichTarget(
        _frame(1.0, control=_ray(yaw_deg=5.0, down_deg=5.0))
    )
    delayed = RichTarget(
        _frame(2.0, control=_ray(yaw_deg=36.0, down_deg=5.0))
    )

    assert confirmation.can_confirm_detection(reviewed)
    assert confirmation.can_confirm_detection(delayed)
    assert confirmation.record_terminal_confirmed_detection(delayed)
    assert actuator.calls == []

    next_frame = RichTarget(
        _frame(2.02, control=_ray(yaw_deg=35.0, down_deg=5.0))
    )
    assert runtime.nav(next_frame)
    assert _execute_one(runtime) is not None
    # -35, not the config's -45. `law.py:279` takes
    # min(config.roll_limit_deg, ROLL_LIMIT_CAP_DEG) and the cap is 35
    # (`law.py:41`), so no configuration can ask for more.
    #
    # It rails because 1 deg of bearing in 20 ms is a 50 deg/s LOS rate, which
    # atan(4 * 25 * rate / g) answers with 83 deg. That the command saturates on
    # so little bearing change at this dt is the law's real sensitivity, not an
    # artefact of the fixture.
    assert actuator.calls[0][0] == pytest.approx(-ROLL_LIMIT_CAP_DEG)


def test_raw_timestamp_commands_are_invariant_while_wall_cadence_scales():
    sequences = []
    for speedup in (0.5, 1.0, 3.0, 10.0):
        cadence = SchedulerCadence(
            lambda value=speedup: value,
            high_resolution_timer=False,
        )
        assert cadence.wall_period_for_scheduler_period(0.04) == pytest.approx(
            0.04 / speedup
        )
        runtime, actuator, _, _ = _runtime()
        for ts, down in ((1.0, 5.0), (1.2, 7.0), (1.4, 6.0)):
            runtime.nav(RichTarget(_frame(ts, control=_ray(down_deg=down))))
            _execute_one(runtime)
        sequences.append(tuple(actuator.calls))
        cadence.close()
    assert sequences[1:] == [sequences[0], sequences[0], sequences[0]]
