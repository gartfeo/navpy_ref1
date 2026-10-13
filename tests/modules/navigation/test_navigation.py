import math
import threading
from contextlib import nullcontext
from dataclasses import dataclass, field
from io import StringIO
from unittest.mock import Mock

import numpy as np
import pytest

from navpy.core.kernel.exception_groups import BaseExceptionGroup
from navpy.core.kernel.resource_cleanup import CleanupStack
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.approach.logs.navigation_log_streams import NavigationLogStreams
from navpy.core.approach.logs.navigation_logger import NavigationLogger
from navpy.core.approach.logs.log_events import LogEvent
from navpy.core.approach.calc_data import CalcData
from navpy.core.command.navigation_command_slot import NavigationCommandSlot
from navpy.core.approach.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.plugins.vehicle.ardupilot.final_approach_airframe_config import (
    VehicleFinalApproachLawConfigProvider,
)
from navpy.core.approach.vision_nav.command_executor import (
    FinalApproachCommandExecutor,
    FinalApproachExecutorPorts,
)
from navpy.plugins.laws.vision_pn.command_anchor import (
    FinalApproachLimits,
)
from navpy.core.approach.vision_nav.command_freshness import (
    FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
    FinalApproachCommandFreshness,
)
from navpy.core.approach.vision_nav.command_hold import (
    ATTITUDE_REFRESH_S,
    FinalApproachCommandHold,
)
from navpy.core.approach.vision_nav.command_liveness import (
    FinalApproachCommandLiveness,
    FinalApproachEpisodeFailures,
)
from navpy.core.approach.vision_nav.docked_evidence import (
    FinalApproachDockedEvidence,
)
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeLatch
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from navpy.core.approach.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.core.approach.vision_nav.command_reset import (
    FinalApproachCommandReset,
    FinalApproachCommandResetPorts,
)
from navpy.core.approach.vision_nav.command_transaction import (
    FinalApproachCommandOutcome,
    FinalApproachCommandResult,
    FinalApproachCommandTransaction,
    FinalApproachLawEvidence,
)
from navpy.core.approach.vision_nav.confirmation import (
    FinalApproachConfirmation,
    FinalApproachConfirmationPorts,
)
from navpy.core.approach.vision_nav.diagnostic_mailbox import (
    FinalApproachDiagnosticMailbox,
)
from navpy.core.approach.vision_nav.diagnostics import (
    NavigationFinalApproachDiagnostics,
    FinalApproachDiagnosticSnapshot,
)
from navpy.core.ports.frame import FinalApproachVisionFrame
from navpy.core.approach.vision_nav.ingress import (
    FinalApproachIngress,
    FinalApproachIngressPorts,
)
from navpy.core.ports.law_config import (
    FixedFinalApproachLawConfigProvider,
    FinalApproachLawConfig,
)
from navpy.plugins.laws.vision_pn.command_anchor import ROLL_LIMIT_CAP_DEG
from navpy.plugins.laws.vision_pn.law import VisionNavLaw
from navpy.plugins.laws.vision_pn.law_plan import FinalApproachPlanOrigin
from navpy.core.approach.vision_nav.runtime import (
    FinalApproachCommandWorkRuntime,
    FinalApproachSessionRuntime,
    VisionNavRuntime,
)
from navpy.core.approach.vision_nav.runtime_composition import (
    FinalApproachVehicleDiagnosticReader,
    compose_final_approach_runtime,
)
from navpy.core.approach.vision_nav.runtime_state import FinalApproachRuntimeStatus
from navpy.core.approach.vision_nav.source_epoch import SourceEpochLedger
from navpy.core.approach.vision_nav.visual_pass import VisualPassDetector


@dataclass
class RichPoi:
    frame: FinalApproachVisionFrame
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
    truth_poi_location: object | None = None
    camera_location: object | None = None


class Projector:
    @staticmethod
    def source_name(poi):
        return poi.frame.source_name

    @staticmethod
    def project(poi, generation, air_speed_mps=None):
        frame = poi.frame
        return FinalApproachVisionFrame(
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

    def capture(self, frame, poi, result):
        assert not self.lock._is_owned()
        if self.fail:
            raise ValueError("diagnostics failed")
        return frame, poi, result

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
    return FinalApproachVisionFrame(
        source, generation, 1, 2, ts, *body, *control,
    )


_DEFAULT_CONFIG = FinalApproachLawConfig(-55.0, 25.0, 45.0, 0.5, None)


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
    mailbox = FinalApproachDiagnosticMailbox()
    epochs = SourceEpochLedger()
    law = law_override or VisionNavLaw(
        FixedFinalApproachLawConfigProvider(config)
    )
    visual_pass = VisualPassDetector()
    episode = FinalApproachEpisodeLatch()
    docked_rows = []
    status = FinalApproachRuntimeStatus()
    actuator = Actuator()
    source_time = source_time or _SourceTimeObserver()
    liveness = FinalApproachCommandLiveness()
    worker_failures = FinalApproachEpisodeFailures(lock, episode, liveness)
    postprocess_fence = FinalApproachPostprocessFence()
    freshness = FinalApproachCommandFreshness(wall_period_s)
    hold = FinalApproachCommandHold(
        actuator, source_time, worker_failures, freshness,
    )
    diagnostics = (
        Diagnostics(lock)
        if diagnostics_factory is None
        else diagnostics_factory(lock)
    )
    docked_recorder = [docked_rows.append]
    armed_rows = []
    docked_evidence = FinalApproachDockedEvidence(
        lock, lambda event: docked_recorder[0](event), postprocess_fence,
        armed_rows.append,
    )
    command_reset = FinalApproachCommandReset(FinalApproachCommandResetPorts(
        lock, slot, mailbox, hold, law, visual_pass, status, liveness,
        postprocess_fence, docked_evidence, episode,
    ))
    ingress = FinalApproachIngress(FinalApproachIngressPorts(
        lock, slot, Projector(), epochs, mailbox, source_time, command_reset,
    ))
    confirmation = FinalApproachConfirmation(FinalApproachConfirmationPorts(
        lock, Projector(), epochs, law,
    ))
    transaction = FinalApproachCommandTransaction(
        law, visual_pass, actuator, episode,
    )
    executor = FinalApproachCommandExecutor(FinalApproachExecutorPorts(
        slot, transaction, mailbox, diagnostics, status, source_time, hold,
        worker_failures, freshness, postprocess_fence,
    ))
    runtime = VisionNavRuntime(
        FinalApproachCommandWorkRuntime(
            slot, ingress, executor, hold, docked_evidence,
        ),
        FinalApproachSessionRuntime(
            lock, epochs, command_reset, episode, status, liveness,
            docked_evidence,
        ),
    )
    runtime._test_confirmation = confirmation
    runtime._test_docked_rows = docked_rows
    runtime._test_armed_rows = armed_rows
    runtime._test_docked_recorder = docked_recorder
    return runtime, actuator, diagnostics, mailbox


def _advancing_poi(frame, aircraft_now_s):
    """Frame whose source (aircraft) clock reads the mutable ``aircraft_now_s``."""
    return RichPoi(
        frame,
        timing=_DiagnosticTiming(detection_now_s=lambda: aircraft_now_s[0]),
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


def test_production_composition_shares_one_hold_across_command_owners():
    lock = threading.RLock()
    slot = NavigationCommandSlot(lock, threading.Event())
    built = compose_final_approach_runtime(
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
    aircraft_now_s = [1.0]
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(_advancing_poi(
        _frame(1.0, control=_ray(yaw_deg=4.0, down_deg=6.0)),
        aircraft_now_s,
    ))
    assert _execute_one(runtime) is not None
    fresh_command = actuator.calls[-1]

    aircraft_now_s[0] += ATTITUDE_REFRESH_S
    held_work = runtime.take_work()
    assert held_work is not None
    try:
        runtime.execute_work(held_work)
    finally:
        runtime.finish_work(held_work)

    assert actuator.calls == [fresh_command, fresh_command]


def test_held_command_copies_primitives_from_mutable_fresh_result():
    aircraft_now_s = [1.0]
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(
        _advancing_poi(_frame(1.0, control=_ray(yaw_deg=4.0)), aircraft_now_s)
    )
    fresh = _execute_one(runtime)
    assert fresh is not None
    issued = actuator.calls[-1]

    fresh.cmd_roll = 999.0
    fresh.cmd_pitch = 999.0
    fresh.cmd_thr = 0.0
    aircraft_now_s[0] += ATTITUDE_REFRESH_S
    held = _execute_one(runtime)

    assert held is not None
    assert actuator.calls == [issued, issued]
    assert held is not fresh


def test_raw_source_age_expires_hold_and_latches_navigation_failure():
    source_now_s = [1.0]
    poi = RichPoi(
        _frame(1.0),
        timing=_DiagnosticTiming(detection_now_s=lambda: source_now_s[0]),
    )
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(poi)
    assert _execute_one(runtime) is not None

    source_now_s[0] = 1.0 + FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S + 0.001
    assert _execute_one(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()
    assert not runtime.consume_command_liveness_failure()


def test_delayed_fresh_frame_fails_closed_before_first_actuation():
    source_now_s = 1.0 + FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S + 0.001
    poi = RichPoi(
        _frame(1.0),
        timing=_DiagnosticTiming(detection_now_s=lambda: source_now_s),
    )
    runtime, actuator, _diagnostics, _mailbox = _runtime()

    assert runtime.nav(poi)
    assert _execute_one(runtime) is None

    assert actuator.calls == []
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_each_continuously_stale_fresh_frame_relatches_liveness():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    for timestamp_s in (1.0, 2.0):
        poi = RichPoi(
            _frame(timestamp_s),
            timing=_DiagnosticTiming(
                detection_now_s=lambda ts=timestamp_s: (
                    ts + FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S + 0.001
                )
            ),
        )
        assert runtime.nav(poi)
        assert _execute_one(runtime) is None
        assert runtime.consume_command_liveness_failure()

    assert actuator.calls == []


@pytest.mark.parametrize("speedup", [0.5, 1.0, 3.0, 10.0])
def test_frozen_source_clock_expires_on_speedup_scaled_receipt_age(speedup):
    receipt_now_s = [100.0]
    poi = RichPoi(
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
    assert runtime.nav(poi)
    assert _execute_one(runtime) is not None

    receipt_now_s[0] += (
        FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S / speedup + 0.001
    )
    assert _execute_one(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_hold_is_cleared_by_reset_discontinuity_and_stale_lease():
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(RichPoi(_frame(1.0)))
    assert _execute_one(runtime) is not None

    stale_hold = runtime.take_work()
    assert stale_hold is not None
    runtime.clear_source_discontinuity_state("cam")
    assert runtime.execute_work(stale_hold) is None
    runtime.finish_work(stale_hold)
    assert len(actuator.calls) == 1
    assert runtime.take_work() is None


def test_held_actuator_failure_clears_the_cached_command():
    aircraft_now_s = [1.0]
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))
    assert _execute_one(runtime) is not None

    aircraft_now_s[0] += ATTITUDE_REFRESH_S
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
    aircraft_now_s = [1.0]
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))
    assert _execute_one(runtime) is not None
    aircraft_now_s[0] += ATTITUDE_REFRESH_S
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
def test_worker_sends_new_commands_on_their_slot_and_paces_held_by_aircraft_time(
    speedup,
):
    """Every new command goes out on its AP slot; held re-sends are paced.

    Before design a2 every empty AP slot re-sent the held command (4 fresh +
    8 held in 12 slots). Worker slots run on the NOMINAL speedup in wall time,
    so when SITL fell behind nominal the re-sends overran SITL's one datagram
    per 10 ms of sim input and queued commands for seconds. Now the held
    command is still freshness-checked on every slot (see the stale tests),
    but re-sent only once ATTITUDE_REFRESH_S of aircraft time has passed since
    the last send, so the rate follows aircraft time, not wall-clock slots.
    """
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

    slot_s = 0.02
    source_period_s = 0.7
    stop_aircraft_s = 1.5
    command_event = threading.Event()
    source_time = _SourceTimeObserver()
    runtime, actuator, _diagnostics, _mailbox = _runtime(
        command_event=command_event,
        source_time=source_time,
    )
    clock = FakeClock()
    stop_event = FakeStopEvent()
    next_source_s = slot_s
    published_s = []
    send_wall_s = []
    original_issue = actuator.issue

    def aircraft_now_s():
        return clock.now_s * speedup

    def issue_and_time(roll_deg, pitch_deg, throttle):
        original_issue(roll_deg, pitch_deg, throttle)
        send_wall_s.append(clock.now_s)

    actuator.issue = issue_and_time

    def sleep_and_publish(duration_s):
        nonlocal next_source_s
        clock.now_s = math.nextafter(clock.now_s + duration_s, math.inf)
        # Publish only once the aircraft clock reached the frame time, so the
        # frame never has a negative source age.
        while next_source_s <= aircraft_now_s():
            poi = RichPoi(
                _frame(next_source_s),
                timing=_DiagnosticTiming(detection_now_s=aircraft_now_s),
            )
            assert runtime.nav(poi)
            published_s.append(aircraft_now_s())
            next_source_s += source_period_s
        if aircraft_now_s() >= stop_aircraft_s:
            stop_event.set()

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

    send_aircraft_s = [wall_s * speedup for wall_s in send_wall_s]
    outcomes = source_time.command_outcomes
    assert len(actuator.calls) == len(outcomes)
    assert outcomes == ["fresh", "held", "fresh", "held", "fresh"]
    # Every send lands on an AP slot; new commands on the slot they arrive.
    for sent_s in send_aircraft_s:
        assert sent_s / slot_s == pytest.approx(round(sent_s / slot_s))
    fresh_s = [
        sent_s
        for sent_s, outcome in zip(send_aircraft_s, outcomes)
        if outcome == "fresh"
    ]
    assert fresh_s == pytest.approx(published_s)
    # A held re-send comes on the first slot at least one refresh period
    # after the previous send: never earlier, never in a burst.
    for previous_s, sent_s, outcome in zip(
        send_aircraft_s, send_aircraft_s[1:], outcomes[1:]
    ):
        if outcome == "held":
            assert ATTITUDE_REFRESH_S - 1e-9 <= sent_s - previous_s
            assert sent_s - previous_s < ATTITUDE_REFRESH_S + slot_s + 1e-9


def _refresh_runtime(frame_ts, aircraft_now_s):
    runtime, actuator, _diagnostics, _mailbox = _runtime()
    assert runtime.nav(_advancing_poi(_frame(frame_ts), aircraft_now_s))
    assert _execute_one(runtime) is not None
    return runtime, actuator


def _held_tick(runtime):
    work = runtime.take_work()
    assert work is not None
    try:
        return runtime.execute_work(work)
    finally:
        runtime.finish_work(work)


def test_new_command_is_sent_immediately_even_inside_refresh_period():
    aircraft_now_s = [1.0]
    runtime, actuator = _refresh_runtime(1.0, aircraft_now_s)

    aircraft_now_s[0] = 1.02
    assert runtime.nav(_advancing_poi(
        _frame(1.02, control=_ray(yaw_deg=5.0)), aircraft_now_s
    ))
    assert _execute_one(runtime) is not None

    assert len(actuator.calls) == 2
    assert actuator.calls[0] != actuator.calls[1]


def test_held_command_is_resent_only_after_refresh_period_of_aircraft_time():
    aircraft_now_s = [1.0]
    runtime, actuator = _refresh_runtime(1.0, aircraft_now_s)

    aircraft_now_s[0] = 1.0 + ATTITUDE_REFRESH_S - 0.01
    assert _held_tick(runtime) is None
    assert len(actuator.calls) == 1
    assert not runtime.consume_command_liveness_failure()
    assert runtime.has_command_pending_or_in_flight()

    aircraft_now_s[0] = 1.0 + ATTITUDE_REFRESH_S
    assert _held_tick(runtime) is not None
    assert actuator.calls == [actuator.calls[0], actuator.calls[0]]


def test_long_gap_sends_one_refresh_without_catch_up_burst():
    aircraft_now_s = [1.0]
    runtime, actuator = _refresh_runtime(1.0, aircraft_now_s)

    # Nearly two refresh periods pass with no slot; the frame is still fresh.
    aircraft_now_s[0] = 1.95
    assert _held_tick(runtime) is not None
    for _ in range(3):
        assert _held_tick(runtime) is None

    assert len(actuator.calls) == 2
    assert not runtime.consume_command_liveness_failure()


def test_stale_held_command_fails_closed_on_its_tick_without_refresh_due():
    aircraft_now_s = [1.9]
    # Frame source time 1.0, first sent at aircraft time 1.9 (age 0.9 s).
    runtime, actuator = _refresh_runtime(1.0, aircraft_now_s)

    aircraft_now_s[0] = 1.0 + FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S + 0.001
    assert aircraft_now_s[0] - 1.9 < ATTITUDE_REFRESH_S
    assert _held_tick(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_new_command_resets_the_held_refresh_deadline():
    aircraft_now_s = [1.0]
    runtime, actuator = _refresh_runtime(1.0, aircraft_now_s)

    aircraft_now_s[0] = 1.4
    assert runtime.nav(_advancing_poi(_frame(1.4), aircraft_now_s))
    assert _execute_one(runtime) is not None

    aircraft_now_s[0] = 1.0 + ATTITUDE_REFRESH_S + 0.1
    assert _held_tick(runtime) is None
    assert len(actuator.calls) == 2

    aircraft_now_s[0] = 1.4 + ATTITUDE_REFRESH_S
    assert _held_tick(runtime) is not None
    assert len(actuator.calls) == 3
    assert actuator.calls[2] == actuator.calls[1]


def test_initial_aft_does_not_invent_an_acquisition_turn():
    runtime, actuator, _, _ = _runtime()
    assert runtime.nav(RichPoi(_frame(1.0, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))

    result = _execute_one(runtime)

    assert result is not None
    assert actuator.calls[0][0] == 0.0
    assert not runtime.poi_passed_override()


def test_duplicate_and_regressed_aft_frames_never_add_miss_evidence():
    runtime, actuator, _, _ = _runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(runtime)
    aft = RichPoi(_frame(2.0, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0)))
    runtime.nav(aft)
    assert _execute_one(runtime) is None
    assert runtime.nav(aft)
    assert runtime.take_work() is None
    assert not runtime.nav(RichPoi(_frame(1.5, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))
    assert runtime.take_work() is None
    assert not runtime.poi_passed_override()
    assert len(actuator.calls) == 1


_AFT = (-1.0, 0.0, 0.0)


def test_second_fresh_aft_is_a_missed_dock_without_command():
    """An aft dock is a miss (NAV_FAIL: missed_dock), never a pass."""
    runtime, actuator, _, _ = _runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(runtime)
    for ts in (2.0, 3.0):
        runtime.nav(RichPoi(_frame(ts, body=_AFT, control=_AFT)))
        assert _execute_one(runtime) is None
    assert not runtime.poi_passed_override()
    assert runtime.session.consume_missed_dock() is True
    assert runtime.session.consume_missed_dock() is False
    assert len(actuator.calls) == 1


def test_ideal_360_visible_aft_dock_after_the_miss_never_commands():
    """ideal_360 keeps an aft dock in view: frames keep coming, no command."""
    runtime, actuator, diagnostics, _ = _runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(runtime)
    for ts in (2.0, 3.0, 4.0, 5.0):
        runtime.nav(RichPoi(_frame(ts, body=_AFT, control=_AFT)))
        _execute_one(runtime)
    # A source reset erases the aft gate but not the episode's miss.
    runtime.invalidate_commands()
    runtime.nav(RichPoi(_frame(6.0)))
    _execute_one(runtime)
    assert len(actuator.calls) == 1
    assert runtime.session.latch_docked(_contact()) is False


def _identity(episode=1, task_id=1, dock_uid=2, vehicle=7):
    return DockContactIdentity(vehicle, task_id, dock_uid, episode)


def _contact(identity=None):
    return DockContactEvent(identity or _identity(), 12.5, 0.3, 0.2, 0.1)


def _armed_runtime():
    runtime, actuator, diagnostics, mailbox = _runtime()
    runtime.session.begin_dock_episode(_identity())
    return runtime, actuator, diagnostics, mailbox


def test_contact_latches_once_and_stages_evidence_for_the_worker():
    """The callback latches and fences; it writes nothing itself."""
    runtime, _, _, _ = _armed_runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(runtime)

    assert runtime._test_armed_rows == [_identity()]  # written at arming
    assert runtime.session.latch_docked(_contact()) is True
    assert runtime.session.latch_docked(_contact()) is False
    assert runtime.poi_passed_override()
    assert runtime._test_docked_rows == []
    assert runtime.has_command_pending_or_in_flight()

    work = runtime.take_work()
    assert runtime.execute_work(work) is None
    runtime.finish_work(work)
    _run_postprocess(runtime, work)
    assert runtime._test_docked_rows == [_contact()]
    assert runtime.take_work() is None


@pytest.mark.parametrize(
    "identity",
    (
        _identity(episode=2),
        _identity(dock_uid=3),
        _identity(task_id=9),
        _identity(vehicle=8),
    ),
)
def test_contact_for_another_episode_dock_task_or_vehicle_is_ignored(identity):
    runtime, _, _, _ = _armed_runtime()
    assert runtime.session.latch_docked(_contact(identity)) is False
    assert not runtime.poi_passed_override()
    assert runtime.take_work() is None


def test_unarmed_runtime_never_docks():
    runtime, _, _, _ = _runtime()
    assert runtime.session.latch_docked(_contact()) is False
    assert not runtime.poi_passed_override()


def test_reset_phase_clears_the_episode_latch_but_invalidate_does_not():
    runtime, _, _, _ = _armed_runtime()
    assert runtime.session.latch_docked(_contact()) is True
    runtime.invalidate_commands()
    runtime.clear_source_discontinuity_state("cam")
    assert runtime.poi_passed_override()
    runtime.reset_phase()
    assert not runtime.poi_passed_override()
    assert runtime.session.latch_docked(_contact()) is False  # no armed episode


def test_contact_then_detection_loss_or_nav_fail_is_ignored():
    """Race: contact first; a later loss/NAV_FAIL is not admitted."""
    runtime, _, _, _ = _armed_runtime()
    assert runtime.session.latch_docked(_contact()) is True
    assert runtime.session.latch_failure() is False
    assert runtime.poi_passed_override()


def test_nav_fail_then_contact_is_ignored():
    """Race: a NAV_FAIL latched first wins; the contact stages nothing."""
    runtime, _, _, _ = _armed_runtime()
    assert runtime.session.latch_failure() is True
    assert runtime.session.latch_docked(_contact()) is False
    assert not runtime.poi_passed_override()
    assert runtime.take_work() is None


def test_contact_source_reset_then_late_frame_issues_zero_commands():
    """Race: contact, a source reset, then a late frame: still no command."""
    aircraft_now_s = [1.0]
    runtime, actuator, diagnostics, _ = _armed_runtime()
    assert runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))
    assert _execute_one(runtime) is not None
    assert runtime.session.latch_docked(_contact()) is True
    _run_postprocess(runtime, runtime.take_work())

    runtime.invalidate_commands()  # detector source reset
    aircraft_now_s[0] = 1.1
    late = _frame(1.1, source="cam", generation=1)
    assert runtime.nav(_advancing_poi(late, aircraft_now_s))
    assert _execute_one(runtime) is None
    aircraft_now_s[0] = 1.0 + 2.0 * ATTITUDE_REFRESH_S
    assert runtime.take_work() is None  # no held refresh after docking

    assert len(actuator.calls) == 1
    assert runtime.poi_passed_override()
    terminal = [
        record for record in diagnostics.records
        if record[2] is not None
        and record[2].outcome is FinalApproachCommandOutcome.TERMINAL_SUPPRESSED
    ]
    assert len(terminal) == 1


def test_contact_source_reset_then_stale_frame_writes_no_row_and_no_liveness():
    """The terminal state is checked before freshness: a stale late frame
    after docking neither latches a liveness failure nor writes a row."""
    aircraft_now_s = [1.0]
    runtime, actuator, diagnostics, _ = _armed_runtime()
    assert runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))
    assert _execute_one(runtime) is not None
    assert runtime.session.latch_docked(_contact()) is True
    _run_postprocess(runtime, runtime.take_work())

    runtime.invalidate_commands()  # detector source reset
    stale = _frame(1.1, source="cam", generation=1)
    assert runtime.nav(_advancing_poi(stale, aircraft_now_s))
    aircraft_now_s[0] = 1.1 + 5.0  # the frame is long stale when executed
    assert _execute_one(runtime) is None

    assert not runtime.consume_command_liveness_failure()
    assert len(actuator.calls) == 1
    [_, _, last] = diagnostics.records[-1]
    assert last.outcome is FinalApproachCommandOutcome.TERMINAL_SUPPRESSED


def test_terminal_suppressed_result_writes_no_command_row():
    events = []

    class EventLogger:
        @staticmethod
        def log_event(event, payload, *, timestamp=None):
            events.append(event)

        @staticmethod
        def capture_event_timestamp():
            return "12:00:00.000"

    diagnostics = NavigationFinalApproachDiagnostics(EventLogger(), reader=None)
    result = FinalApproachCommandResult(
        None, FinalApproachCommandOutcome.TERMINAL_SUPPRESSED, False,
    )
    diagnostics.record(diagnostics.capture(_frame(1.0), None, result))
    assert events == []


def test_worker_failure_before_contact_wins_and_the_contact_is_ignored():
    """Codex P2: a stale frame fails on the worker before the navigation loop
    consumes it; a contact in that window must not complete the approach."""
    aircraft_now_s = [1.0]
    runtime, actuator, _, _ = _armed_runtime()
    assert runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))
    aircraft_now_s[0] = 1.0 + 5.0  # stale when the worker executes it
    assert _execute_one(runtime) is None  # freshness failure on the worker

    assert runtime.session.latch_docked(_contact()) is False
    assert not runtime.poi_passed_override()
    assert runtime.take_work() is None  # no DOCKED evidence staged
    assert runtime.consume_command_liveness_failure()
    assert actuator.calls == []


def test_held_command_expiry_before_contact_wins():
    aircraft_now_s = [1.0]
    runtime, actuator = _refresh_runtime(1.0, aircraft_now_s)
    runtime.session.begin_dock_episode(_identity())
    aircraft_now_s[0] = 1.0 + 5.0  # the held command has expired
    _held_tick(runtime)

    assert runtime.session.latch_docked(_contact()) is False
    assert not runtime.poi_passed_override()
    assert runtime.consume_command_liveness_failure()


def test_worker_failure_after_contact_is_dropped():
    """Contact first wins: a later worker failure neither ends the episode
    as a failure nor latches liveness."""
    episode = FinalApproachEpisodeLatch()
    episode.begin(_identity())
    assert episode.latch_docked(_contact())
    liveness = FinalApproachCommandLiveness()
    FinalApproachEpisodeFailures(threading.RLock(), episode, liveness).mark_failed()
    assert episode.docked
    assert not liveness.consume_failure()


def test_due_held_refresh_after_contact_does_not_actuate():
    """The latch retires the hold and queued frames atomically."""
    aircraft_now_s = [1.0]
    runtime, actuator, _, _ = _armed_runtime()
    assert runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))
    assert _execute_one(runtime) is not None
    assert runtime.nav(_advancing_poi(_frame(1.01), aircraft_now_s))

    assert runtime.session.latch_docked(_contact()) is True
    aircraft_now_s[0] = 1.0 + 2.0 * ATTITUDE_REFRESH_S

    for _ in range(3):
        work = runtime.take_work()
        if work is None:
            break
        try:
            assert runtime.execute_work(work) is None
        finally:
            runtime.finish_work(work)
    assert runtime.take_work() is None
    assert len(actuator.calls) == 1
    assert runtime.poi_passed_override()
    assert not runtime.consume_command_liveness_failure()


def test_docked_row_follows_an_earlier_command_whose_postprocess_is_pending():
    """The worker issued a command but not yet submitted its evidence when
    the contact latches; FIFO order still puts the DOCKED row last."""
    runtime, _, diagnostics, _ = _armed_runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    command_work = runtime.take_work()
    assert runtime.execute_work(command_work) is not None
    command_job = runtime.postprocess_job(command_work)
    runtime.finish_work(command_work)
    order = []
    diagnostics.record = lambda diagnostic: order.append("cmd")
    runtime._test_docked_recorder[0] = lambda event: order.append("docked")

    assert runtime.session.latch_docked(_contact()) is True
    assert order == []
    docked_work = runtime.take_work()
    runtime.execute_work(docked_work)
    docked_job = runtime.postprocess_job(docked_work)
    runtime.finish_work(docked_work)

    for job in (command_job, docked_job):  # the dispatcher's FIFO order
        job()
    assert order == ["cmd", "docked"]


def test_reset_flushes_untaken_docked_evidence_after_blocked_command_evidence():
    """A reset before the worker took the DOCKED row waits for the earlier
    command evidence (fence), then writes the DOCKED row inline."""
    runtime, _, diagnostics, _ = _armed_runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    command_work = runtime.take_work()
    runtime.execute_work(command_work)
    command_job = runtime.postprocess_job(command_work)
    runtime.finish_work(command_work)
    order = []
    release = threading.Event()

    def slow_record(_diagnostic):
        release.wait(timeout=5.0)
        order.append("cmd")

    diagnostics.record = slow_record
    runtime._test_docked_recorder[0] = lambda event: order.append("docked")
    assert runtime.session.latch_docked(_contact()) is True

    command_thread = threading.Thread(target=command_job)
    reset_thread = threading.Thread(target=runtime.reset_phase)
    command_thread.start()
    reset_thread.start()
    reset_thread.join(timeout=0.2)
    assert reset_thread.is_alive()
    assert order == []
    release.set()
    command_thread.join(timeout=5.0)
    reset_thread.join(timeout=5.0)

    assert order == ["cmd", "docked"]
    assert runtime.take_work() is None


def test_failed_docked_evidence_surfaces_from_the_postprocess_job():
    runtime, _, _, _ = _armed_runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(runtime)

    def broken(_event):
        raise OSError("evidence sink failed")

    runtime._test_docked_recorder[0] = broken
    assert runtime.session.latch_docked(_contact()) is True
    work = runtime.take_work()
    job = runtime.postprocess_job(work)
    with pytest.raises(OSError, match="evidence sink failed"):
        job()
    runtime.invalidate_commands()  # the fence slot was released


def test_concurrent_contacts_stage_exactly_one_docked_row():
    runtime, _, _, _ = _armed_runtime()
    runtime.nav(RichPoi(_frame(1.0)))
    _execute_one(runtime)
    start = threading.Barrier(8)
    results = []

    def latch():
        start.wait(timeout=5.0)
        results.append(runtime.session.latch_docked(_contact()))

    threads = [threading.Thread(target=latch) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=5.0)

    assert sorted(results) == [False] * 7 + [True]
    work = runtime.take_work()
    _run_postprocess(runtime, work)
    assert len(runtime._test_docked_rows) == 1
    assert runtime.take_work() is None


def test_actuator_failure_mutates_neither_law_anchor_nor_pass_arm():
    runtime, actuator, _, mailbox = _runtime()
    actuator.fail = True
    runtime.nav(RichPoi(_frame(1.0, control=_ray(down_deg=5.0))))
    with pytest.raises(RuntimeError, match="actuator failed"):
        _execute_one(runtime)
    assert mailbox.size == 0
    assert runtime.consume_command_liveness_failure()

    actuator.fail = False
    runtime.nav(RichPoi(_frame(2.0, body=(-1.0, 0.0, 0.0), control=(-1.0, 0.0, 0.0))))
    assert _execute_one(runtime) is not None
    assert len(actuator.calls) == 1


def test_diagnostic_failure_does_not_mask_actuator_failure():
    runtime, actuator, diagnostics, mailbox = _runtime()
    actuator.fail = True
    diagnostics.fail = True
    runtime.nav(RichPoi(_frame(1.0)))

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
            return FinalApproachDiagnosticSnapshot(attitude=None, location=None)

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
        diagnostics_factory=lambda _lock: NavigationFinalApproachDiagnostics(
            navigation_logger,
            Reader(),
        )
    )
    assert runtime.nav(RichPoi(_frame(1.0)))

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
        diagnostics_factory=lambda _lock: NavigationFinalApproachDiagnostics(
            navigation_logger,
            FinalApproachVehicleDiagnosticReader(
                lambda: Vehicle().attitude,
                lambda: Vehicle().location(False),
            ),
        )
    )
    assert runtime.nav(RichPoi(_frame(1.0)))

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
    aircraft_now_s = [1.0]
    runtime, actuator, diagnostics, mailbox = _runtime()
    diagnostics.fail = True
    runtime.nav(_advancing_poi(_frame(1.0), aircraft_now_s))

    with pytest.raises(ValueError, match="diagnostics failed"):
        _execute_one(runtime)

    assert mailbox.size == 0
    aircraft_now_s[0] += ATTITUDE_REFRESH_S
    assert _execute_one(runtime) is not None
    assert len(actuator.calls) == 2


def test_final_approach_diagnostics_sample_every_command_but_rate_limit_compact_rows():
    poi_location = Location(40.0, 44.0, 100.0, is_absolute=True)
    locations = iter((
        Location(40.01, 44.0, 100.0, is_absolute=True),
        Location(40.001, 44.0, 100.0, is_absolute=True),
    ))

    class Reader:
        @staticmethod
        def read():
            return FinalApproachDiagnosticSnapshot(
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
    diagnostics = NavigationFinalApproachDiagnostics(navigation_logger, Reader())
    result = FinalApproachCommandResult(
        CalcData(0.0, 0.0, 1.0, 2.0, None),
        FinalApproachCommandOutcome.ISSUED,
        False,
    )
    poi = RichPoi(
        _frame(1.0),
        geo=_DiagnosticGeo(
            is_simulation=True,
            truth_poi_location=poi_location,
        ),
    )

    diagnostics.record(diagnostics.capture(poi.frame, poi, result))
    first_snap = navigation_logger.get_snap()
    diagnostics.record(diagnostics.capture(poi.frame, poi, result))
    second_snap = navigation_logger.get_snap()

    assert second_snap.dist < first_snap.dist
    navigation_logger.drain()
    assert len(compact.getvalue().strip().splitlines()) == 1
    assert debug.getvalue().count("EVENT:FINAL_APPROACH_CMD") == 1
    navigation_logger.close()


def test_final_approach_diagnostic_capture_is_immutable_before_async_record():
    attitude = Attitude(-5.0, 12.0, 7.0)
    location = Location(40.0, 44.0, 100.0, is_absolute=True)
    poi_location = Location(40.001, 44.002, 90.0, is_absolute=True)
    camera_location = Location(40.0, 44.0, 101.0, is_absolute=True)
    matrix = np.eye(3)

    class Reader:
        @staticmethod
        def read():
            return FinalApproachDiagnosticSnapshot(attitude, location)

    class Optics:
        @staticmethod
        def camera_matrix():
            return matrix

    logger = Mock()
    logger.capture_event_timestamp.return_value = "12:00:00.000"
    logger.log.return_value = True
    poi = RichPoi(
        _frame(1.0),
        pixel=_DiagnosticPixel(12, 34, 1.0),
        optics=Optics(),
        geo=_DiagnosticGeo(True, poi_location, camera_location),
    )
    result = FinalApproachCommandResult(
        CalcData(0.0, 0.0, 3.0, -9.0, 0.55),
        FinalApproachCommandOutcome.ISSUED,
        False,
    )
    diagnostics = NavigationFinalApproachDiagnostics(logger, Reader())

    captured = diagnostics.capture(poi.frame, poi, result)
    attitude.roll = 99.0
    location.lat = 50.0
    poi_location.lat = 51.0
    camera_location.lat = 52.0
    matrix[0, 0] = 99.0
    poi.pixel = _DiagnosticPixel(98, 97, 1.0)
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


def test_final_approach_diagnostics_record_separate_command_causality_event():
    logger = Mock()
    logger.capture_event_timestamp.return_value = "12:00:00.000"
    logger.log.return_value = True
    diagnostics = NavigationFinalApproachDiagnostics(logger, Mock())
    evidence = FinalApproachLawEvidence(
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
        configured_limits=FinalApproachLimits(45.0, -55.0, 25.0),
        effective_limits=FinalApproachLimits(35.0, -55.0, 3.0),
        raw_roll_deg=12.0,
        raw_pitch_deg=-7.6,
        origin=FinalApproachPlanOrigin(
            reason="normal",
            anchor_cmd_roll_deg=11.0,
            anchor_cmd_pitch_deg=-7.0,
            dt_s=0.1,
            lateral_nav_constant=4.0,
            vertical_nav_constant=4.0,
        ),
    )
    result = FinalApproachCommandResult(
        CalcData(0.0, 7.0, 12.0, -7.6, None),
        FinalApproachCommandOutcome.ISSUED,
        False,
        evidence,
    )

    captured = diagnostics.capture(_frame(1.0), None, result)
    diagnostics.record(captured)

    assert logger.log_event.call_args_list[-1].args[0] is LogEvent.FINAL_APPROACH_RESPONSE_STATE
    payload = logger.log_event.call_args_list[-1].args[1]
    assert payload["control_bearing_deg"] == 3.0
    assert payload["raw_roll_deg"] == 12.0
    assert payload["vertical_rate_deg_s"] == 0.4
    assert payload["cmd_roll_deg"] == 12.0


def test_final_approach_reset_waits_for_owned_postprocess_before_state_reset():
    lock = threading.RLock()
    fence = FinalApproachPostprocessFence()
    fence.begin()
    slot = NavigationCommandSlot(lock, threading.Event())
    reset_observed = threading.Event()
    hold = Mock()
    law = Mock()
    law.reset.side_effect = reset_observed.set
    reset = FinalApproachCommandReset(FinalApproachCommandResetPorts(
        lock,
        slot,
        FinalApproachDiagnosticMailbox(),
        hold,
        law,
        Mock(),
        Mock(),
        Mock(),
        fence,
        FinalApproachDockedEvidence(lock, Mock(), fence, Mock()),
        Mock(),
    ))
    thread = threading.Thread(target=reset.invalidate_commands)
    thread.start()

    assert not reset_observed.wait(timeout=0.05)
    fence.finish()
    thread.join(timeout=2.0)

    assert not thread.is_alive()
    assert reset_observed.is_set()
    hold.clear.assert_called_once_with()


def test_aft_suppressed_frame_finishes_the_snap_segment_after_crossing():
    poi_location = Location(40.0, 44.0, 100.0, is_absolute=True)
    locations = iter((
        Location(39.99999, 44.0, 100.0, is_absolute=True),
        Location(40.00001, 44.0, 100.0, is_absolute=True),
    ))

    class Reader:
        @staticmethod
        def read():
            return FinalApproachDiagnosticSnapshot(
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
        diagnostics = NavigationFinalApproachDiagnostics(navigation_logger, Reader())
        poi = RichPoi(
            _frame(1.0),
            geo=_DiagnosticGeo(
                is_simulation=True,
                truth_poi_location=poi_location,
            ),
        )
        issued = FinalApproachCommandResult(
            CalcData(0.0, 0.0, 1.0, 2.0, None),
            FinalApproachCommandOutcome.ISSUED,
            False,
        )
        suppressed = FinalApproachCommandResult(
            None,
            FinalApproachCommandOutcome.AFT_SUPPRESSED,
            False,
        )

        diagnostics.record(diagnostics.capture(poi.frame, poi, issued))
        before_crossing = navigation_logger.get_snap()
        diagnostics.record(diagnostics.capture(poi.frame, poi, suppressed))
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
        runtime.nav(RichPoi(_frame(10.0, control=_ray(down_deg=5.0))))
        _execute_one(runtime)
        if with_failure:
            actuator.fail = True
            runtime.nav(RichPoi(_frame(10.5, control=_ray(down_deg=30.0))))
            with pytest.raises(RuntimeError):
                _execute_one(runtime)
            assert runtime.take_work() is None
            actuator.fail = False
        runtime.nav(RichPoi(_frame(11.0, control=_ray(down_deg=7.0))))
        _execute_one(runtime)
        return actuator.calls[-1][1]

    assert pitch_after(with_failure=True) == pytest.approx(
        pitch_after(with_failure=False)
    )


def test_mailbox_has_no_leaks_on_replace_reset_or_stale_inflight():
    runtime, _, _, mailbox = _runtime()
    assert runtime.nav(RichPoi(_frame(1.0)))
    assert runtime.nav(RichPoi(_frame(2.0)))
    assert mailbox.size == 1
    work = runtime.take_work()
    assert work is not None
    assert work.frame.source_timestamp_s == 2.0
    runtime.nav(RichPoi(_frame(3.0)))
    assert mailbox.size == 2
    runtime.reset_phase()
    assert mailbox.size == 1
    assert runtime.execute_work(work) is None
    runtime.finish_work(work)
    assert mailbox.size == 0


def test_runtime_discontinuity_restarts_only_named_source_epoch():
    runtime, _, _, _ = _runtime()
    runtime.nav(RichPoi(_frame(100.0, source="a")))
    _execute_one(runtime)
    runtime.nav(RichPoi(_frame(100.0, source="b")))
    _execute_one(runtime)
    runtime.clear_source_discontinuity_state("a")

    assert runtime.nav(RichPoi(_frame(1.0, source="a")))
    assert _execute_one(runtime) is not None
    assert not runtime.nav(RichPoi(_frame(1.0, source="b")))


def test_runtime_batch_discontinuity_is_unique_atomic_and_source_local():
    runtime, _, _, _ = _runtime()
    for source in ("a", "b", "c"):
        assert runtime.nav(RichPoi(_frame(100.0, source=source)))
        assert _execute_one(runtime) is not None
    assert runtime.nav(RichPoi(_frame(101.0, source="a")))
    stale_work = runtime.take_work()
    assert stale_work is not None

    runtime.clear_source_discontinuity_state(("a", "b", "a"))

    assert runtime.execute_work(stale_work) is None
    runtime.finish_work(stale_work)
    _run_postprocess(runtime, stale_work)
    for source in ("a", "b"):
        assert runtime.nav(RichPoi(_frame(1.0, source=source)))
        assert _execute_one(runtime) is not None
    assert not runtime.nav(RichPoi(_frame(1.0, source="c")))


def test_unavailable_limits_suppress_command_and_confirmation_closed():
    runtime, actuator, _, mailbox = _runtime(config=None)
    poi = RichPoi(_frame(1.0))

    assert runtime.nav(poi)
    assert _execute_one(runtime) is None
    assert not runtime._test_confirmation.can_confirm_detection(poi)
    assert not runtime._test_confirmation.record_final_approach_confirmed_detection(poi)
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
        assert runtime.nav(RichPoi(frame))
        assert _execute_one(runtime) is None

    assert not runtime.poi_passed_override()
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

    args = type("Args", (), {"final_approach_throttle": None})()
    law = VisionNavLaw(
        VehicleFinalApproachLawConfigProvider(Vehicle(), args)
    )
    runtime, actuator, _, mailbox = _runtime(law_override=law)
    poi = RichPoi(_frame(1.0))

    assert not law.available
    assert runtime.nav(poi)
    assert _execute_one(runtime) is None
    assert not runtime._test_confirmation.can_confirm_detection(poi)
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
    assert runtime.nav(RichPoi(_frame(1.0)))
    assert _execute_one(runtime) is not None

    provider.config = None
    runtime.reset_phase()
    assert runtime.nav(RichPoi(_frame(2.0)))
    assert _execute_one(runtime) is None

    assert len(actuator.calls) == 1
    assert runtime.take_work() is None
    assert runtime.consume_command_liveness_failure()


def test_valid_aft_suppression_does_not_require_available_limits():
    from navpy.core.approach.vision_nav.visual_pass import VisualPassState

    frame = _frame(1.0, body=_AFT, control=_AFT)
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(None))
    visual_pass = VisualPassDetector()
    visual_pass._state = VisualPassState(
        frame.continuity_key,
        armed=True,
        aft_pending=True,
    )
    actuator = Actuator()
    episode = FinalApproachEpisodeLatch()
    transaction = FinalApproachCommandTransaction(
        law, visual_pass, actuator, episode,
    )

    result = transaction.execute(frame)

    assert not result.issued
    assert result.outcome is FinalApproachCommandOutcome.AFT_SUPPRESSED
    assert result.missed
    assert episode.consume_missed()
    assert actuator.calls == []


def test_terminal_episode_suppresses_before_the_law_or_the_aft_gate():
    episode = FinalApproachEpisodeLatch()
    episode.begin(_identity())
    assert episode.latch_docked(_contact())
    actuator = Actuator()
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(_DEFAULT_CONFIG))
    transaction = FinalApproachCommandTransaction(
        law, VisualPassDetector(), actuator, episode,
    )

    result = transaction.execute(_frame(1.0))

    assert result.outcome is FinalApproachCommandOutcome.TERMINAL_SUPPRESSED
    assert not result.missed
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
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(_DEFAULT_CONFIG))
    law.seed(_frame(1.0, control=_ray(yaw_deg=3.0, down_deg=5.0)))
    transaction = FinalApproachCommandTransaction(
        law, VisualPassDetector(), Actuator(), FinalApproachEpisodeLatch(),
    )

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
        RichPoi(_frame(1.0, control=_ray(yaw_deg=3.0, down_deg=5.0)))
    )
    _execute_one(runtime)
    assert runtime.nav(
        RichPoi(_frame(2.0, control=_ray(yaw_deg=5.0, down_deg=7.0)))
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
    poi = RichPoi(_frame(1.0, control=_ray(yaw_deg=5.0, down_deg=5.0)))
    confirmation = runtime._test_confirmation
    assert confirmation.can_confirm_detection(poi)
    assert confirmation.record_final_approach_confirmed_detection(poi)
    assert not confirmation.can_confirm_detection(poi)
    assert not confirmation.record_final_approach_confirmed_detection(poi)
    assert runtime.nav(poi)
    assert runtime.take_work() is None
    assert actuator.calls == []


def test_delayed_confirmation_uses_the_newer_visual_command_immediately():
    runtime, actuator, _, _ = _runtime()
    confirmation = runtime._test_confirmation
    reviewed = RichPoi(
        _frame(1.0, control=_ray(yaw_deg=5.0, down_deg=5.0))
    )
    delayed = RichPoi(
        _frame(2.0, control=_ray(yaw_deg=36.0, down_deg=5.0))
    )

    assert confirmation.can_confirm_detection(reviewed)
    assert confirmation.can_confirm_detection(delayed)
    assert confirmation.record_final_approach_confirmed_detection(delayed)
    assert actuator.calls == []

    next_frame = RichPoi(
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
            runtime.nav(RichPoi(_frame(ts, control=_ray(down_deg=down))))
            _execute_one(runtime)
        sequences.append(tuple(actuator.calls))
        cadence.close()
    assert sequences[1:] == [sequences[0], sequences[0], sequences[0]]
