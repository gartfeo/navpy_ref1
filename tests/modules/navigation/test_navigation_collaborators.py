"""Focused regression tests for Navigation's extracted collaborators."""

from __future__ import annotations

import math
import threading
import time
from contextlib import contextmanager, nullcontext
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from navpy.args.navigation_args import NavigationAlgorithm
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.navigation_command_session import NavigationCommandSession
from navpy.modules.navigation.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
    _next_fixed_deadline,
)
from navpy.modules.navigation.navigation import Navigation
from navpy.modules.navigation.navigation_lifecycle import NavigationLifecycle
from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.navigation_mode import (
    ActiveNavigationMode,
    NavigationModeBuilder,
    NavigationModeSelector,
    NavigationModeState,
    NavigationTerminalCapabilities,
    RuntimeBuildResult,
)
from navpy.modules.navigation.navigation_runtime import LegacyNavigationRuntime
from navpy.modules.navigation.navigation_terminal import TerminalNavigationService
from navpy.modules.navigation.legacy_destination_resolver import (
    LegacyNavigationState,
    LegacyDestinationResolver,
    geodetic_target_ned,
)
from navpy.modules.navigation.legacy_terminal_command import (
    LegacyTerminalCommand,
    LegacyTerminalCommandPorts,
)
from navpy.modules.navigation.nav.nav_law import NavCommand, NavCommandMode
from navpy.modules.navigation.nav.nav_law_factory import (
    NavAlgorithmSpec,
    get_nav_algorithm_spec,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from tests.detection_factory import make_detected_target


def _detection(*, simulation: bool = False):
    return make_detected_target(
        x_error=12.0,
        y_error=34.0,
        k=np.eye(3),
        g_data=GimbalData(att=Attitude(-20.0, 1.0, 2.0)),
        uas_att=Attitude(-3.0, 4.0, 5.0),
        c_g_loc=Location(40.0, 44.0, 1000.0, is_absolute=True),
        t_g_loc_debug=Location(40.001, 44.002, 900.0, is_absolute=True),
        reference_height_m=2.0,
        is_simulation=simulation,
    )


def _active_mode(
    algorithm: str,
    *,
    runtime=None,
    terminal=None,
) -> ActiveNavigationMode:
    return ActiveNavigationMode(
        spec=get_nav_algorithm_spec(algorithm),
        runtime=runtime or Mock(),
        terminal=terminal,
    )


def test_target_resolver_locks_the_projected_pixel_target():
    vehicle = Mock()
    args = SimpleNamespace(use_direct_target=False)
    pixel_ned = np.array([0.8, 0.1, 0.2])
    geo_ref = Mock(calc_ned=Mock(return_value=pixel_ned))
    projected_target = Location(40.003, 44.004, 850.0, is_absolute=True)
    zc_util = Mock(ray_to_terrain_ned=Mock(return_value=projected_target))
    state = LegacyNavigationState()
    target_resolver = LegacyDestinationResolver(vehicle, args, geo_ref, zc_util, state)

    detection = _detection()
    target_ned, attitude = target_resolver.resolve(detection)

    assert target_ned is pixel_ned
    assert attitude is detection.pose.aircraft_attitude
    zc_util.ray_to_terrain_ned.assert_called_once()
    assert target_resolver.locked_target is projected_target


def test_real_detection_never_uses_debug_target_for_direct_target_resolution():
    vehicle = Mock()
    args = SimpleNamespace(use_direct_target=True)
    pixel_ned = np.array([0.7, -0.2, 0.3])
    geo_ref = Mock(calc_ned=Mock(return_value=pixel_ned))
    state = LegacyNavigationState()
    state.set_locked_target(Location(
        40.01,
        44.01,
        800.0,
        is_absolute=True,
    ))
    target_resolver = LegacyDestinationResolver(vehicle, args, geo_ref, Mock(), state)

    target_ned, _ = target_resolver.resolve(_detection(simulation=False))

    assert target_ned is pixel_ned
    geo_ref.calc_ned.assert_called_once()
    vehicle.location.assert_not_called()


def test_sim_detection_uses_truth_for_explicit_legacy_direct_target():
    vehicle = Mock()
    current = Location(40.0, 44.0, 900.0, is_absolute=True)
    vehicle.location.return_value = current
    target_resolver = LegacyDestinationResolver(
        vehicle,
        SimpleNamespace(use_direct_target=True),
        Mock(),
        Mock(),
        LegacyNavigationState(),
    )
    detection = _detection(simulation=True)

    target_ned, attitude = target_resolver.resolve(detection)

    assert np.allclose(
        target_ned,
        geodetic_target_ned(current, detection.geo.truth_target_location),
    )
    assert attitude is detection.pose.aircraft_attitude


def test_legacy_terminal_command_preserves_command_before_log_order():
    order: list[str] = []
    current = Location(40.0, 44.0, 1000.0, is_absolute=True)
    locked = Location(40.001, 44.0, 900.0, is_absolute=True)
    vehicle = Mock()
    vehicle.location = Mock(return_value=current)
    vehicle.attitude = Attitude(-5.0, 20.0, 3.0)
    vehicle.set_attitude = Mock(side_effect=lambda *args, **kwargs: order.append("command"))
    geo_ref = Mock()
    geo_ref.calc_yaw_pitch_proj = Mock(return_value=(4.0, -2.0))
    geo_ref.calc_pitch_los = Mock(return_value=-6.0)
    nav = Mock()
    nav.calc = Mock(
        side_effect=lambda context: (
            order.append("nav"),
            NavCommand(
                NavCommandMode.ACTUATOR,
                cmd_roll_deg=8.0,
                cmd_pitch_deg=-10.0,
                cmd_thr=0.55,
            ),
        )[1]
    )
    navigation_logger = Mock()
    navigation_logger.log = Mock(side_effect=lambda **kwargs: order.append("log"))
    ports = LegacyTerminalCommandPorts(
        vehicle=vehicle,
        geo_ref=geo_ref,
        logger=Mock(),
        navigation_logger=navigation_logger,
        nav=nav,
        get_locked_target=Mock(return_value=locked),
        adjust_nav=Mock(side_effect=lambda *args: order.append("adjust") or False),
        is_adjusted=Mock(return_value=False),
        calc_nav_target=Mock(return_value=locked),
    )

    result = LegacyTerminalCommand(ports).execute(
        np.array([1.0, 0.0, 0.1]),
        _detection(simulation=False),
    )

    assert result is not None
    assert order == ["adjust", "nav", "command", "log"]
    roll, pitch = vehicle.set_attitude.call_args.args
    assert roll == math.radians(8.0)
    assert pitch == math.radians(-10.0)
    assert vehicle.set_attitude.call_args.kwargs == {"thr": 0.55}
    assert navigation_logger.log.call_args.kwargs["detect_t_loc_debug"] is None


def test_command_worker_finishes_work_when_execution_raises():
    stop_event = threading.Event()
    command_event = threading.Event()
    command_event.set()
    runtime = Mock()
    runtime.take_work = Mock(return_value="work")

    def fail_and_stop(work):
        assert work == "work"
        stop_event.set()
        raise RuntimeError("boom")

    runtime.execute_work = Mock(side_effect=fail_and_stop)
    logger = Mock()
    worker = NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=command_event,
            wall_period_s=Mock(return_value=0.004),
            runtime_session=lambda: nullcontext(runtime),
            logger=logger,
        )
    )

    worker.run()

    runtime.finish_work.assert_called_once_with("work")
    logger.error.assert_called_once()


def test_command_worker_scopes_process_cadence_to_command_work():
    stop_event = threading.Event()
    command_event = threading.Event()
    command_event.set()
    runtime = Mock()
    runtime.take_work.return_value = "work"
    runtime.has_command_pending_or_in_flight.return_value = True
    runtime.postprocess_job.return_value = None
    runtime.execute_work.side_effect = lambda _work: stop_event.set()
    cadence_transitions = []
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=lambda _period: 0.0,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        set_process_command_cadence_active=cadence_transitions.append,
        monotonic_s=lambda: 0.0,
        sleep_s=lambda _duration: None,
    ))

    worker.run()

    assert cadence_transitions == [True, False]


def test_finish_failure_cannot_strand_prepared_postprocess():
    stop_event = threading.Event()
    runtime = Mock()
    runtime.take_work.return_value = "work"
    postprocessed = threading.Event()

    def execute(_work):
        stop_event.set()

    runtime.execute_work.side_effect = execute
    runtime.postprocess_job.return_value = postprocessed.set
    runtime.finish_work.side_effect = RuntimeError("finish failed")
    logger = Mock()
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=threading.Event(),
        wall_period_s=lambda _period: 0.0,
        runtime_session=lambda: nullcontext(runtime),
        logger=logger,
        monotonic_s=lambda: 0.0,
        sleep_s=lambda _duration: None,
    ))

    worker.run()

    assert postprocessed.is_set()
    assert any(
        "Error finishing termination work" in call.args[0]
        for call in logger.error.call_args_list
    )


def test_command_worker_defaults_to_high_resolution_deadline_clock():
    ports = NavigationCommandWorkerPorts(
        stop_event=threading.Event(),
        command_event=threading.Event(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(Mock()),
        logger=Mock(),
    )

    assert ports.monotonic_s is time.perf_counter
    assert ports.sleep_s is time.sleep


def test_command_worker_issues_staged_work_before_preparing_next_source():
    stop_event = threading.Event()
    order: list[str] = []
    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = True
    runtime.take_work.side_effect = lambda: order.append("take") or "work"
    runtime.execute_work.side_effect = lambda _work: order.append("execute")
    runtime.finish_work.side_effect = lambda _work: order.append("finish")
    runtime.postprocess_job.return_value = None

    def dispatch_source() -> bool:
        order.append("source")
        stop_event.set()
        return True

    NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=threading.Event(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        source_dispatch=dispatch_source,
    )).run()

    assert order == ["take", "execute", "finish", "source"]


@pytest.mark.parametrize(
    ("first_source_duration_s", "expected_issue_s"),
    (
        (0.0015, (0.002, 0.004, 0.006)),
        (0.0025, (0.002, 0.006, 0.008)),
    ),
)
def test_source_prepare_preserves_cadence_without_history_replay(
    first_source_duration_s,
    expected_issue_s,
):
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

        def sleep(self, duration_s):
            self.now_s += duration_s

    class HotCommandEvent:
        @staticmethod
        def is_set():
            return True

        @staticmethod
        def wait(timeout):
            assert timeout == 0.0
            return True

    class AdvancingStopEvent:
        def __init__(self, clock):
            self._clock = clock
            self._set = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        def wait(self, timeout):
            self._clock.now_s += timeout
            return self._set

    clock = FakeClock()
    stop_event = AdvancingStopEvent(clock)
    source_durations_s = iter((first_source_duration_s, 0.0, 0.0015))
    current_work = {"value": "held-0"}
    source_calls = 0

    def dispatch_source():
        nonlocal source_calls
        source_calls += 1
        clock.now_s += next(source_durations_s)
        current_work["value"] = f"fresh-{source_calls}"
        return True

    issued_at = []
    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = True
    runtime.take_work.side_effect = lambda: current_work["value"]
    runtime.postprocess_job.return_value = None

    def execute(work):
        issued_at.append((clock.monotonic(), work))
        clock.now_s += 0.0001
        if len(issued_at) == 3:
            stop_event.set()

    runtime.execute_work.side_effect = execute
    NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=HotCommandEvent(),
        wall_period_s=lambda _period_s: 0.002,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        source_dispatch=dispatch_source,
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    )).run()

    assert [wall_s for wall_s, _work in issued_at] == pytest.approx(
        expected_issue_s
    )
    assert [work for _wall_s, work in issued_at] == [
        "held-0",
        "fresh-1",
        "fresh-2",
    ]
    assert source_calls == 2


def test_command_worker_submits_postprocess_before_preparing_next_source():
    stop_event = threading.Event()
    order: list[str] = []
    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = True
    runtime.take_work.return_value = "staged-work"
    runtime.execute_work.side_effect = lambda _work: order.append("issue")
    runtime.postprocess_job.return_value = lambda: order.append("postprocess")

    def dispatch_source():
        order.append("source")
        stop_event.set()
        return True

    NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=threading.Event(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        source_dispatch=dispatch_source,
    )).run()

    assert order == ["issue", "postprocess", "source"]


def test_command_worker_does_not_dispatch_source_after_stop_wakeup():
    stop_event = threading.Event()
    source_dispatch = Mock()
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=threading.Event(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(Mock()),
        logger=Mock(),
        source_dispatch=source_dispatch,
    ))

    def stop_after_wait(_deadline_s):
        stop_event.set()
        return False, False

    with patch.object(
        worker,
        "_wait_for_deadline",
        side_effect=stop_after_wait,
    ):
        worker.run()

    source_dispatch.assert_not_called()


def test_signaled_samples_cannot_bypass_fixed_autopilot_command_cadence():
    stop_event = threading.Event()
    command_event = threading.Event()
    command_event.set()  # Permanently hot source: every wait wakes immediately.
    issued_at = []
    runtime = Mock()
    runtime.take_work.return_value = "latest-work"

    def execute_and_stop(work):
        assert work == "latest-work"
        issued_at.append(time.perf_counter())
        if len(issued_at) == 3:
            stop_event.set()

    runtime.execute_work.side_effect = execute_and_stop
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
    ))

    worker.run()

    gaps = [later - earlier for earlier, later in zip(issued_at, issued_at[1:])]
    assert len(gaps) == 2
    assert min(gaps) >= 0.015


def test_command_execution_time_does_not_accumulate_cadence_drift():
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

        def sleep(self, duration_s):
            self.now_s += duration_s

    class HotCommandEvent:
        @staticmethod
        def wait(timeout):
            assert timeout >= 0.0
            return True

    class AdvancingStopEvent:
        def __init__(self, clock):
            self._clock = clock
            self._set = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        def wait(self, timeout):
            self._clock.now_s += timeout
            return self._set

    clock = FakeClock()
    stop_event = AdvancingStopEvent(clock)
    issued_at = []
    runtime = Mock()
    runtime.take_work.return_value = "latest-work"

    def execute_and_stop(_work):
        issued_at.append(clock.monotonic())
        clock.now_s += 0.01  # Nonzero work still fits inside the 20 ms slot.
        if len(issued_at) == 3:
            stop_event.set()

    runtime.execute_work.side_effect = execute_and_stop
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=HotCommandEvent(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    ))

    worker.run()

    assert issued_at == pytest.approx([0.02, 0.04, 0.06])


@pytest.mark.parametrize(
    ("speedup", "expected_wall_s"),
    ((1.0, [0.02, 0.04, 0.06]), (10.0, [0.002, 0.004, 0.006])),
)
def test_fixed_worker_maps_ap_slots_without_scaling_source_timestamps(
    speedup,
    expected_wall_s,
):
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

    clock = FakeClock()
    stop_event = threading.Event()
    command_event = threading.Event()
    slot = NavigationCommandSlot(threading.RLock(), command_event)
    next_source_frame_s = 0.02
    issued = []

    def sleep_and_publish(duration_s):
        nonlocal next_source_frame_s
        deadline_s = clock.now_s + duration_s
        source_deadline_s = deadline_s * speedup
        while next_source_frame_s <= source_deadline_s + 1e-12:
            slot.replace(next_source_frame_s)
            slot.signal_pending()
            next_source_frame_s += 0.02
        clock.now_s = deadline_s

    runtime = Mock()
    runtime.has_command_pending_or_in_flight.side_effect = (
        slot.has_pending_or_in_flight
    )
    runtime.take_work.side_effect = lambda: slot.take_or_else(lambda: None)

    def execute(lease):
        def record():
            issued.append((clock.monotonic(), lease.payload))
            if len(issued) == 3:
                stop_event.set()

        slot.execute_if_current(lease, record)

    runtime.execute_work.side_effect = execute
    runtime.finish_work.side_effect = slot.finish
    wall_period = Mock(side_effect=lambda period_s: period_s / speedup)
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=wall_period,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=sleep_and_publish,
    ))

    worker.run()

    assert [wall for wall, _source in issued] == pytest.approx(expected_wall_s)
    assert [source for _wall, source in issued] == pytest.approx(
        [0.02, 0.04, 0.06]
    )
    assert all(call.args == (0.02,) for call in wall_period.call_args_list)


def _fixed_clock_starts_for_execution(execution_s):
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

        def sleep(self, duration_s):
            self.now_s += duration_s

    class HotCommandEvent:
        @staticmethod
        def wait(timeout):
            return True

    class AdvancingStopEvent:
        def __init__(self, clock):
            self._clock = clock
            self._set = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        def wait(self, timeout):
            self._clock.now_s += timeout
            return self._set

    clock = FakeClock()
    stop_event = AdvancingStopEvent(clock)
    issued_at = []
    runtime = Mock()
    runtime.take_work.return_value = "latest-work"

    def execute_twice(_work):
        issued_at.append(clock.monotonic())
        clock.now_s += execution_s
        if len(issued_at) == 2:
            stop_event.set()

    runtime.execute_work.side_effect = execute_twice
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=HotCommandEvent(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=clock.sleep,
    ))
    worker.run()
    return issued_at


def test_execution_completing_exactly_at_next_slot_preserves_cadence():
    assert _fixed_clock_starts_for_execution(0.02) == pytest.approx(
        [0.02, 0.04]
    )


def test_execution_overrun_skips_missed_slot_without_grid_drift():
    assert _fixed_clock_starts_for_execution(0.03) == pytest.approx(
        [0.02, 0.06]
    )


def test_partial_late_wakeup_keeps_the_next_nominal_ap_slot():
    next_deadline = _next_fixed_deadline(
        scheduled_s=0.040,
        completed_s=0.0395,
        period_s=0.020,
    )

    assert next_deadline == pytest.approx(0.040)


@pytest.mark.parametrize(
    ("started", "completed", "expected"),
    [
        (0.020, 0.025, 0.040),
        (0.020, 0.040, 0.040),
        (0.039, 0.0395, 0.040),
        (0.039, 0.041, 0.060),
        (0.040, 0.0405, 0.060),
        (0.020, 0.050, 0.060),
        (0.071, 0.0715, 0.080),
    ],
)
def test_next_deadline_preserves_grid_and_skips_missed_slots(
    started,
    completed,
    expected,
):
    deadline = _next_fixed_deadline(
        scheduled_s=0.040,
        completed_s=completed,
        period_s=0.020,
    )

    assert deadline == pytest.approx(expected)
    assert deadline >= completed
    assert (deadline - 0.040) / 0.020 == pytest.approx(
        round((deadline - 0.040) / 0.020)
    )


def test_idle_ticks_advance_deadlines_without_zero_timeout_spin():
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

    class StopEvent:
        def __init__(self):
            self.set_flag = False

        def is_set(self):
            return self.set_flag

        def set(self):
            self.set_flag = True

        @staticmethod
        def wait(_timeout):
            return False

    clock = FakeClock()
    stop_event = StopEvent()
    event_timeouts = []
    sleep_timeouts = []

    def sleep_to_deadline(timeout):
        sleep_timeouts.append(timeout)
        clock.now_s += timeout

    class IdleCommandEvent:
        def wait(self, timeout):
            event_timeouts.append(timeout)
            if len(event_timeouts) == 3:
                stop_event.set()
            return False

    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = False
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=IdleCommandEvent(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=sleep_to_deadline,
    ))

    worker.run()

    assert sleep_timeouts == pytest.approx([0.02, 0.02, 0.02])
    assert event_timeouts == pytest.approx([0.0, 0.0, 0.0])
    runtime.take_work.assert_not_called()


def test_early_sleep_return_is_retried_without_early_or_widened_slot():
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

    class SampledCommandEvent:
        def __init__(self):
            self.timeouts = []

        def wait(self, timeout):
            self.timeouts.append(timeout)
            return False

    class EarlyFalseStopEvent:
        def __init__(self):
            self._set = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        @staticmethod
        def wait(_timeout):
            return False

    clock = FakeClock()
    command_event = SampledCommandEvent()
    stop_event = EarlyFalseStopEvent()
    issued_at = []
    sleep_timeouts = []

    def early_once_sleep(timeout):
        sleep_timeouts.append(timeout)
        if len(sleep_timeouts) > 1:
            clock.now_s += timeout

    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = True
    runtime.take_work.return_value = "retained-work"

    def execute_twice(_work):
        issued_at.append(clock.monotonic())
        if len(issued_at) == 2:
            stop_event.set()

    runtime.execute_work.side_effect = execute_twice
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=early_once_sleep,
    ))

    worker.run()

    assert issued_at == pytest.approx([0.02, 0.04])
    assert sleep_timeouts == pytest.approx([0.02, 0.02, 0.02])
    assert command_event.timeouts == pytest.approx([0.0, 0.0])


def test_late_timer_wakeup_resumes_on_the_next_nominal_ap_slot():
    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

    class HotCommandEvent:
        @staticmethod
        def wait(timeout):
            return True

    class OversleepingStopEvent:
        def __init__(self, clock):
            self._clock = clock
            self._set = False
            self._overslept = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        def wait(self, timeout):
            oversleep_s = 0.019 if not self._overslept else 0.0
            self._overslept = True
            self._clock.now_s += timeout + oversleep_s
            return self._set

    clock = FakeClock()
    stop_event = OversleepingStopEvent(clock)
    issued_at = []

    def oversleep_once(timeout):
        oversleep_s = 0.019 if not stop_event._overslept else 0.0
        stop_event._overslept = True
        clock.now_s += timeout + oversleep_s

    runtime = Mock()
    runtime.take_work.return_value = "latest-work"

    def execute_twice(_work):
        issued_at.append(clock.monotonic())
        if len(issued_at) == 2:
            stop_event.set()

    runtime.execute_work.side_effect = execute_twice
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=HotCommandEvent(),
        wall_period_s=lambda period_s: period_s,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=oversleep_once,
    ))

    worker.run()

    assert issued_at == pytest.approx([0.039, 0.04])


def test_command_slot_invalidates_stale_lease_without_hiding_in_flight_work():
    slot = NavigationCommandSlot(threading.RLock(), threading.Event())
    slot.replace("old")
    lease = slot.take_or_else(lambda: None)

    assert lease is not None
    assert slot.has_pending_or_in_flight()
    slot.invalidate()
    assert slot.has_pending_or_in_flight()
    assert slot.execute_if_current(lease, lambda: "issued") is None

    slot.finish(lease)
    assert not slot.has_pending_or_in_flight()


def test_command_slot_stale_finish_cannot_clear_a_newer_lease():
    slot = NavigationCommandSlot(threading.RLock(), threading.Event())
    slot.replace("old")
    old_lease = slot.take_or_else(lambda: None)
    assert old_lease is not None
    slot.invalidate()
    slot.finish(old_lease)

    slot.replace("new")
    new_lease = slot.take_or_else(lambda: None)
    assert new_lease is not None
    slot.finish(old_lease)
    assert slot.has_pending_or_in_flight()

    slot.finish(new_lease)
    assert not slot.has_pending_or_in_flight()


def test_legacy_runtime_clears_wake_signal_before_retained_stream_work():
    event = threading.Event()
    slot = NavigationCommandSlot(threading.RLock(), event)
    target_ned = np.array([1.0, 0.0, 0.0])
    detection = _detection()
    runtime = LegacyNavigationRuntime(
        command_slot=slot,
        target_resolver=Mock(return_value=(
            target_ned,
            detection.pose.aircraft_attitude,
        )),
        command_executor=Mock(),
        reset_law=Mock(),
    )

    assert runtime.nav(detection)
    assert event.is_set()
    first = runtime.take_work()

    assert first[0] is target_ned
    assert first[1] is detection
    assert not event.is_set()
    assert runtime.take_work() is first


def test_mode_selector_handoffs_when_the_configured_algorithm_changes():
    args = SimpleNamespace(
        navigation_algorithm=NavigationAlgorithm.PN.value,
        pitch_controller="pn",
        pitch_args=SimpleNamespace(kp=1.25),
        refresh=Mock(),
    )
    state = NavigationModeState()
    logger = Mock()
    navs = {
        NavigationAlgorithm.PN: Mock(),
        NavigationAlgorithm.VISION_NAV_PN: Mock(),
    }
    runtimes = {
        NavigationAlgorithm.PN: Mock(),
        NavigationAlgorithm.VISION_NAV_PN: Mock(),
    }

    def spec(algorithm, log_name):
        return NavAlgorithmSpec(
            algorithm=algorithm,
            log_name=log_name,
            log_message=f"using {log_name}",
            kp_provider=(
                (lambda config: config.pitch_args.kp)
                if algorithm is NavigationAlgorithm.PN
                else (lambda config: None)
            ),
        )

    builders = {
        algorithm: NavigationModeBuilder(
            algorithm_spec,
            lambda algorithm=algorithm: navs[algorithm],
            lambda nav, algorithm=algorithm: RuntimeBuildResult(
                runtimes[algorithm]
            ),
        )
        for algorithm, algorithm_spec in (
            (NavigationAlgorithm.PN, spec(NavigationAlgorithm.PN, "pn")),
            (
                NavigationAlgorithm.VISION_NAV_PN,
                spec(NavigationAlgorithm.VISION_NAV_PN, "vision-nav-pn"),
            ),
        )
    }
    selector = NavigationModeSelector(args, builders, state, logger)
    previous = selector.refresh(force=True)

    args.navigation_algorithm = NavigationAlgorithm.VISION_NAV_PN.value
    active = selector.initialize()

    assert active.spec.algorithm is NavigationAlgorithm.VISION_NAV_PN
    previous.runtime.invalidate_commands.assert_called_once_with()
    navs[NavigationAlgorithm.PN].reset.assert_not_called()
    navs[NavigationAlgorithm.VISION_NAV_PN].reset.assert_not_called()
    assert selector.algorithm_info == ("vision-nav-pn", None)


def test_mode_switch_resets_before_publish_and_blocks_navigation_ingress():
    state = NavigationModeState()
    old_runtime = Mock()
    old = _active_mode("pn", runtime=old_runtime)
    state.install_initialized(old, expected_previous=None)

    reset_entered = threading.Event()
    release_reset = threading.Event()

    def invalidate_old() -> None:
        reset_entered.set()
        assert release_reset.wait(timeout=2.0)

    old_runtime.invalidate_commands.side_effect = invalidate_old
    candidate_runtime = Mock()
    candidate_runtime.nav.return_value = True
    candidate = _active_mode(
        "vision-nav-pn",
        runtime=candidate_runtime,
    )
    switch_errors = []

    def switch_mode() -> None:
        try:
            state.install_initialized(candidate, expected_previous=old)
        except BaseException as exc:  # surfaced in the owning test thread
            switch_errors.append(exc)

    switch_thread = threading.Thread(target=switch_mode)
    switch_thread.start()
    assert reset_entered.wait(timeout=2.0)
    assert state._active is old

    navigation = Navigation.__new__(Navigation)
    navigation._mode_state = state
    nav_started = threading.Event()
    nav_finished = threading.Event()
    nav_results = []

    def nav_during_switch() -> None:
        nav_started.set()
        nav_results.append(navigation.nav(_detection()))
        nav_finished.set()

    nav_thread = threading.Thread(target=nav_during_switch)
    nav_thread.start()
    assert nav_started.wait(timeout=2.0)
    assert not nav_finished.wait(timeout=0.05)
    old.runtime.nav.assert_not_called()
    candidate_runtime.nav.assert_not_called()

    release_reset.set()
    switch_thread.join(timeout=2.0)
    nav_thread.join(timeout=2.0)

    assert not switch_thread.is_alive()
    assert not nav_thread.is_alive()
    assert switch_errors == []
    assert state.snapshot() is candidate
    assert nav_results == [True]
    old.runtime.invalidate_commands.assert_called_once_with()
    candidate_runtime.nav.assert_called_once()


def test_terminal_call_uses_one_mode_session_during_switch():
    state = NavigationModeState()
    old_terminal = NavigationTerminalCapabilities(
        status=Mock(),
        confirmation=Mock(),
    )
    old = _active_mode("pn", terminal=old_terminal)
    state.install_initialized(old, expected_previous=None)

    reset_entered = threading.Event()
    release_reset = threading.Event()

    def invalidate_old() -> None:
        reset_entered.set()
        assert release_reset.wait(timeout=2.0)

    old.runtime.invalidate_commands.side_effect = invalidate_old
    candidate_terminal = NavigationTerminalCapabilities(
        status=Mock(),
        confirmation=Mock(),
    )
    candidate_terminal.confirmation.can_confirm_detection.return_value = True
    candidate = _active_mode(
        "vision-nav-pn",
        terminal=candidate_terminal,
    )
    switch_thread = threading.Thread(
        target=lambda: state.install_initialized(
            candidate,
            expected_previous=old,
        )
    )
    switch_thread.start()
    assert reset_entered.wait(timeout=2.0)

    result = []
    terminal_call_finished = threading.Event()

    def call_terminal() -> None:
        result.append(
            TerminalNavigationService(state).can_confirm_detection(_detection())
        )
        terminal_call_finished.set()

    terminal_thread = threading.Thread(target=call_terminal)
    terminal_thread.start()
    assert not terminal_call_finished.wait(timeout=0.05)
    old_terminal.confirmation.can_confirm_detection.assert_not_called()
    candidate_terminal.confirmation.can_confirm_detection.assert_not_called()

    release_reset.set()
    switch_thread.join(timeout=2.0)
    terminal_thread.join(timeout=2.0)

    assert not switch_thread.is_alive()
    assert not terminal_thread.is_alive()
    assert result == [True]
    candidate_terminal.confirmation.can_confirm_detection.assert_called_once()


def test_navigation_only_admits_work_for_the_command_worker():
    state = NavigationModeState()
    runtime = Mock()
    runtime.nav.return_value = True
    state.install_initialized(
        _active_mode("vision-nav-pn", runtime=runtime),
        expected_previous=None,
    )
    navigation = Navigation.__new__(Navigation)
    navigation._mode_state = state

    assert navigation.nav(_detection()) is True

    runtime.nav.assert_called_once()
    runtime.take_work.assert_not_called()
    runtime.execute_work.assert_not_called()
    runtime.finish_work.assert_not_called()


def test_terminal_navigation_ingress_remains_serialized_with_command_runtime():
    state = NavigationModeState()
    nav_entered = threading.Event()
    release_nav = threading.Event()
    runtime = Mock()

    def block_nav(_detection_data):
        nav_entered.set()
        assert release_nav.wait(timeout=2.0)
        return True

    runtime.nav.side_effect = block_nav
    terminal = NavigationTerminalCapabilities(status=Mock(), confirmation=Mock())
    state.install_initialized(
        _active_mode(
            "vision-nav-pn",
            runtime=runtime,
            terminal=terminal,
        ),
        expected_previous=None,
    )
    navigation = Navigation.__new__(Navigation)
    navigation._mode_state = state
    nav_thread = threading.Thread(target=lambda: navigation.nav(_detection()))
    nav_thread.start()
    assert nav_entered.wait(timeout=2.0)

    command_entered = threading.Event()

    def enter_command_session() -> None:
        with state.runtime_session() as selected:
            assert selected is runtime
            command_entered.set()

    command_thread = threading.Thread(target=enter_command_session)
    command_thread.start()
    try:
        assert not command_entered.wait(timeout=0.05)
    finally:
        release_nav.set()
        nav_thread.join(timeout=2.0)
        command_thread.join(timeout=2.0)

    assert not nav_thread.is_alive()
    assert not command_thread.is_alive()
    assert command_entered.is_set()


def test_navigation_construction_defers_worker_start_until_explicit_start():
    lifecycle = Mock()
    composition = SimpleNamespace(
        lifecycle=lifecycle,
        mode_state=Mock(),
        terminal=Mock(),
        legacy_targets=Mock(),
        vehicle_commands=Mock(),
        bind_source_dispatch=Mock(),
    )

    with patch(
        "navpy.modules.navigation.navigation.compose_navigation",
        return_value=composition,
    ):
        navigation = Navigation(
            Mock(),
            Mock(),
            None,
            Mock(),
            Mock(),
            SimpleNamespace(use_terrain=False),
        )

    lifecycle.start.assert_not_called()

    navigation.start()

    lifecycle.start.assert_called_once_with()

    callback = Mock(return_value=False)
    navigation.bind_terminal_source_dispatch(callback)
    # The observer travels with the callback because it belongs to the same
    # owner. A source without one publishes None, and the worker keeps the
    # no-op observer it was constructed with.
    composition.bind_source_dispatch.assert_called_once_with(callback, None)


def test_mode_switch_invalidates_old_slot_before_candidate_reset_and_publish():
    event = threading.Event()
    slot = NavigationCommandSlot(threading.RLock(), event)
    old_runtime = LegacyNavigationRuntime(
        command_slot=slot,
        target_resolver=Mock(),
        command_executor=Mock(),
        reset_law=Mock(),
    )
    state = NavigationModeState()
    old = _active_mode("pn", runtime=old_runtime)
    state.install_initialized(old, expected_previous=None)

    slot.replace((np.array([1.0, 0.0, 0.0]), _detection()))
    stale_lease = slot.take_or_else(lambda: None)
    assert stale_lease is not None
    slot.replace((np.array([2.0, 0.0, 0.0]), _detection()))
    slot.signal_pending()
    assert event.is_set()

    original_invalidate = old_runtime.invalidate_commands

    def assert_old_work_is_fenced() -> None:
        original_invalidate()
        assert slot.peek() is None
        assert not event.is_set()
        assert slot.execute_if_current(stale_lease, lambda: True) is None
        assert state.snapshot() is old

    old_runtime.invalidate_commands = assert_old_work_is_fenced
    candidate = _active_mode(
        "vision-nav-pn",
        runtime=Mock(),
    )

    state.install_initialized(candidate, expected_previous=old)
    slot.finish(stale_lease)

    assert state.snapshot() is candidate


def test_worker_mode_session_finishes_old_work_before_switch_reset():
    state = NavigationModeState()
    stop_event = threading.Event()
    command_event = threading.Event()
    command_event.set()
    execute_entered = threading.Event()
    release_execute = threading.Event()
    switch_started = threading.Event()
    old_invalidated = threading.Event()
    order = []

    old_runtime = Mock()

    def take_old_work():
        command_event.clear()
        return "old-work"

    def execute_old_work(work):
        assert work == "old-work"
        order.append("old-execute")
        execute_entered.set()
        assert release_execute.wait(timeout=2.0)
        order.append("old-command-finished")
        stop_event.set()

    old_runtime.take_work.side_effect = take_old_work
    old_runtime.execute_work.side_effect = execute_old_work
    old_runtime.finish_work.side_effect = lambda work: order.append("old-finish")
    old = _active_mode("pn", runtime=old_runtime)
    state.install_initialized(old, expected_previous=None)

    def invalidate_old() -> None:
        order.append("old-invalidated")
        old_invalidated.set()

    old_runtime.invalidate_commands.side_effect = invalidate_old
    candidate = _active_mode(
        "vision-nav-pn",
        runtime=Mock(),
    )
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=lambda period_s: period_s,
        runtime_session=state.runtime_session,
        logger=Mock(),
    ))
    worker_thread = threading.Thread(target=worker.run)
    worker_thread.start()
    assert execute_entered.wait(timeout=2.0)

    def switch_mode() -> None:
        switch_started.set()
        state.install_initialized(candidate, expected_previous=old)

    switch_thread = threading.Thread(target=switch_mode)
    switch_thread.start()
    assert switch_started.wait(timeout=2.0)
    assert not old_invalidated.wait(timeout=0.05)

    release_execute.set()
    worker_thread.join(timeout=2.0)
    switch_thread.join(timeout=2.0)

    assert not worker_thread.is_alive()
    assert not switch_thread.is_alive()
    assert order.index("old-finish") < order.index("old-invalidated")
    assert state.snapshot() is candidate


def test_worker_rechecks_stop_after_entering_runtime_session():
    stop_event = threading.Event()
    command_event = threading.Event()
    command_event.set()
    session_entered = threading.Event()
    release_session = threading.Event()
    runtime = Mock()

    @contextmanager
    def blocked_runtime_session():
        session_entered.set()
        assert release_session.wait(timeout=2.0)
        yield runtime

    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=command_event,
        wall_period_s=lambda period_s: period_s,
        runtime_session=blocked_runtime_session,
        logger=Mock(),
    ))
    worker_thread = threading.Thread(target=worker.run)
    worker_thread.start()
    assert session_entered.wait(timeout=2.0)

    stop_event.set()
    release_session.set()
    worker_thread.join(timeout=2.0)

    assert not worker_thread.is_alive()
    runtime.has_command_pending_or_in_flight.assert_not_called()
    runtime.take_work.assert_not_called()
    runtime.execute_work.assert_not_called()


def test_retained_worker_uses_fixed_autopilot_period_through_wall_adapter():
    wait_timeouts = []
    scheduler_periods = []

    class FakeClock:
        now_s = 0.0

        def monotonic(self):
            return self.now_s

    class AdvancingStopEvent:
        def __init__(self, clock):
            self._clock = clock
            self._set = False

        def is_set(self):
            return self._set

        def set(self):
            self._set = True

        def wait(self, timeout):
            self._clock.now_s += timeout
            return self._set

    clock = FakeClock()
    stop_event = AdvancingStopEvent(clock)
    sleep_timeouts = []

    def sleep_to_deadline(timeout):
        sleep_timeouts.append(timeout)
        clock.now_s += timeout

    class SequencedEvent:
        def wait(self, timeout):
            wait_timeouts.append(timeout)
            if len(wait_timeouts) == 1:
                return True
            clock.now_s += timeout
            return False

    runtime = Mock()
    runtime.has_command_pending_or_in_flight.return_value = True
    runtime.take_work.return_value = "retained-work"

    def execute_twice(work):
        assert work == "retained-work"
        if runtime.execute_work.call_count == 2:
            stop_event.set()

    runtime.execute_work.side_effect = execute_twice

    def wall_period(period_s):
        scheduler_periods.append(period_s)
        return period_s / 10.0

    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop_event,
        command_event=SequencedEvent(),
        wall_period_s=wall_period,
        runtime_session=lambda: nullcontext(runtime),
        logger=Mock(),
        monotonic_s=clock.monotonic,
        sleep_s=sleep_to_deadline,
    ))

    worker.run()

    assert scheduler_periods == [0.02, 0.02, 0.02]
    assert sleep_timeouts == pytest.approx([0.002, 0.002])
    assert wait_timeouts == pytest.approx([0.0, 0.0])
    assert runtime.execute_work.call_count == 2


def test_command_session_uses_captured_autopilot_scheduler_rate():
    session = NavigationCommandSession(
        threading.Event(),
        lambda: nullcontext(Mock()),
        lambda period_s: period_s,
        Mock(),
        scheduler_rate_hz=100.0,
    )

    assert session._worker._ports.scheduler_period_s == pytest.approx(0.01)


@pytest.mark.parametrize("rate_hz", [True, 0.0, -1.0, float("nan"), float("inf")])
def test_command_session_rejects_invalid_autopilot_scheduler_rate(rate_hz):
    with pytest.raises(ValueError, match="finite and positive"):
        NavigationCommandSession(
            threading.Event(),
            lambda: nullcontext(Mock()),
            lambda period_s: period_s,
            Mock(),
            scheduler_rate_hz=rate_hz,
        )


def test_lifecycle_orders_reset_before_start_and_join_before_logger_close():
    order = []
    state = NavigationModeState()
    runtime = Mock()
    runtime.invalidate_commands.side_effect = lambda: order.append("invalidate")
    runtime.reset_phase.side_effect = lambda: order.append("phase-reset")
    active = _active_mode("pn", runtime=runtime)
    state.install_initialized(active, expected_previous=None)
    order.clear()

    selector = Mock()
    selector.algorithm_info = ("pn", 1.25)
    selector.initialize.side_effect = lambda: order.append("select") or active
    legacy_state = Mock()
    legacy_state.reset.side_effect = lambda: order.append("legacy-reset")
    snap = Mock()
    navigation_logger = Mock()
    navigation_logger.write_summary_and_reset.side_effect = (
        lambda **kwargs: order.append("summary") or snap
    )
    navigation_logger.close.side_effect = lambda: order.append("logger-close")
    command_session = Mock()
    command_session.start.side_effect = lambda: order.append("worker-start")
    command_session.stop.side_effect = lambda: order.append("worker-stop")
    lifecycle = NavigationLifecycle(
        state,
        selector,
        legacy_state,
        navigation_logger,
        command_session,
    )

    assert lifecycle.start() is snap
    assert order == [
        "invalidate",
        "summary",
        "legacy-reset",
        "select",
        "phase-reset",
        "worker-start",
    ]

    lifecycle.stop()
    assert order[-2:] == [
        "worker-stop",
        "logger-close",
    ]


def test_lifecycle_summary_failure_leaves_retained_command_invalidated():
    state = NavigationModeState()
    pending = True
    runtime = Mock()

    def invalidate():
        nonlocal pending
        pending = False

    runtime.invalidate_commands.side_effect = invalidate
    runtime.has_command_pending_or_in_flight.side_effect = lambda: pending
    runtime.take_work.side_effect = lambda: object() if pending else None
    active = _active_mode("pn", runtime=runtime)
    state.install_initialized(active, expected_previous=None)
    selector = Mock()
    selector.algorithm_info = ("pn", 1.0)
    navigation_logger = Mock()
    navigation_logger.write_summary_and_reset.side_effect = RuntimeError(
        "summary failed"
    )
    lifecycle = NavigationLifecycle(
        state,
        selector,
        Mock(),
        navigation_logger,
        Mock(),
    )

    with pytest.raises(RuntimeError, match="summary failed"):
        lifecycle.reset()

    assert not runtime.has_command_pending_or_in_flight()
    assert runtime.take_work() is None
    selector.initialize.assert_not_called()


def test_lifecycle_stop_waits_for_in_progress_reset():
    state = NavigationModeState()
    active = _active_mode("pn")
    state.install_initialized(active, expected_previous=None)
    summary_entered = threading.Event()
    release_summary = threading.Event()
    worker_stop_entered = threading.Event()

    navigation_logger = Mock()

    def blocked_summary(**kwargs):
        summary_entered.set()
        assert release_summary.wait(timeout=2.0)
        return Mock()

    navigation_logger.write_summary_and_reset.side_effect = blocked_summary
    selector = Mock()
    selector.algorithm_info = ("pn", 1.0)
    command_session = Mock()
    command_session.stop.side_effect = worker_stop_entered.set
    lifecycle = NavigationLifecycle(
        state,
        selector,
        Mock(),
        navigation_logger,
        command_session,
    )
    reset_thread = threading.Thread(target=lifecycle.reset)
    reset_thread.start()
    assert summary_entered.wait(timeout=2.0)

    stop_thread = threading.Thread(target=lifecycle.stop)
    stop_thread.start()
    assert not worker_stop_entered.wait(timeout=0.05)

    release_summary.set()
    reset_thread.join(timeout=2.0)
    stop_thread.join(timeout=2.0)

    assert not reset_thread.is_alive()
    assert not stop_thread.is_alive()
    assert worker_stop_entered.is_set()
    command_session.stop.assert_called_once_with()
    navigation_logger.close.assert_called_once_with()


def test_command_session_stop_is_bounded_and_retryable():
    entered = threading.Event()
    release = threading.Event()
    session = NavigationCommandSession(
        threading.Event(),
        lambda: nullcontext(Mock()),
        lambda period_s: period_s,
        Mock(),
    )

    def block():
        entered.set()
        assert release.wait(timeout=1.0)

    session._worker.run = block
    session.start()
    assert entered.wait(timeout=1.0)

    with patch(
        "navpy.modules.navigation.navigation_command_session."
        "NAVIGATION_COMMAND_JOIN_TIMEOUT_S",
        0.01,
    ):
        started_s = time.perf_counter()
        with pytest.raises(TimeoutError, match="runtime remains owned"):
            session.stop()
        assert time.perf_counter() - started_s < 0.2

    release.set()
    session.stop()


def test_lifecycle_defers_navigation_logger_until_command_worker_stops():
    command_session = Mock()
    command_session.stop.side_effect = [TimeoutError("worker live"), None]
    navigation_logger = Mock()
    lifecycle = NavigationLifecycle(
        Mock(),
        Mock(),
        Mock(),
        navigation_logger,
        command_session,
    )

    with pytest.raises(TimeoutError, match="worker live"):
        lifecycle.stop()
    navigation_logger.close.assert_not_called()

    lifecycle.stop()
    navigation_logger.close.assert_called_once_with()
