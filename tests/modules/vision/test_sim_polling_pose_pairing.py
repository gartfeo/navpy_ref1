"""Finite-FOV sim camera renders only exact ATTITUDE/GLOBAL_POSITION_INT pairs."""

from __future__ import annotations

import math
import threading
from types import SimpleNamespace
from typing import Optional
from unittest.mock import Mock, patch

import pytest

from navpy.core.approach.vision_nav.command_freshness import (
    FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
)
from navpy.core.kernel.attitude import Attitude
from navpy.plugins.vehicle.ardupilot.pose_telemetry import (
    attitude_sample_from_message,
    position_sample_from_message,
)
from navpy.sim.detection.pose_frame_clock import PoseFrameClock
from navpy.sim.detection.scheduler_cadence_lease import SchedulerCadenceLease
from navpy.sim.detection.sim_detector_execution import SimDetectorExecution
from navpy.sim.detection.sim_polling_pose_reader import (
    PollingPoseRead,
    SimPollingPoseReader,
    whole_boot_ms,
)
from navpy.sim.detection.sim_source_composition import build_polling_reader
from navpy.sim.detection.unpaired_pose_monitor import UnpairedPoseMonitor


BUDGET_S = 0.05
POLL_WALL_PERIOD_S = 0.04


def _attitude(boot_ms: int, receipt_s: float = 100.0) -> SimpleNamespace:
    return SimpleNamespace(
        attitude=Attitude(-3.0, 120.0, 12.0),
        time_boot_s=float(boot_ms) * 1e-3,
        receipt_time_s=receipt_s,
        body_rates_rad_s=(0.01, -0.02, 0.03),
    )


def _position(boot_ms: int, receipt_s: float = 100.0) -> SimpleNamespace:
    return SimpleNamespace(
        lat=40.1,
        lon=44.2,
        alt=1250.0,
        time_boot_s=float(boot_ms) * 1e-3,
        receipt_time_s=receipt_s,
    )


def _reader(
    attitude: Optional[SimpleNamespace],
    position: Optional[SimpleNamespace],
    budget_s: float = BUDGET_S,
) -> SimPollingPoseReader:
    return SimPollingPoseReader(
        attitude_sample=lambda: attitude,
        position_sample=lambda: position,
        air_speed=lambda: 22.0,
        maximum_receipt_skew_s=lambda: budget_s,
    )


# --- reader -----------------------------------------------------------------


def test_equal_boot_times_give_the_pair() -> None:
    attitude = _attitude(12_340)
    position = _position(12_340)

    read = _reader(attitude, position).read()

    assert read.attitude_time_boot_s == attitude.time_boot_s
    sample = read.sample
    assert sample is not None
    assert (sample.location.lat, sample.location.lng, sample.location.alt) == (
        40.1, 44.2, 1250.0,
    )
    assert sample.location.is_absolute is True
    assert (sample.attitude.pitch, sample.attitude.yaw, sample.attitude.roll) == (
        -3.0, 120.0, 12.0,
    )
    assert sample.time_boot_s == attitude.time_boot_s
    assert sample.body_rates_rad_s == (0.01, -0.02, 0.03)
    assert sample.air_speed_mps == 22.0


@pytest.mark.parametrize(
    "position",
    [
        pytest.param(_position(12_360), id="position_one_step_newer"),
        pytest.param(_position(12_320), id="position_one_step_older"),
        pytest.param(None, id="position_missing"),
        pytest.param(
            SimpleNamespace(**{**vars(_position(0)), "time_boot_s": 12.3405}),
            id="position_non_integer_ms",
        ),
        pytest.param(
            SimpleNamespace(**{**vars(_position(0)), "time_boot_s": math.nan}),
            id="position_nan",
        ),
        pytest.param(
            SimpleNamespace(**{**vars(_position(0)), "time_boot_s": math.inf}),
            id="position_inf",
        ),
    ],
)
def test_unequal_or_malformed_position_gives_no_sample(
    position: Optional[SimpleNamespace],
) -> None:
    read = _reader(_attitude(12_340), position).read()

    assert read.sample is None
    assert read.attitude_time_boot_s == pytest.approx(12.34)


@pytest.mark.parametrize(
    "fields",
    [
        pytest.param({"attitude": Attitude(-3.0, 120.0, math.nan)}, id="roll_nan"),
        pytest.param({"attitude": Attitude(math.inf, 120.0, 12.0)}, id="pitch_inf"),
        pytest.param({"attitude": None}, id="attitude_missing"),
        pytest.param({"body_rates_rad_s": (0.01, math.nan, 0.03)}, id="rate_nan"),
        pytest.param({"body_rates_rad_s": (0.01, 0.02)}, id="rates_short"),
    ],
)
def test_malformed_attitude_values_give_no_sample(fields: dict) -> None:
    attitude = SimpleNamespace(**{**vars(_attitude(12_340)), **fields})

    read = _reader(attitude, _position(12_340)).read()

    assert read.sample is None
    assert read.attitude_time_boot_s == pytest.approx(12.34)


@pytest.mark.parametrize(
    "boot_s", [None, 12.3405, math.nan, math.inf, True, "x"],
)
def test_missing_or_malformed_attitude_boot_gives_no_sample_or_clock_time(
    boot_s: object,
) -> None:
    attitude = SimpleNamespace(**{**vars(_attitude(0)), "time_boot_s": boot_s})

    read = _reader(attitude, _position(12_340)).read()

    assert read == PollingPoseRead(None, None)
    assert _reader(None, _position(12_340)).read() == PollingPoseRead(None, None)


def test_boot_time_conversion_is_exact_and_shared_with_attitude() -> None:
    boot_ms = 4_294_967  # large stamps keep exact equality
    attitude = attitude_sample_from_message(
        SimpleNamespace(
            pitch=0.0, yaw=0.0, roll=0.0,
            rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
            time_boot_ms=boot_ms,
        ),
        1.0,
    )
    position = position_sample_from_message(
        SimpleNamespace(lat=1, lon=2, alt=3, time_boot_ms=boot_ms),
        1.0,
    )

    assert position.time_boot_s == attitude.time_boot_s
    assert whole_boot_ms(position.time_boot_s) == boot_ms
    assert whole_boot_ms(0.1 + 0.2) is None


@pytest.mark.parametrize(
    ("attitude_receipt_s", "position_receipt_s"),
    [(100.00, 100.03), (100.03, 100.00)],
)
def test_reported_receipt_is_the_earlier_receipt(
    attitude_receipt_s: float,
    position_receipt_s: float,
) -> None:
    read = _reader(
        _attitude(5_000, attitude_receipt_s),
        _position(5_000, position_receipt_s),
    ).read()

    assert read.sample is not None
    assert read.sample.receipt_time_s == 100.00


def test_returned_sample_holds_copies_of_the_views() -> None:
    attitude = _attitude(5_000)
    position = _position(5_000)

    sample = _reader(attitude, position).read().sample
    assert sample is not None
    sample.attitude.pitch = 99.0
    sample.location.lat = 0.0

    assert attitude.attitude.pitch == -3.0
    assert position.lat == 40.1
    assert sample.attitude is not attitude.attitude


@pytest.mark.parametrize(
    ("skew_s", "paired"),
    [
        pytest.param(BUDGET_S * 0.5, True, id="below"),
        pytest.param(BUDGET_S, True, id="at"),
        pytest.param(BUDGET_S * 1.5, False, id="above"),
    ],
)
def test_stalled_attitude_and_delayed_equal_boot_position(
    skew_s: float,
    paired: bool,
) -> None:
    read = _reader(
        _attitude(7_000, 200.0),
        _position(7_000, 200.0 + skew_s),
    ).read()

    assert (read.sample is not None) is paired
    if paired:
        assert read.sample.receipt_time_s == 200.0


def test_retained_pre_reboot_position_cannot_pair_with_post_reboot_attitude() -> None:
    # Same numeric boot time, but the position was received seconds before
    # the reboot: outside the association budget.
    read = _reader(_attitude(1_500, 260.0), _position(1_500, 251.0)).read()

    assert read.sample is None
    assert read.attitude_time_boot_s == 1.5


def test_polling_reader_reads_neither_location_nor_bare_attitude() -> None:
    vehicle = Mock()
    vehicle.attitude_sample = _attitude(9_000)
    vehicle.position_sample = _position(9_000)
    vehicle.air_speed = 21.0

    read = build_polling_reader(vehicle, lambda: BUDGET_S).read()

    assert read.sample is not None
    vehicle.location.assert_not_called()


def test_source_graph_polling_reader_uses_the_wall_scaled_budget() -> None:
    from navpy.sim.detection.sim_detector_config import (
        SimDetectorDependencies,
        SimDetectorOptions,
    )
    from navpy.sim.detection.sim_source_composition import build_source_graph

    source_skew_s = 2.0 / 40.0  # SCHED_LOOP_RATE 50 Hz -> 40 Hz stream
    speedup = 10.0
    vehicle = SimpleNamespace(
        target_system=1,
        get_param_or_default=Mock(return_value=50.0),
        attitude_sample=_attitude(3_000, 50.0),
        position_sample=_position(3_000, 50.0 + source_skew_s / speedup),
        simulator_truth_pose=None,
        air_speed=20.0,
        is_armed=True,
        on_message=Mock(),
    )
    cadence = Mock()
    cadence.wall_period_for_scheduler_period.side_effect = (
        lambda period_s: period_s / speedup
    )
    graph = build_source_graph(
        SimDetectorDependencies(
            vehicle=vehicle, mount=Mock(), geo_ref=Mock(), logger=Mock(),
            args=SimpleNamespace(), zc_util=None, scheduler_cadence=cadence,
        ),
        SimDetectorOptions(
            sim_assets_path=None, tracking_config=None, zoom_config=None,
            ideal_360=False, frame_generator_factory=Mock(),
        ),
    )

    assert graph.polling_reader.read().sample is not None
    vehicle.position_sample = _position(3_000, 50.0 + source_skew_s / speedup * 1.5)
    assert graph.polling_reader.read().sample is None


# --- polling loop -----------------------------------------------------------


class _StopAfterWaits(threading.Event):
    """Record each wait timeout; set the stop event during the Nth wait."""

    def __init__(self, waits_before_stop: int) -> None:
        super().__init__()
        self.timeouts: list[float] = []
        self._waits_before_stop = waits_before_stop

    def wait(self, timeout: Optional[float] = None) -> bool:
        self.timeouts.append(float(timeout or 0.0))
        if len(self.timeouts) >= self._waits_before_stop:
            self.set()
        return self.is_set()


def _execution(
    reads: list[PollingPoseRead],
    *,
    detect_pois: object,
    record_outcome: object,
    advance_source_time: object = None,
    monitor: Optional[UnpairedPoseMonitor] = None,
) -> tuple[SimDetectorExecution, _StopAfterWaits, Mock]:
    stop_event = _StopAfterWaits(len(reads))
    reader = Mock(read=Mock(side_effect=reads))
    cadence = Mock()
    cadence.wall_period_for_scheduler_period.return_value = POLL_WALL_PERIOD_S
    execution = SimDetectorExecution(
        coordinator=SimpleNamespace(stop_event=stop_event),
        polling_pose_reader=reader,
        advance_source_time=advance_source_time or Mock(),
        unpaired_pose_monitor=monitor or UnpairedPoseMonitor(Mock()),
        error=Mock(),
        cadence_lease=SchedulerCadenceLease(cadence, owned=False),
        scheduler_period_s=1.0 / 30.0,
        ideal_360=False,
        detect_pois=detect_pois,
        record_outcome=record_outcome,
    )
    return execution, stop_event, reader


def test_loop_drops_unpaired_polls_without_detecting_and_keeps_its_period() -> None:
    pair = _reader(_attitude(1_040, 10.04), _position(1_040, 10.04)).read()
    reads = [
        PollingPoseRead(1.0, None),
        PollingPoseRead(1.02, None),
        pair,
    ]
    detect = Mock(return_value=True)
    record = Mock()
    advance = Mock()
    execution, stop_event, reader = _execution(
        reads, detect_pois=detect, record_outcome=record,
        advance_source_time=advance,
    )

    execution.run()

    assert reader.read.call_count == 3  # stop during the third wait exits
    assert [c.args for c in record.call_args_list] == [
        (1.0, "unpaired_pose_dropped", None),
        (1.02, "unpaired_pose_dropped", None),
    ]
    detect.assert_called_once()
    assert detect.call_args.kwargs["attitude_time_boot_s"] == pair.sample.time_boot_s
    assert detect.call_args.kwargs["frame_receipt_timestamp_s"] == 10.04
    assert [c.args for c in advance.call_args_list] == [(1.0,), (1.02,), (1.04,)]
    assert len(stop_event.timeouts) == 3
    assert all(0.0 < t <= POLL_WALL_PERIOD_S for t in stop_event.timeouts)


def test_poll_without_attitude_boot_does_not_advance_the_clock() -> None:
    record = Mock()
    advance = Mock()
    execution, _stop, _reader_mock = _execution(
        [PollingPoseRead(None, None)], detect_pois=Mock(),
        record_outcome=record, advance_source_time=advance,
    )

    execution.run()

    advance.assert_not_called()
    record.assert_called_once_with(None, "unpaired_pose_dropped", None)


class _ClockHarness(SimpleNamespace):
    clock: PoseFrameClock
    outcomes: list[tuple]
    restart: Mock


def _clock_harness() -> _ClockHarness:
    """The real source clock, its restart action and the pipeline's accept."""
    outcomes: list[tuple] = []

    def record(boot_s: Optional[float], outcome: str, frame_s: Optional[float]) -> None:
        outcomes.append((boot_s, outcome))

    clock = PoseFrameClock(reorder_tolerance_s=BUDGET_S, record_outcome=record)
    restart = Mock(side_effect=clock.reset)

    def advance(value: Optional[float]) -> Optional[float]:
        return clock.advance(value, on_restart=restart)

    def detect(*_args: object, attitude_time_boot_s: float, **_kwargs: object) -> bool:
        clock.accept_attitude_timestamp(attitude_time_boot_s, on_restart=restart)
        return True

    return _ClockHarness(
        clock=clock, outcomes=outcomes, restart=restart,
        advance=advance, detect=detect, record=record,
    )


def _pair(boot_ms: int, receipt_s: float) -> PollingPoseRead:
    return _reader(_attitude(boot_ms, receipt_s), _position(boot_ms, receipt_s)).read()


def test_reboot_restart_fires_on_polled_attitude_before_first_post_reboot_pair() -> None:
    harness = _clock_harness()
    reads = [
        _pair(30_000, 300.0),  # pre-reboot frame
        PollingPoseRead(0.5, None),  # through the reset: never paired
        PollingPoseRead(1.5, None),
        PollingPoseRead(2.5, None),
        _pair(3_500, 304.0),  # first pair, after the 3 s fresh-boot window
    ]
    execution, _stop, _reader_mock = _execution(
        reads, detect_pois=harness.detect, record_outcome=harness.record,
        advance_source_time=harness.advance,
    )

    execution.run()

    harness.restart.assert_called_once_with()
    assert (3.5, "emitted") in harness.outcomes
    assert not any(
        outcome == "reordered_attitude" for _boot, outcome in harness.outcomes
    )
    assert harness.clock.last_frame_s == 3.5


def test_repeated_pair_across_polls_is_refused_as_dup_attitude() -> None:
    harness = _clock_harness()
    pair = _pair(8_000, 80.0)
    execution, _stop, _reader_mock = _execution(
        [pair, pair], detect_pois=harness.detect,
        record_outcome=harness.record, advance_source_time=harness.advance,
    )

    execution.run()

    assert harness.outcomes == [(8.0, "emitted"), (8.0, "dup_attitude")]


# --- unpaired warning -------------------------------------------------------


@pytest.mark.parametrize(
    ("unpaired_ms", "warnings"),
    [
        pytest.param(980, 0, id="below"),
        pytest.param(1000, 1, id="at"),
        pytest.param(1500, 1, id="above"),
    ],
)
def test_warns_once_when_unpaired_for_the_freshness_limit(
    unpaired_ms: int,
    warnings: int,
) -> None:
    assert FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S == 1.0
    warning = Mock()
    monitor = UnpairedPoseMonitor(warning)

    for boot_ms in range(10_000, 10_000 + unpaired_ms + 1, 20):
        monitor.observe(float(boot_ms) * 1e-3, paired=False)

    assert warning.call_count == warnings


def test_warns_once_on_recovery_and_rearms() -> None:
    warning = Mock()
    monitor = UnpairedPoseMonitor(warning)
    monitor.observe(10.0, paired=False)
    monitor.observe(11.0, paired=False)
    monitor.observe(11.2, paired=False)
    assert warning.call_count == 1

    monitor.observe(11.22, paired=True)
    monitor.observe(11.24, paired=True)
    assert warning.call_count == 2
    assert "resumed" in warning.call_args.args[0]

    monitor.observe(12.0, paired=False)
    monitor.observe(12.5, paired=False)
    assert warning.call_count == 2
    monitor.observe(13.0, paired=False)
    assert warning.call_count == 3


def test_no_recovery_warning_without_an_unpaired_warning() -> None:
    warning = Mock()
    monitor = UnpairedPoseMonitor(warning)
    monitor.observe(10.0, paired=False)
    monitor.observe(10.5, paired=True)

    warning.assert_not_called()


def test_unpaired_window_restarts_with_the_source_clock() -> None:
    warning = Mock()
    monitor = UnpairedPoseMonitor(warning)
    monitor.observe(30.0, paired=False)
    monitor.observe(0.5, paired=False)  # reboot
    monitor.observe(1.4, paired=False)

    warning.assert_not_called()
    monitor.observe(1.5, paired=False)
    warning.assert_called_once()


def test_execution_composition_wires_pairing_collaborators() -> None:
    from navpy.sim.detection.sim_execution_composition import build_execution_graph

    deps = SimpleNamespace(
        vehicle=Mock(), mount=Mock(), logger=Mock(), scheduler_cadence=Mock(),
    )
    source = SimpleNamespace(
        coordinator=Mock(), pose_source=Mock(), activation=Mock(),
        record_outcome=Mock(), polling_reader=Mock(),
    )
    render = SimpleNamespace(
        poi_provider=Mock(), capture=Mock(), gap=Mock(), tracking=Mock(),
        ideal_camera=Mock(), evidence_recorder=None,
    )
    with patch(
        "navpy.sim.detection.sim_execution_composition.SimDetectorWorker"
    ) as worker_type:
        build_execution_graph(
            deps, SimpleNamespace(ideal_360=False), source, render, Mock(),
        )

    kwargs = worker_type.call_args.kwargs
    assert kwargs["polling_pose_reader"] is source.polling_reader
    assert kwargs["advance_source_time"] is source.pose_source.advance_source_now
    assert isinstance(kwargs["unpaired_pose_monitor"], UnpairedPoseMonitor)
