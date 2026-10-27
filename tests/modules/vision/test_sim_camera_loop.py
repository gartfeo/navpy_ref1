"""The sim camera's latency and drops on a tick clock (``deterministic_dev_sim.md`` step 5)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.sim.detection import sim_camera_loop
from navpy.sim.detection.scheduler_cadence_lease import SchedulerCadenceLease
from navpy.sim.detection.sim_camera_loop import SimCamera, SimCameraLoop
from navpy.sim.detection.sim_detector_execution import SimDetectorExecution
from navpy.sim.detection.sim_polling_pose_reader import PollingPoseRead
from navpy.sim.detection.unpaired_pose_monitor import UnpairedPoseMonitor
from navpy.sim.lockstep.camera_frames import TICK_S, FrameDraw

#: The receipt clock (``rt.time``) is another domain than the tick clock (``rt.monotonic``).
TIME_OFFSET_S = 1000.0


class TickClock:
    """Lockstep time: a wait wakes at the first tick at or after its deadline."""

    def __init__(self, stop_tick: int) -> None:
        self.tick = 0
        self._stop_tick = stop_tick

    def monotonic(self) -> float:
        return self.tick * TICK_S

    def time(self) -> float:
        return TIME_OFFSET_S + self.tick * TICK_S

    def is_set(self) -> bool:
        return self.tick >= self._stop_tick

    def wait(self, timeout: float) -> bool:
        assert timeout > 0, "the loop must never spin without a deadline ahead"
        self.tick = max(self.tick + 1, math.ceil((self.monotonic() + timeout) / TICK_S - 1e-9))
        return self.is_set()


class Schedule:
    def __init__(self, draws: list[FrameDraw]) -> None:
        self._draws = iter(draws)

    def next_frame(self) -> FrameDraw:
        return next(self._draws)


@dataclass
class Frame:
    clock: TickClock
    timestamp_s: float
    accept: bool
    published: list = field(default_factory=list)

    def publish(self, receipt_timestamp_s: float) -> bool:
        self.published.append((self.clock.tick, receipt_timestamp_s))
        return self.accept


@dataclass
class Rig:
    clock: TickClock
    draws: list[FrameDraw]
    accept: bool = True
    paired: bool = True
    frames: list[Frame] = field(default_factory=list)
    captures: list = field(default_factory=list)
    outcomes: list = field(default_factory=list)

    def read_pose(self) -> PollingPoseRead:
        boot_s = 100.0 + self.clock.tick * TICK_S
        sample = SimpleNamespace(
            location="loc", attitude="att", time_boot_s=boot_s, body_rates_rad_s=(0.0, 0.0, 0.0),
            receipt_time_s=self.clock.time(), air_speed_mps=18.0,
        ) if self.paired else None
        return PollingPoseRead(boot_s, sample)

    def capture(self, location, attitude, attitude_time_boot_s=None, uas_body_rates_rad_s=None, *,
                frame_receipt_timestamp_s=None, frame_air_speed_mps=None):
        self.captures.append((self.clock.tick, location, attitude, attitude_time_boot_s, frame_air_speed_mps))
        frame = Frame(self.clock, attitude_time_boot_s, self.accept)
        self.frames.append(frame)
        return frame

    def run(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(sim_camera_loop, "rt", self.clock)
        camera = SimCamera(Schedule(self.draws), self.capture)
        SimCameraLoop(camera, self.clock, self.read_pose, lambda *args: self.outcomes.append(args)).run()


def test_a_frame_is_published_its_latency_in_ticks_after_capture_with_the_capture_time_kept(monkeypatch):
    rig = Rig(TickClock(stop_tick=12), [FrameDraw(False, 5, 3)] * 4)
    rig.run(monkeypatch)
    assert [tick for tick, *_ in rig.captures] == [0, 3, 6, 9]
    assert [f.timestamp_s for f in rig.frames] == pytest.approx([100.0, 100.06, 100.12, 100.18])
    published = [f.published for f in rig.frames]
    # Each published 5 ticks later; its receipt is the publish tick on the receipt clock.
    assert [p[0][0] for p in published[:3]] == [5, 8, 11]
    assert [p[0][1] for p in published[:3]] == pytest.approx([TIME_OFFSET_S + k * TICK_S for k in (5, 8, 11)])
    assert published[3] == [] and rig.outcomes == [(pytest.approx(100.18), "camera_stopped_dropped", None)]


def test_frames_publish_in_capture_order_when_a_later_one_is_due_earlier(monkeypatch):
    rig = Rig(TickClock(stop_tick=10), [FrameDraw(False, 8, 1), FrameDraw(False, 4, 20)])
    rig.run(monkeypatch)
    assert [tick for tick, *_ in rig.captures] == [0, 1]
    assert [f.published[0][0] for f in rig.frames] == [8, 8]  # the second waits for the first


def test_a_dropped_slot_is_never_rendered_and_the_next_slot_still_follows_its_gap(monkeypatch):
    rig = Rig(TickClock(stop_tick=10), [FrameDraw(True, 4, 2), FrameDraw(False, 4, 20)])
    rig.run(monkeypatch)
    assert [tick for tick, *_ in rig.captures] == [2]
    assert rig.outcomes == [(pytest.approx(100.0), "camera_dropped", None)]
    assert rig.frames[0].published[0][0] == 6


def test_a_frame_a_reset_made_stale_is_recorded_as_rejected(monkeypatch):
    rig = Rig(TickClock(stop_tick=6), [FrameDraw(False, 4, 20)], accept=False)
    rig.run(monkeypatch)
    assert rig.frames[0].published[0][0] == 4
    assert rig.outcomes == [(pytest.approx(100.0), "camera_rejected_dropped", None)]


def test_an_unpaired_pose_renders_nothing(monkeypatch):
    rig = Rig(TickClock(stop_tick=6), [FrameDraw(False, 4, 2)] * 3, paired=False)
    rig.run(monkeypatch)
    assert rig.captures == [] and rig.outcomes == []


def test_a_capture_reads_this_ticks_pose_fields(monkeypatch):
    rig = Rig(TickClock(stop_tick=2), [FrameDraw(False, 1, 20)])
    rig.run(monkeypatch)
    assert rig.captures == [(0, "loc", "att", 100.0, 18.0)]


def _execution(*, ideal_360: bool, reads: list[PollingPoseRead], capture, record, advance):
    stop = TickClock(stop_tick=1)
    return SimDetectorExecution(
        coordinator=SimpleNamespace(stop_event=stop),
        polling_pose_reader=Mock(read=Mock(side_effect=reads)),
        advance_source_time=advance,
        unpaired_pose_monitor=UnpairedPoseMonitor(Mock()),
        error=Mock(),
        cadence_lease=SchedulerCadenceLease(Mock(), owned=False),
        scheduler_period_s=1.0 / 30.0,
        ideal_360=ideal_360,
        detect_pois=Mock(side_effect=AssertionError("the polling loop must not run")),
        record_outcome=record,
        camera=SimCamera(Schedule([FrameDraw(False, 4, 20)]), capture),
    ), stop


def test_the_execution_runs_the_camera_loop_through_the_polled_pose_read(monkeypatch):
    outcomes, advance, capture = [], Mock(), Mock()
    execution, stop = _execution(ideal_360=False, reads=[PollingPoseRead(1.0, None)], capture=capture,
                                 record=lambda *args: outcomes.append(args), advance=advance)
    monkeypatch.setattr(sim_camera_loop, "rt", stop)
    execution.run()
    # The unpaired read still advances the source clock and is recorded, as in the polling loop.
    advance.assert_called_once_with(1.0)
    assert outcomes == [(1.0, "unpaired_pose_dropped", None)]
    capture.assert_not_called()


def test_the_camera_model_refuses_ideal_360():
    with pytest.raises(ValueError, match="not ideal_360"):
        _execution(ideal_360=True, reads=[], capture=Mock(), record=Mock(), advance=Mock())
