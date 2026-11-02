"""HITL stage 1 step 10: the capture clock (``rt.perf_counter``).

Frames, ATTITUDE and gimbal receipts, their histories and capture deadlines
share one high-resolution clock that never steps backward. A rebound test
runtime makes the wall clock jump by hours and the monotonic clock tick
coarsely on another epoch: any capture-path subtraction that mixed them would
be off by thousands of seconds and fail closed. The wall receipt stays for
telemetry. No raw epoch is assumed equal to another host's.
"""

from __future__ import annotations

import math
import itertools
import re
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_AUTOPILOT1

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.board_clock_map import AttitudeBoardClock
from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
from navpy.core.perception.camera.capture_interpolation import (
    STATUS_NO_CAPTURE_RECEIPT,
    interpolate_attitude_at,
)
from navpy.core.perception.camera.capture_lookup import CaptureLookupCalibration
from navpy.core.perception.camera.frame_publication import FramePublicationStore
from navpy.core.perception.pipeline.real_detector_state import FreshnessPolicy
from navpy.core.perception.pipeline.real_frame_association import FrameAssociationBuilder
from navpy.core.ports.frame_capture_ports import CaptureStamp, CaptureStampKind
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.plugins.camera.opencv.frame_capture_opencv import OpenCvFrameCapture
from navpy.plugins.gimbal.siyi.siyi_sdk import SIYISDK
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

REPO = Path(__file__).resolve().parents[3]
PERF_EPOCH_S = 50_000.0          # this host's perf_counter, not the board's
WALL_EPOCH_S = 1_800_000_000.0   # unix seconds
MONOTONIC_EPOCH_S = 9_000.0      # yet another epoch
WINDOWS_MONOTONIC_TICK_S = 0.015625
ATTITUDE_PERIOD_S = 0.020
BOOT_START_S = 300.0
ROLL_RATE_DEG_S = 60.0
LINK = "udp:capture-clock#1"
SOURCE = "npyc://capture-clock"
# Done criterion of step 10: reported and observed tick spacing.
TICK_SPACING_LIMIT_S = 1e-4
_DEVICES = itertools.count(1)


class ScriptedClock:
    """perf_counter advances only when told; the wall clock jumps on request."""

    def __init__(self) -> None:
        self.elapsed_s = 0.0
        self.wall_jump_s = 0.0

    def advance(self, seconds: float) -> None:
        self.elapsed_s += seconds

    def perf_counter(self) -> float:
        return PERF_EPOCH_S + self.elapsed_s

    def monotonic(self) -> float:
        ticks = math.floor(self.elapsed_s / WINDOWS_MONOTONIC_TICK_S)
        return MONOTONIC_EPOCH_S + ticks * WINDOWS_MONOTONIC_TICK_S

    def time(self) -> float:
        return WALL_EPOCH_S + self.elapsed_s + self.wall_jump_s

    def thread_time(self) -> float:
        return time.thread_time()

    def now(self, tz: Any = None) -> datetime:
        return datetime.fromtimestamp(self.time(), tz)

    def sleep(self, seconds: float) -> None:
        time.sleep(min(seconds, 0.001))


@pytest.fixture
def clock(monkeypatch: pytest.MonkeyPatch) -> ScriptedClock:
    scripted = ScriptedClock()
    wall = rt.wall_runtime()
    binding = rt.RuntimeBinding()
    binding.bind(rt.RuntimeTime(scripted, wall.threads, wall.entropy))
    monkeypatch.setattr(rt, "_binding", binding)
    return scripted


def _vehicle() -> VehicleMav:
    connection = MagicMock()
    connection.recv_match = MagicMock(return_value=None)
    with patch(
        "navpy.plugins.vehicle.ardupilot.mav_bus.mavutil.mavlink_connection",
        return_value=connection,
    ):
        return _open_vehicle()


def _open_vehicle() -> VehicleMav:
    return VehicleMav(
        device=f"dev:{next(_DEVICES)}",
        target_system=1,
        logger=MagicMock(),
        skip_mission_download=True,
        wait_heartbeat=False,
        heartbeat_timeout=999.0,
    )


def _attitude(boot_s: float) -> MagicMock:
    message = MagicMock()
    message.get_type.return_value = "ATTITUDE"
    message.get_srcSystem.return_value = 1
    message.get_srcComponent.return_value = MAV_COMP_ID_AUTOPILOT1
    message.get_seq.return_value = 0
    message.get_msgId.return_value = 30
    message.time_boot_ms = int(round(boot_s * 1000))
    message.roll = math.radians(_roll_at(boot_s))
    message.pitch = 0.0
    message.yaw = 0.0
    message.rollspeed = math.radians(ROLL_RATE_DEG_S)
    message.pitchspeed = 0.0
    message.yawspeed = 0.0
    return message


def _roll_at(boot_s: float) -> float:
    return ROLL_RATE_DEG_S * (boot_s - BOOT_START_S)


def _feed(vehicle: VehicleMav, clock: ScriptedClock, count: int,
          jumps: dict[int, float]) -> list[tuple[float, float]]:
    """ATTITUDE at 50 Hz with a fixed link delay; the wall clock jumps at given indexes."""
    receipts = []
    for index in range(count):
        if index in jumps:
            clock.wall_jump_s += jumps[index]
        boot_s = BOOT_START_S + index * ATTITUDE_PERIOD_S
        vehicle.feed_message(_attitude(boot_s))
        receipts.append((clock.perf_counter(), clock.time()))
        clock.advance(ATTITUDE_PERIOD_S)
    return receipts


def _calibration() -> CaptureLookupCalibration:
    return CaptureLookupCalibration(
        source_id=SOURCE,
        link_id=LINK,
        capture_lookup_bias_s=0.0,
        attitude_link_delay_s=0.0,
        residual_bound_s=0.002,
        camera_variation_bound_s=0.002,
        attitude_variation_bound_s=0.002,
        min_frame_interval_s=0.033,
    )


def _fixed_mount(width: int, height: int) -> CameraMountFrameState:
    return CameraMountFrameState(
        gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
        k=np.array([[1000.0, 0.0, width / 2], [0.0, 1000.0, height / 2], [0.0, 0.0, 1.0]]),
        dist=np.zeros(5),
        gimbal_timestamp_s=None,
        gimbal_is_static=True,
        zoom_command=None,
        zoom_sample_id="static:fixed",
        zoom_sample_age_s=0.0,
    )


def test_attitude_receipt_keeps_the_wall_stamp_and_adds_a_capture_stamp(clock) -> None:
    vehicle = _vehicle()
    try:
        receipts = _feed(vehicle, clock, 30, {10: -3600.0, 20: +7200.0})
        history = vehicle.attitude_history(30)
    finally:
        vehicle.close()

    assert [sample.capture_receipt_s for sample in history] == [perf for perf, _ in receipts]
    assert [sample.receipt_time_s for sample in history] == [wall for _, wall in receipts]
    captures = [sample.capture_receipt_s for sample in history]
    assert all(later > earlier for earlier, later in zip(captures, captures[1:]))
    walls = [sample.receipt_time_s for sample in history]
    assert walls[10] < walls[9] - 3000.0  # the wall clock did step back
    assert vehicle.attitude_sample.capture_receipt_s == captures[-1]


def test_exposure_frame_associates_across_wall_jumps(clock) -> None:
    """Board time -> capture clock -> publication -> association, all on perf_counter."""
    vehicle = _vehicle()
    try:
        _feed(vehicle, clock, 40, {12: -3600.0, 25: +5400.0, 33: -86400.0})
        exposure_boot_s = BOOT_START_S + 30.5 * ATTITUDE_PERIOD_S
        exposure_s = AttitudeBoardClock(vehicle.attitude_history).capture_time_s(
            int(round(exposure_boot_s * 1e6)),
        )
        assert exposure_s == pytest.approx(PERF_EPOCH_S + 30.5 * ATTITUDE_PERIOD_S, abs=1e-9)

        store = FramePublicationStore()
        store.set_running(True)
        clock.wall_jump_s -= 3600.0           # between capture and association too
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        store.publish(frame, CaptureStamp(exposure_s, CaptureStampKind.EXPOSURE))
        snapshot = store.snapshot()
        builder = FrameAssociationBuilder(
            attitude_sample_reader=lambda: vehicle.attitude_sample,
            mount_state_reader=_fixed_mount,
            location_reader=lambda: None,
            air_speed_reader=lambda: 22.0,
            attitude_history_reader=vehicle.attitude_history,
            link_identity_reader=lambda: LINK,
            calibration=_calibration(),
            source_id=SOURCE,
        )
        clock.advance(0.004)
        association = builder.capture(snapshot, frame)
    finally:
        vehicle.close()

    assert association.pose_status == "real_frame_pose"
    assert association.pose_is_frame_atomic
    assert association.uas_att.roll == pytest.approx(_roll_at(exposure_boot_s), abs=1e-6)
    assert association.associated_at_s == clock.perf_counter()
    assert association.frame_timestamp_s == snapshot.published_at_s
    # Newest ATTITUDE at 39 periods, association 1 period + 4 ms later: no wall term.
    assert association.pose_age_s == pytest.approx((40 - 30.5) * ATTITUDE_PERIOD_S + 0.004)
    assert association.attitude_receipt_time_s == pytest.approx(exposure_s)
    assert store.has_fresh_frame(0.1)


def test_a_sample_without_a_capture_receipt_is_refused_not_read_on_the_wall_clock() -> None:
    history = tuple(
        SimpleNamespace(
            attitude=Attitude(0.0, 0.0, 0.0), body_rates_rad_s=None,
            time_boot_s=1.0 + index * 0.02, receipt_time_s=100.0 + index * 0.02,
            capture_receipt_s=None if index == 1 else 100.0 + index * 0.02,
        )
        for index in range(3)
    )
    result, status = interpolate_attitude_at(history, 100.03, 0.0)
    assert result is None
    assert status == STATUS_NO_CAPTURE_RECEIPT
    no_capture = tuple(
        SimpleNamespace(time_boot_s=1.0 + index * 0.02, receipt_time_s=100.0 + index * 0.02)
        for index in range(12)
    )
    assert AttitudeBoardClock(lambda count: no_capture).capture_time_s(1_100_000) is None


def test_read_frames_and_track_freshness_use_the_capture_clock(clock) -> None:
    class _Stub:
        def __init__(self) -> None:
            self.grabs = 0

        def isOpened(self) -> bool:
            return True

        def grab(self) -> bool:
            self.grabs += 1
            clock.advance(0.033)
            clock.wall_jump_s -= 3600.0
            return self.grabs <= 2

        def retrieve(self) -> tuple[bool, np.ndarray]:
            return True, np.zeros((4, 4, 3), dtype=np.uint8)

        def release(self) -> None:
            pass

    published: list[CaptureStamp] = []
    calls = {"count": 0}

    def is_running() -> bool:
        calls["count"] += 1
        return calls["count"] <= 6

    OpenCvFrameCapture(_Stub()).run(lambda _frame, stamp: published.append(stamp), is_running)
    assert [stamp.captured_at_s for stamp in published] == pytest.approx(
        [PERF_EPOCH_S + 0.033, PERF_EPOCH_S + 0.066],
    )
    assert all(stamp.kind is CaptureStampKind.READ for stamp in published)

    track = SimpleNamespace(timestamp=clock.perf_counter())
    clock.wall_jump_s += 7200.0
    clock.advance(0.05)
    assert FreshnessPolicy(0.05).is_fresh(track)
    clock.advance(1.0)
    assert not FreshnessPolicy(0.05).is_fresh(track)


def test_siyi_receipts_are_stamped_on_the_capture_clock() -> None:
    sdk = SIYISDK()
    try:
        before = time.perf_counter()
        assert sdk.parseAttitudeMsg("640038ff3200000000000000", 9)
        assert sdk.parseCurrentZoomLevelMsg("0305", 4)
        after = time.perf_counter()
        _seq, stamp, *_angles = sdk.getAttitudeSample()
        _zoom_seq, _level, zoom_receipt = sdk.getCurrentZoomLevelSample()
    finally:
        sdk.disconnect()
    assert before <= stamp <= zoom_receipt <= after


CAPTURE_PATH = (
    "src/navpy/core/perception/camera/board_clock_map.py",
    "src/navpy/core/perception/camera/camera_mount_frame_state.py",
    "src/navpy/core/perception/camera/capture_interpolation.py",
    "src/navpy/core/perception/camera/capture_lookup.py",
    "src/navpy/core/perception/camera/frame_publication.py",
    "src/navpy/core/perception/pipeline/real_detected_object_builder.py",
    "src/navpy/core/perception/pipeline/real_detector_controls.py",
    "src/navpy/core/perception/pipeline/real_detector_diagnostics.py",
    "src/navpy/core/perception/pipeline/real_detector_state.py",
    "src/navpy/core/perception/pipeline/real_frame_association.py",
    "src/navpy/core/perception/pipeline/real_frame_association_gate.py",
    "src/navpy/plugins/camera/ffmpeg/frame_capture_ffmpeg.py",
    "src/navpy/plugins/camera/gstreamer/frame_capture_gstreamer.py",
    "src/navpy/plugins/camera/opencv/frame_capture_opencv.py",
    "src/navpy/plugins/camera/unreal/frame_capture_npyc.py",
    "src/navpy/plugins/gimbal/siyi/hardware/polling.py",
    "src/navpy/plugins/gimbal/siyi/hardware/readback.py",
    "src/navpy/plugins/gimbal/siyi/hardware/state.py",
    "src/navpy/plugins/tracking/boxmot/tracker_backends_boxmot.py",
    "src/navpy/plugins/tracking/kalman/multi_object_tracker.py",
)
OTHER_CLOCK = re.compile(r"\brt\.(time|monotonic)\b|\btime\.(time|monotonic)\b")


@pytest.mark.parametrize("relative", CAPTURE_PATH)
def test_capture_path_reads_no_other_clock(relative: str) -> None:
    source = (REPO / relative).read_text(encoding="utf-8")
    assert OTHER_CLOCK.findall(source) == []


def test_capture_clock_ticks_finely_under_load() -> None:
    """Done check: reported and observed tick spacing <= 0.1 ms with busy threads."""
    reported_s = time.get_clock_info("perf_counter").resolution
    stop = threading.Event()

    def burn() -> None:
        while not stop.is_set():
            sum(range(1000))

    burners = [threading.Thread(target=burn, daemon=True) for _ in range(4)]
    for burner in burners:
        burner.start()
    try:
        spacings = []
        for _ in range(200):
            first = rt.perf_counter()
            second = rt.perf_counter()
            while second == first:
                second = rt.perf_counter()
            spacings.append(second - first)
    finally:
        stop.set()
        for burner in burners:
            burner.join()
    assert reported_s <= TICK_SPACING_LIMIT_S
    assert statistics.median(spacings) <= TICK_SPACING_LIMIT_S
