"""HITL step 21: the search sweep (core/command/gimbal/gimbal_search.py, gimbal_search_pattern.py, the search tick in
gimbal_visual_tracking.py and the tracking sink), docs/design/hitl_guidance.md section 9 item 21.

The scan is profile-defined and dock-independent, runs only while nothing is detected, and writes through the
mount's gimbal actuator (the fenced SIYI writer). Unit tests drive the scan with a fake actuator, readback and clock;
the seeded replays close the loop with navpy's simulated SIYI plant and a truth-side detector, which alone knows the
dock: navpy's side gets pixels and gimbal readback only.
"""
from __future__ import annotations

import ast
import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import pytest

from navpy.core.command.gimbal.gimbal_navigation import GimbalNavigation
from navpy.core.command.gimbal.gimbal_navigation_state import GimbalTrackingSetup
from navpy.core.command.gimbal.gimbal_search import GimbalSearch, GimbalSearchSetup
from navpy.core.command.gimbal.gimbal_search_pattern import GimbalSearchPattern
from navpy.core.command.gimbal.gimbal_tracking_constants import MODE_FOLLOW, MODE_LOCK
from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.pipeline.real_detector_state import FreshnessPolicy
from navpy.core.perception.pipeline.real_gimbal_adapters import GimbalMeasurementAdapter
from navpy.core.perception.pipeline.real_tracking import TrackingNavigationSink
from navpy.core.perception.pointing.gimbal_rate_tracker import GimbalRateTrackerConfig
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory, GimbalHistorySample
from navpy.plugins.gimbal.siyi.hardware.command_fence import SiyiCommandFence
from navpy.plugins.gimbal.siyi.hardware.commands import SiyiHardwareCommands
from navpy.plugins.gimbal.siyi.hardware.session import SiyiSdkSession
from navpy.plugins.gimbal.siyi.hardware.state import SiyiReadbackStore
from navpy.plugins.gimbal.siyi.siyi_message import SIYIMESSAGE
from navpy.sim.gimbal.siyi.gimbal_angles import GimbalAngularPlant
from tests.detection_factory import make_detected_poi
from tools.hitl.scenario_service import draw_dock

REPO = Path(__file__).resolve().parents[3]
CLOCK = "navpy.core.kernel.runtime_time.perf_counter"
# The hitl profile's values (vision_profiles.json siyi_zr10_hitl_encoded); the unit tests use a two-row pattern.
PROFILE = {"yaw_deg": [-45.0, 45.0], "pitch_rows_deg": [-17.0], "rate_dps": 30.0, "tolerance_deg": 1.0,
           "approach_s": 0.5, "readback_max_age_s": 0.5, "frame_max_age_s": 1.0, "clear_s": 2.0}
UNIT = {**PROFILE, "yaw_deg": [-60.0, 60.0], "pitch_rows_deg": [-20.0, -40.0], "rate_dps": 20.0}
SLEW_DPS = 90.0


class Clock:
    def __init__(self, now: float = 100.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class Actuator:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.fail_rate = False

    def set_rate(self, yaw: float, pitch: float) -> None:
        if self.fail_rate:
            raise OSError("socket closed")
        self.calls.append(("rate", yaw, pitch))

    def set_motion_mode(self, mode: int) -> None:
        self.calls.append(("mode", mode))

    def set_att(self, attitude: Attitude) -> None:
        self.calls.append(("att", attitude.yaw, attitude.pitch))

    def rates(self) -> list[tuple[float, float]]:
        return [(c[1], c[2]) for c in self.calls if c[0] == "rate"]


class Readback:
    """The newest attitude readback; ``att=None`` gives no samples."""

    def __init__(self, clock: Clock, yaw: float = 0.0, pitch: float = -14.0) -> None:
        self.clock, self.yaw, self.pitch, self.age, self.att_present = clock, yaw, pitch, 0.02, True

    def __call__(self) -> GimbalCaptureHistory | None:
        att = Attitude(self.pitch, self.yaw, 0.0)
        samples = (GimbalHistorySample(1, 1, self.clock.now - self.age, att),) if self.att_present else ()
        return GimbalCaptureHistory(1, samples, GimbalData(att))


def pattern(**overrides) -> GimbalSearchPattern:
    return GimbalSearchPattern.from_profile({**UNIT, **overrides})


def make_search(revoked=lambda: False, **overrides):
    clock, actuator = Clock(), Actuator()
    readback, zoom = Readback(clock), Mock(return_value=True)
    search = GimbalSearch(GimbalSearchSetup(pattern(**overrides), revoked), actuator, readback, zoom,
                          SLEW_DPS, Mock())
    return search, clock, actuator, readback, zoom


def frames(search: GimbalSearch, clock: Clock, count: int, seen: bool = False, dt: float = 1 / 30,
           blocked: str | None = None) -> None:
    for _ in range(count):
        clock.now += dt
        search.tick(blocked, seen, True)


# ---- the pattern -------------------------------------------------------------------------------------------------
def test_two_rows_sweep_each_row_end_to_end_and_join_the_next_cycle():
    assert pattern().targets == ((-60.0, -20.0), (60.0, -20.0), (60.0, -40.0), (-60.0, -40.0))


def test_three_rows_go_down_and_back_up():
    rows = [t[1] for t in pattern(pitch_rows_deg=[-10, -30, -50]).targets]
    assert rows == [-10, -10, -30, -30, -50, -50, -30, -30]


def test_one_row_sweeps_there_and_back():
    assert pattern(pitch_rows_deg=[-25]).targets == ((-60.0, -25.0), (60.0, -25.0), (60.0, -25.0), (-60.0, -25.0))


@pytest.mark.parametrize("overrides, match", [
    ({"yaw_deg": [60, -60]}, "yaw_deg"), ({"yaw_deg": [-150, 60]}, "yaw_deg"), ({"yaw_deg": [0]}, "yaw_deg"),
    ({"pitch_rows_deg": []}, "empty"), ({"pitch_rows_deg": [-20, -20]}, "repeats"),
    ({"pitch_rows_deg": [-95]}, "pitch_rows_deg"), ({"rate_dps": 0}, "rate_dps"), ({"tolerance_deg": 0}, "tolerance"),
    ({"approach_s": -1}, "approach_s"), ({"readback_max_age_s": 0}, "readback"), ({"frame_max_age_s": 0}, "frame"),
    ({"clear_s": -1}, "clear_s"), ({"rate_dps": float("nan")}, "finite"), ({"rate_dps": True}, "not a number"),
    ({"pitch_rows_deg": "-20"}, "not a list"),
])
def test_a_wrong_profile_is_refused(overrides, match):
    with pytest.raises(ValueError, match=match):
        pattern(**overrides)


def test_a_profile_missing_a_value_is_refused():
    spec = dict(UNIT)
    del spec["clear_s"]
    with pytest.raises(ValueError, match="clear_s"):
        GimbalSearchPattern.from_profile(spec)


def test_the_hitl_profile_defines_the_search_and_the_flight_profile_does_not():
    from navpy.app.perception.gimbal_search_profile import build_gimbal_search
    from navpy.core.perception.profile.vision_profile_loader import resolve_profile
    _, hitl, _ = resolve_profile("siyi_zr10_hitl_encoded", Mock())
    _, flight, _ = resolve_profile("siyi_zr10", Mock())
    setup = build_gimbal_search(hitl["devices"][0], lambda: False)
    assert setup.pattern == GimbalSearchPattern.from_profile(PROFILE)
    assert build_gimbal_search(flight["devices"][0], lambda: False) is None
    with pytest.raises(ValueError, match="command fence"):
        build_gimbal_search(hitl["devices"][0], None)


# ---- the scan ----------------------------------------------------------------------------------------------------
def test_the_scan_starts_in_follow_at_the_widest_zoom_and_steers_to_its_first_target():
    search, clock, actuator, readback, zoom = make_search()
    with patch(CLOCK, clock):
        frames(search, clock, 1)
    assert actuator.calls[0] == ("mode", MODE_FOLLOW) and zoom.call_count == 1
    yaw, pitch = actuator.rates()[-1]
    assert yaw < 0 and pitch < 0                       # toward (-60, -20) from (0, -14)
    assert math.hypot(yaw, pitch) == pytest.approx(20.0 / SLEW_DPS * 100.0)
    assert search.scanning and search.reason == "scanning"


def test_the_scan_advances_on_arrival_and_slows_near_a_target():
    search, clock, actuator, readback, _ = make_search()
    with patch(CLOCK, clock):
        readback.yaw, readback.pitch = -59.5, -20.2     # within 1 deg of (-60, -20): next target (60, -20)
        frames(search, clock, 1)
        yaw, pitch = actuator.rates()[-1]
        assert yaw == pytest.approx(20.0 / SLEW_DPS * 100.0, abs=0.5) and abs(pitch) < 1.5
        readback.yaw, readback.pitch = 57.0, -20.0      # 3 deg short: 6 deg/s, not the full rate
        frames(search, clock, 1)
        assert actuator.rates()[-1][0] == pytest.approx(6.0 / SLEW_DPS * 100.0)


def test_a_small_error_outside_the_tolerance_still_moves_by_one_unit():
    search, clock, actuator, readback, _ = make_search(approach_s=100.0)
    with patch(CLOCK, clock):
        readback.yaw, readback.pitch = -58.5, -20.0
        frames(search, clock, 1)
    assert actuator.rates()[-1] == (-1.0, 0.0)


def test_own_detection_stops_the_scan_at_once_and_it_resumes_only_after_clear_s():
    search, clock, actuator, _, _ = make_search()
    with patch(CLOCK, clock):
        frames(search, clock, 3)
        moving = len(actuator.rates())
        frames(search, clock, 1, seen=True)
        assert actuator.rates()[moving:] == [(0.0, 0.0)] and not search.scanning and search.reason == "detected"
        frames(search, clock, 59)                       # 1.97 s after the detection: still held, no writes
        assert actuator.rates()[moving:] == [(0.0, 0.0)]
        frames(search, clock, 2)
        assert search.scanning and actuator.rates()[-1] != (0.0, 0.0)


def test_a_coast_tick_steers_nothing_and_frames_older_than_their_limit_stop_the_scan():
    search, clock, actuator, _, _ = make_search()
    with patch(CLOCK, clock):
        frames(search, clock, 1)
        writes = len(actuator.calls)
        for _ in range(60):                             # 60 Hz coast ticks with no new frame: 1.0 s
            clock.now += 1 / 60
            search.tick(None, False, False)
        assert len(actuator.calls) == writes
        clock.now += 1 / 60
        search.tick(None, False, False)
    assert actuator.rates()[-1] == (0.0, 0.0) and search.reason == "frames_stale"


@pytest.mark.parametrize("change", ["stale", "missing", "future", "nan"])
def test_stale_or_missing_readback_stops_the_scan(change):
    search, clock, actuator, readback, _ = make_search()
    with patch(CLOCK, clock):
        frames(search, clock, 1)
        if change == "stale":
            readback.age = 0.51
        elif change == "missing":
            readback.att_present = False
        elif change == "future":
            readback.age = -0.01
        else:
            readback.yaw = float("nan")
        frames(search, clock, 1)
    assert actuator.rates()[-1] == (0.0, 0.0) and search.reason == "readback_stale"


def test_no_readback_never_starts_the_scan():
    search, clock, actuator, readback, zoom = make_search()
    readback.att_present = False
    with patch(CLOCK, clock):
        frames(search, clock, 10)
    assert actuator.calls == [] and zoom.call_count == 0


@pytest.mark.parametrize("owner", ["tracking", "geo", "guidance"])
def test_a_session_that_owns_the_gimbal_blocks_the_scan(owner):
    search, clock, actuator, _, _ = make_search()
    with patch(CLOCK, clock):
        frames(search, clock, 2)
        frames(search, clock, 5, blocked=owner)
    assert actuator.rates()[-1] == (0.0, 0.0) and actuator.rates().count((0.0, 0.0)) == 1
    assert search.reason == owner


def test_revoke_latches_and_writes_nothing_more():
    revoked = [False]
    search, clock, actuator, _, _ = make_search(revoked=lambda: revoked[0])
    with patch(CLOCK, clock):
        frames(search, clock, 2)
        calls = len(actuator.calls)
        revoked[0] = True
        frames(search, clock, 5)
        search.yield_to("detection_session")
        revoked[0] = False                              # a revoke never lifts
        frames(search, clock, 90)
    assert len(actuator.calls) == calls and search.reason == "revoked" and not search.scanning


def test_a_failed_rate_write_is_stopped_with_a_zero_next():
    search, clock, actuator, _, _ = make_search()
    with patch(CLOCK, clock):
        actuator.fail_rate = True
        frames(search, clock, 1)
        actuator.fail_rate = False
        frames(search, clock, 1, seen=True)
    assert actuator.rates() == [(0.0, 0.0)]


def test_a_failed_widest_zoom_holds_the_scan_and_is_retried_each_start_interval():
    search, clock, actuator, _, zoom = make_search()
    zoom.return_value = False
    with patch(CLOCK, clock):
        frames(search, clock, 29)                       # 0.93 s: one try only
        assert zoom.call_count == 1 and actuator.calls == [] and search.reason == "zoom_failed"
        zoom.return_value = True
        frames(search, clock, 3)
    assert zoom.call_count == 2 and search.scanning and actuator.calls[0] == ("mode", MODE_FOLLOW)


def test_a_failed_follow_command_starts_no_scan():
    search, clock, actuator, _, _ = make_search()
    actuator.set_motion_mode = Mock(side_effect=OSError("no gimbal"))
    with patch(CLOCK, clock):
        frames(search, clock, 3)
    assert actuator.calls == [] and not search.scanning


# ---- composition: sessions, the sink, the fence ------------------------------------------------------------------
class StrictMount:
    """The mount the gimbal navigation gets: any attribute but the camera-state ones is a forbidden read."""

    ALLOWED = frozenset({"name", "gimbal", "gimbal_capture_history", "get_gimbal_data", "get_k", "is_valid",
                         "set_zoom", "camera"})

    def __init__(self, gimbal, history) -> None:
        object.__setattr__(self, "forbidden", [])
        object.__setattr__(self, "name", "zr10")
        object.__setattr__(self, "gimbal", gimbal)
        object.__setattr__(self, "gimbal_capture_history", history)

    def get_gimbal_data(self) -> GimbalData:
        return GimbalData(att=Attitude(-14.0, 0.0, 0.0))

    def __getattr__(self, name: str):
        self.forbidden.append(name)
        raise AttributeError(f"forbidden read of mount.{name}")


def navigation(revoked=lambda: False, tracking=True):
    clock, actuator = Clock(), Actuator()
    readback = Readback(clock)
    mount = StrictMount(actuator, readback)
    nav = GimbalNavigation(
        mount, Mock(), GimbalTrackingSetup(GimbalRateTrackerConfig()) if tracking else None,
        neutral_pitch_deg=-14.0, search=GimbalSearchSetup(pattern(), revoked))
    return nav, clock, actuator, readback, mount


def test_a_detection_session_start_stops_the_scan_before_its_lock_command():
    nav, clock, actuator, _, _ = navigation()
    with patch(CLOCK, clock):
        clock.now += 1 / 30
        nav.search_tick(False, True)
        actuator.calls.clear()
        nav.start_tracking(7)
    assert actuator.calls[:2] == [("rate", 0.0, 0.0), ("mode", MODE_LOCK)]


def test_a_geo_session_start_stops_the_scan_before_its_lock_command():
    nav, clock, actuator, _, _ = navigation()
    with patch(CLOCK, clock):
        clock.now += 1 / 30
        nav.search_tick(False, True)
        actuator.calls.clear()
        nav.start_geo_tracking(Mock(), Mock())
        clock.now += 1 / 30
        nav.search_tick(False, True)                    # the geo session owns the gimbal now
    assert actuator.calls == [("rate", 0.0, 0.0), ("mode", MODE_LOCK)]


def test_armed_but_never_tracked_scans_and_a_tracking_session_blocks():
    nav, clock, actuator, _, _ = navigation()
    search = nav._parts.visual._search
    with patch(CLOCK, clock):
        nav.start_tracking(7)
        clock.now += 1 / 30
        nav.search_tick(False, True)
        assert search.scanning
        clock.now += 1 / 30
        nav.search_tick(True, True)                     # the first detection: an accepted observation tracks
        nav.update(make_detected_poi(obj_id=7, x_error=1000.0, y_error=600.0, k=K, timestamp=clock.now,
                                     frame_width_px=WIDTH_PX, frame_height_px=HEIGHT_PX))
        clock.now += 2.5                                # past clear_s: only the tracking session blocks now
        nav.search_tick(False, True)
    assert search.reason == "tracking" and not search.scanning     # the tracker's own rates own the gimbal now


def test_a_first_acquisition_after_a_scan_in_an_armed_session_restores_lock():
    nav, clock, actuator, _, _ = navigation()
    with patch(CLOCK, clock):
        nav.start_tracking(7)
        clock.now += 1 / 30
        nav.search_tick(False, True)
        assert actuator.calls[-2:][0] == ("mode", MODE_FOLLOW)
        actuator.calls.clear()
        clock.now += 1 / 30
        nav.search_tick(True, True)
        nav.update(make_detected_poi(obj_id=7, x_error=1000.0, y_error=600.0, k=K, timestamp=clock.now,
                                     frame_width_px=WIDTH_PX, frame_height_px=HEIGHT_PX))
    assert actuator.calls[0] == ("rate", 0.0, 0.0) and ("mode", MODE_LOCK) in actuator.calls[1:]


def test_the_sink_ticks_the_search_first_with_any_fresh_detection():
    calls = []
    nav = Mock(tracking_obj_id=None)
    nav.search_tick.side_effect = lambda seen, new: calls.append(("search", seen, new))
    sink = TrackingNavigationSink(nav, FreshnessPolicy(1 / 60))
    with patch(CLOCK, return_value=100.0):
        sink.update([make_detected_poi(obj_id=3, timestamp=99.99)], new_frame=True)
        sink.update([make_detected_poi(obj_id=3, timestamp=99.0)])
        sink.update([])
    assert calls == [("search", True, True), ("search", False, False), ("search", False, False)]
    nav.update.assert_not_called()


def test_a_navigation_without_a_search_profile_ignores_the_tick():
    actuator = Actuator()
    nav = GimbalNavigation(StrictMount(actuator, Readback(Clock())), Mock(),
                           GimbalTrackingSetup(GimbalRateTrackerConfig()), neutral_pitch_deg=-14.0)
    nav.search_tick(False, True)
    assert actuator.calls == []


class FencedSdk:
    """The SDK end of the real SIYI writer: frames reach ``sent`` only through the command fence."""

    def __init__(self) -> None:
        self.sent: list[str] = []
        self._msg = SIYIMESSAGE()

    def sendMsg(self, msg: str) -> bool:
        self.sent.append(msg)
        return True

    def requestGimbalSpeed(self, yaw: int, pitch: int) -> bool:
        return self.sendMsg(self._msg.gimbalSpeedMsg(yaw, pitch))

    def requestFollowMode(self) -> bool:
        return self.sendMsg(self._msg.followModeMsg())


def test_the_scan_writes_through_the_fenced_siyi_writer_and_nothing_after_revoke():
    fence, sdk = SiyiCommandFence(), FencedSdk()
    session = SiyiSdkSession()
    session.install(fence.wrap(lambda **_: sdk)(server_ip="127.0.0.1", port=37260))
    writer = SiyiHardwareCommands(session, SiyiReadbackStore(GimbalData(att=Attitude(-14.0, 0.0, 0.0))), Mock())
    clock = Clock()
    search = GimbalSearch(GimbalSearchSetup(pattern(), lambda: fence.closed), writer, Readback(clock),
                          Mock(return_value=True), SLEW_DPS, Mock())
    with patch(CLOCK, clock):
        frames(search, clock, 3)
        commands = [bytes.fromhex(f)[7] for f in sdk.sent]
        assert commands[0] == 0x0C and set(commands[1:]) == {0x07}          # FOLLOW, then speed frames
        fence.close()
        sent = len(sdk.sent)
        frames(search, clock, 30)
    assert len(sdk.sent) == sent and fence.dropped == 0 and search.reason == "revoked"


def test_the_search_modules_read_no_pose_heading_or_truth():
    forbidden = {"heading", "location", "altitude", "relative_altitude", "ground_speed", "velocity", "vehicle",
                 "geo", "dock", "truth", "scenario"}
    for name in ("gimbal_search.py", "gimbal_search_pattern.py"):
        tree = ast.parse((REPO / "src/navpy/core/command/gimbal" / name).read_text(encoding="utf-8"))
        names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        imports = {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names}
        imports |= {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
        for word in forbidden:
            assert not any(word in item.lower() for item in names | imports), (name, word)


# ---- seeded replays ----------------------------------------------------------------------------------------------
FRAME_HZ, TICK_HZ = 30.0, 60.0
LANE_HEIGHT_M, AIRSPEED_M_S = 80.0, 24.0          # step 13's lane values (approach_lane, dock_pass)
LEVEL_DOCK_NORTH_M = 900.0                        # the replay's level dock, ahead of the aircraft's start
LATERAL_M = (0.0, 60.0, -60.0, 120.0, -120.0, 90.0, -90.0, 30.0, -30.0, 150.0)
DOCK_WIDTH_M, MIN_PIXELS = 3.5, 36.0              # detector_class_dimensions and dock_presets of siyi_zr10
K = np.array([[3273.052, 0.0, 960.0], [0.0, 3263.063, 540.0], [0.0, 0.0, 1.0]])   # the widest step, 2.0
WIDTH_PX, HEIGHT_PX = 1920.0, 1080.0
READBACK_LAG_S, COMMAND_LAG_S = 0.04, 0.02
REPLAY_SEEDS = tuple(range(2101, 2111))


class PlantGimbal:
    """navpy's simulated SIYI plant behind the actuator port, with command latency and the run's command fence:
    once ``close`` runs, every rate but a stop and every mode or angle command is dropped (the SiyiCommandFence's
    policy), and the fence's own stop goes out (``quiesce_and_stop``)."""

    def __init__(self, plant: GimbalAngularPlant) -> None:
        self.plant, self.pending, self.now, self.closed, self.rates = plant, [], 0.0, False, []

    def _queue(self, command) -> None:
        self.pending.append((self.now + COMMAND_LAG_S, command))

    def set_rate(self, yaw: float, pitch: float) -> None:
        self.rates.append((round(yaw), round(pitch)))
        if not self.closed or (round(yaw), round(pitch)) == (0, 0):
            self._queue(lambda: self.plant.set_speed(round(yaw), round(pitch)))

    def set_motion_mode(self, mode: int) -> None:
        if not self.closed:
            self._queue(lambda: self.plant.set_motion_mode(mode))

    def set_att(self, attitude: Attitude) -> None:
        if not self.closed:
            self._queue(lambda: self.plant.set_target_angles(attitude.yaw, attitude.pitch))

    def close(self) -> None:
        self.closed = True
        self._queue(lambda: self.plant.set_speed(0, 0))

    def advance(self, now: float, dt: float) -> None:
        self.now = now
        due = [cmd for at, cmd in self.pending if at <= now]
        self.pending = [(at, cmd) for at, cmd in self.pending if at > now]
        for command in due:
            command()
        self.plant.advance(dt)


@dataclass
class ReplayRecord:
    seed: int
    armed_early: bool = False
    first_detection_s: float | None = None
    scanned_before_detection: bool = False
    resumed_in_loss: bool = False
    seen_while_moving: int = 0
    stops_on_detection: list[float] = field(default_factory=list)
    plant_moving_after_stop: int = 0          # scan-only window: no tracker command can explain motion
    stop_window_ticks: int = 0
    stop_tick_rates: list[list[tuple[int, int]]] = field(default_factory=list)   # rates sent on each stop tick
    lock_while_tracking: bool = False
    revoke_s: float | None = None
    scanning_at_revoke: bool = False
    writes_at_revoke: int = 0
    writes_after_revoke: int = 0
    plant_moving_after_revoke: int = 0
    search_writes: int = 0
    forbidden_reads: list[str] = field(default_factory=list)
    stops: list[str] = field(default_factory=list)


def _pixel(dock_ned, aircraft_ned, yaw_deg, pitch_deg):
    """Truth side only: the dock's pixel and size in the camera (level aircraft heading north, FOLLOW joint
    angles), or None when it is behind, outside the image or too small to detect."""
    y, p = math.radians(yaw_deg), math.radians(pitch_deg)
    forward = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])
    right = np.array([-math.sin(y), math.cos(y), 0.0])
    down = np.cross(forward, right)
    rel = np.asarray(dock_ned) - np.asarray(aircraft_ned)
    depth = float(rel @ forward)
    if depth <= 1.0:
        return None
    u = K[0, 2] + K[0, 0] * float(rel @ right) / depth
    v = K[1, 2] + K[1, 1] * float(rel @ down) / depth
    size = K[0, 0] * DOCK_WIDTH_M / float(np.linalg.norm(rel))
    if not (0.0 <= u < WIDTH_PX and 0.0 <= v < HEIGHT_PX) or size < MIN_PIXELS:
        return None
    return u, v


def replay(seed: int, index: int, duration_s: float = 90.0) -> ReplayRecord:
    """Odd indexes arm the detection session before any detection (the scan runs while it is armed); even indexes
    revoke while the scan runs in the vision loss, odd ones 2 s or more after it, scanning or not."""
    rng, record = random.Random(seed), ReplayRecord(seed, armed_early=index % 2 == 1)
    offset = draw_dock(seed, 30.0)
    dock = (LEVEL_DOCK_NORTH_M + offset["north_mm"] / 1000.0, LATERAL_M[index] + offset["east_mm"] / 1000.0, 0.0)
    miss_p, start_delay_s = 0.1, 0.3
    loss_after_s, loss_len_s = rng.uniform(2.0, 4.0), 4.0
    plant = PlantGimbal(GimbalAngularPlant(initial_pitch=-14.0, initial_yaw=0.0))
    plant.plant.set_motion_mode(MODE_FOLLOW)
    clock, samples = Clock(0.0), []

    def history():
        lagged = [s for s in samples if s[0] <= clock.now - READBACK_LAG_S]
        if not lagged:
            return None
        t, att = lagged[-1]
        return GimbalCaptureHistory(1, (GimbalHistorySample(1, len(lagged), t + READBACK_LAG_S, att),),
                                    GimbalData(att))

    mount = StrictMount(plant, history)
    nav = GimbalNavigation(mount, Mock(), GimbalTrackingSetup(GimbalRateTrackerConfig()), neutral_pitch_deg=-14.0,
                           search=GimbalSearchSetup(GimbalSearchPattern.from_profile(PROFILE),
                                                    lambda: plant.closed))
    search = nav._parts.visual._search
    sink = TrackingNavigationSink(GimbalMeasurementAdapter(nav), FreshnessPolicy(1.0 / TICK_HZ))
    pois, start_at, loss = [], None, None
    tracking_at = 0.0 if record.armed_early else None
    dt, ticks = 1.0 / TICK_HZ, int(duration_s * TICK_HZ)
    settle_s = COMMAND_LAG_S + 2 * dt                  # a stop reaches the plant within the command latency
    with patch(CLOCK, clock):
        if record.armed_early:
            nav.start_tracking(7)
        for tick in range(ticks):
            clock.now = tick * dt
            plant.advance(clock.now, dt)
            snap = plant.plant.snapshot()
            samples.append((clock.now, Attitude(snap.pitch, snap.yaw, 0.0)))
            moving = (snap.yaw_command, snap.pitch_command) != (0.0, 0.0)
            new_frame = tick % int(TICK_HZ / FRAME_HZ) == 0
            if new_frame:
                aircraft = (AIRSPEED_M_S * clock.now, 0.0, -LANE_HEIGHT_M)
                blind = loss is not None and loss <= clock.now < loss + loss_len_s
                pixel = None if blind else _pixel(dock, aircraft, snap.yaw, snap.pitch)
                pois = [] if pixel is None or rng.random() < miss_p else [
                    make_detected_poi(obj_id=7, x_error=pixel[0], y_error=pixel[1], k=K, timestamp=clock.now,
                                      frame_width_px=WIDTH_PX, frame_height_px=HEIGHT_PX)]
            seen = any(clock.now - p.timing.detection_timestamp_s <= 0.25 for p in pois)
            moving_before, sent = search.scanning, len(plant.rates)
            sink.update(pois, new_frame=new_frame)
            if search.scanning and record.first_detection_s is None:
                record.scanned_before_detection = True
            if seen:
                if record.first_detection_s is None:
                    record.first_detection_s = clock.now
                    start_at = None if record.armed_early else clock.now + start_delay_s
                if moving_before and not search.scanning:
                    record.stops_on_detection.append(clock.now)
                    record.stop_tick_rates.append(plant.rates[sent:])
                record.seen_while_moving += int(search.scanning)
            last_stop = record.stops_on_detection[-1] if record.stops_on_detection else None
            if (not record.armed_early and last_stop is not None and tracking_at is None
                    and last_stop + settle_s <= clock.now < start_at):
                record.plant_moving_after_stop += int(moving)
                record.stop_window_ticks += 1
            if start_at is not None and tracking_at is None and clock.now >= start_at:
                nav.start_tracking(7)
                tracking_at = clock.now
            first = record.first_detection_s
            if first is not None and loss is None and clock.now >= first + start_delay_s + 0.5:
                record.lock_while_tracking = snap.motion_mode == MODE_LOCK
            if first is not None and loss is None and clock.now >= first + start_delay_s + loss_after_s:
                loss = clock.now
            if loss is not None and loss + 2.5 <= clock.now < loss + loss_len_s and search.scanning:
                record.resumed_in_loss = True
            revoke_at = None if loss is None else (
                loss + 3.0 + 0.1 * index if index % 2 == 0 else loss + loss_len_s + 2.0 + index * 0.5)
            if revoke_at is not None and record.revoke_s is None and clock.now >= revoke_at:
                record.scanning_at_revoke, record.writes_at_revoke = search.scanning, search.writes
                record.revoke_s = clock.now
                plant.close()                          # the terminal path: fence closed, its stop sent
            if record.revoke_s is not None and clock.now >= record.revoke_s + settle_s:
                record.plant_moving_after_revoke += int(moving)
    record.search_writes = search.writes
    record.writes_after_revoke = search.writes - record.writes_at_revoke
    record.forbidden_reads = list(mount.forbidden)
    record.stops = list(search._state.stops)
    return record


@pytest.mark.parametrize("index", range(len(REPLAY_SEEDS)))
def test_seeded_replay_stops_the_search_on_detection_and_revoke(index):
    record = replay(REPLAY_SEEDS[index], index)
    assert record.scanned_before_detection and record.first_detection_s is not None, record
    assert record.seen_while_moving == 0, record        # never moving on a tick with a fresh detection
    assert record.stops_on_detection, record
    # The scan's zero goes out before any tracker rate on the stop tick (the actuator is FIFO, so the plant applies
    # it first); with no session yet, the plant itself holds still until tracking starts.
    assert all(rates and rates[0] == (0, 0) for rates in record.stop_tick_rates), record
    if not record.armed_early:
        assert record.stop_window_ticks > 0 and record.plant_moving_after_stop == 0, record
    assert record.lock_while_tracking, record           # tracking runs in LOCK, armed early or not
    assert record.resumed_in_loss, record                # the vision loss returned to search
    assert record.scanning_at_revoke or index % 2 == 1, record
    assert record.revoke_s is not None and record.writes_after_revoke == 0, record
    assert record.plant_moving_after_revoke == 0, record
    assert "revoked" in record.stops and record.forbidden_reads == [], record
