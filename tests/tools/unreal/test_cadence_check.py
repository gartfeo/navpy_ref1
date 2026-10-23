"""tools/unreal/cadence_check.py: the Unreal-convention math, the oracle and the run checks on synthetic runs that
follow the plugin's rules (NavPyVehicleStateReceiver admission and clock map, NavPyGimbalCameraActor
TickBoardTimeCadence)."""
from __future__ import annotations

import dataclasses
import json
import math
import socket
import threading
import zlib

import numpy as np
import pytest

from navpy.plugins.camera.unreal.npyc_codec import NPYC_PIXEL_FORMAT_BGR8, NPYC_VERSION_2, NpycHeader, encode_header
from tools.unreal import (cadence_check as cc, cadence_log as clog, cadence_oracle as co,
                          cadence_record as rec, unreal_math as um)
from tools.unreal.dock_pass_geometry import zr10_camera
from tools.unreal.pose_stamp_log import _angle_deg as quat_angle_deg

FPS = 30.0
POSE_HZ = 50.0
GIMBAL_HZ = 100.0
OFFSET_S = 1000.0            # board time 0 = PC (QPC) 1000 s, plus the pose transport delay
DELAY_S = 0.003
PORT = 17110
MOUNT = clog.Mount(receiver_port=PORT, pivot_cm=(20.0, 0.0, -15.0), camera_in_pitch_cm=(8.0, 0.0, -4.0),
                 camera_in_pitch_quat=(0.0, 0.0, 0.0, 1.0), stabilize_roll=True, min_pitch=-90.0, max_pitch=25.0,
                 fps=FPS)
BOARD = clog.BoardPose(location=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0), width_cm=240.0, height_cm=320.0)


# --- Unreal conventions --------------------------------------------------------------------------------------------
def test_rotator_quat_matches_unreal_values():
    # FRotator(0, 90, 0).Quaternion() = (0, 0, sin 45, cos 45); pitch up 90 turns X onto Z.
    assert um.rotator_to_quat(0, 90, 0) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    assert um.quat_rotate(um.rotator_to_quat(90, 0, 0), (1, 0, 0)) == pytest.approx((0, 0, 1), abs=1e-12)
    assert um.quat_rotate(um.rotator_to_quat(0, 90, 0), (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-12)
    for rot in ((-30.0, 120.0, 10.0), (89.0, -170.0, -45.0), (0.0, 0.0, 179.0)):
        assert um.quat_to_rotator(um.rotator_to_quat(*rot)) == pytest.approx(rot, abs=1e-9)


@pytest.mark.parametrize("rot", [(-89.95, 20.0, 30.0), (89.95, 20.0, 30.0), (-89.99, -150.0, -70.0)])
def test_rotator_at_the_poles_follows_ue57(rot):
    # FQuat4d::Rotator: past the singularity threshold the pitch is +-90, the roll 0 and the yaw +-2 atan2(X, W).
    pitch, yaw, roll = um.quat_to_rotator(um.rotator_to_quat(*rot))
    assert abs(pitch) == 90.0 and roll == 0.0
    # The same rotation to within the pitch's distance from the pole.
    assert quat_angle_deg(um.rotator_to_quat(pitch, yaw, roll), um.rotator_to_quat(*rot)) < 2 * (90 - abs(rot[0]))


def test_rotator_pole_value_from_the_engine_formula():
    # Codex's independent scalar evaluation of UE 5.7's singular branch: yaw 49.982906, roll 0.
    pitch, yaw, roll = um.quat_to_rotator(um.rotator_to_quat(-89.95, 20.0, 30.0))
    assert (pitch, roll) == (-90.0, 0.0) and yaw == pytest.approx(49.982906, abs=1e-6)


def test_compose_gimbal_points_and_stabilises():
    vehicle = um.rotator_to_quat(5.0, 40.0, 20.0)                     # pitched, yawed, banked
    camera = um.compose_gimbal(vehicle, 0.0, -90.0, 0.0, zero_world_roll=False)
    forward = um.quat_rotate(camera, (1, 0, 0))
    assert forward == pytest.approx(um.quat_rotate(vehicle, um.quat_rotate(um.rotator_to_quat(-90, 0, 0), (1, 0, 0))))
    level = um.compose_gimbal(vehicle, 10.0, -20.0, 0.0, zero_world_roll=True)
    assert um.quat_to_rotator(level)[2] == pytest.approx(0.0, abs=1e-9)


def test_projection_centre_and_axes():
    camera = zr10_camera()
    look = um.rotator_to_quat(0, 0, 0)                                # X forward, Y right, Z up
    px = co.project(np.array([[1000.0, 0, 0], [1000.0, 100.0, 0], [1000.0, 0, 100.0]]), (0, 0, 0), look, camera)
    assert px[0] == pytest.approx([camera.cx, camera.cy])
    assert px[1][0] > camera.cx and px[2][1] < camera.cy              # right is +u, up is -v
    # Skew as OpenCV's K: u = fx x/z + skew y/z + cx with y down, so a point above the axis moves left.
    skewed = dataclasses.replace(camera, skew=50.0)
    px = co.project(np.array([[1000.0, 0, 100.0]]), (0, 0, 0), look, skewed)
    assert px[0] == pytest.approx([camera.cx - 5.0, camera.cy - camera.fy * 0.1])


def test_slot_time_rounds_half_up():
    assert um.slot_time_us(1000, 1, 30.0) == 1000 + 33333
    assert um.slot_time_us(0, 1, 2e6) == 1                            # 0.5 us rounds up, as FMath::RoundToDouble


# --- synthetic run -------------------------------------------------------------------------------------------------
def vehicle_at(t_s: float, gen: int) -> tuple[tuple, tuple]:
    shift = 0.0 if gen == 1 else 5000.0                               # a reboot reuses board times elsewhere
    loc = (-3000.0 + 300.0 * t_s + shift, 50.0 * math.sin(t_s), 2500.0 - 40.0 * t_s)
    return loc, um.rotator_to_quat(-3.0 + 2.0 * math.sin(1.3 * t_s), 5.0 * t_s, 15.0 * math.sin(0.7 * t_s))


def gimbal_angles(qpc_s: float) -> tuple[float, float, float]:
    t = qpc_s - OFFSET_S
    return 8.0 * math.sin(0.9 * t), -40.0 + 10.0 * math.sin(1.7 * t), 2.0 * math.sin(2.3 * t)


def gimbal_steps(seconds: float) -> list[clog.GimbalStep]:
    return [clog.GimbalStep(OFFSET_S + i / GIMBAL_HZ, i + 1, *gimbal_angles(OFFSET_S + i / GIMBAL_HZ), roll_stabilized=True)
            for i in range(int(seconds * GIMBAL_HZ) + 2)]


def synthetic_run(seconds: float = 4.0, *, pose_lag_us: int = 0, gimbal_lag_s: float = 0.0, query_lag_s: float = 0.0,
                  mount: clog.Mount = MOUNT, repeat_frame: int | None = None, jitter_us: int = 0, gens: int = 1,
                  dropped: bool = False):
    """Unreal's log and the recording for a clean run at FPS, built with the plugin's rules. The lags, the mount
    and the jitter stand for the bugs the checker must catch: a stale pose, a stale gimbal sample, a stale gimbal
    query (logged as it was made), a lost mount offset, captures at tick time instead of slot time. ``dropped``
    adds a pose the receiver dropped (10 m off) between every two it admitted: never a source."""
    log = clog.UnrealLog(mounts={"run1": MOUNT}, board=BOARD)
    frames = []
    steps = gimbal_steps(gens * seconds + 1.0)
    step_times = [s.t for s in steps]
    frame = 0
    for gen in range(1, gens + 1):
        # Generation g's board clock starts again at 0, (g - 1) * seconds later on the PC clock.
        pc_base = OFFSET_S + (gen - 1) * seconds
        rng = np.random.default_rng(gen)
        admits = []
        for i in range(int(seconds * POSE_HZ) + 2):
            board_us = round(i * 1e6 / POSE_HZ)
            loc, quat = vehicle_at(board_us / 1e6, gen)
            receipt = pc_base + board_us / 1e6 + DELAY_S + float(rng.uniform(0.0, 0.002))
            admits.append(clog.Admit(PORT, gen, True, board_us, receipt, loc, quat))
            log.admits.append(admits[-1])
            if dropped:
                log.admits.append(clog.Admit(PORT, gen, False, board_us + 10000, receipt - 0.001,
                                           (loc[0] + 1000.0, loc[1], loc[2]), quat))
        log.generations.add((PORT, gen))
        stream = co.StreamSources(admits)
        origin = 500000                                               # from 0.5 s: the lagged fixtures stay inside
        slots = int((seconds - 0.6) * FPS)
        for slot in range(slots):
            board_us = um.slot_time_us(origin, slot, FPS)
            if jitter_us and 0 < slot < slots - 1:
                board_us += jitter_us if slot % 2 else -jitter_us
            # The capture's map_n: the poses admitted by then (the interpolator renders 100 ms behind the newest).
            n = sum(1 for a in admits if a.board_us <= board_us + 100000)
            offset = stream.clock_offset(n, board_us)
            query = None if offset is None else board_us / 1e6 + offset - query_lag_s
            pose = stream.pose(n, board_us - pose_lag_us)
            gimbal = co.gimbal_at(steps, step_times, (pc_base + board_us / 1e6 if query is None else query) - gimbal_lag_s)
            rendered = co.expected_camera(pose, gimbal, mount)
            picture = slot if repeat_frame is None or slot != repeat_frame + 1 else repeat_frame
            crc = zlib.crc32(f"picture {gen} {picture}".encode())
            log.slots.append({
                "outcome": "sent", "run": "run1", "gen": str(gen), "origin_us": str(origin), "slot": str(slot),
                "board_us": str(board_us), "frame": str(frame), "crc": f"{crc:08x}",
                "mapped": "0" if offset is None else "1",
                "offset_s": repr(0.0 if offset is None else offset - query_lag_s), "map_n": str(n),
                "gimbal_qpc": repr(0.0 if query is None else query), "pose_valid": "1", "gimbal_valid": "1",
                "cam_loc": ",".join(map(repr, rendered.location)), "cam_quat": ",".join(map(repr, rendered.quat))})
            frames.append({"conn": 1, "frame": frame, "board_us": board_us, "crc": crc, "corners": None})
            frame += 1
        log.stream_ends[(PORT, gen)] = [slots]
    return log, frames, steps


def test_clean_run_passes_the_oracle_and_the_run_checks():
    log, frames, steps = synthetic_run()
    report = cc.check_run(log, frames, steps)
    for part in ("identity", "joins", "grid", "coverage", "unique_rate", "repeated_pictures", "oracle"):
        assert report[part]["pass"], (part, report[part])
    assert report["oracle"]["max_angle_deg"] < 1e-6 and report["oracle"]["max_location_cm"] < 1e-6


def test_reboot_reusing_board_times_scores_each_stream_on_its_own_poses():
    log, frames, steps = synthetic_run(gens=2)
    report = cc.check_run(log, frames, steps)
    assert report["oracle"]["pass"], report["oracle"]
    assert report["coverage"]["pass"] and report["unique_rate"]["pass"] and report["grid"]["pass"]
    assert set(report["coverage"]["streams"]) == {1, 2}


@pytest.mark.parametrize("bug", [{"pose_lag_us": 20000}, {"gimbal_lag_s": 0.020}, {"query_lag_s": 0.020},
                                 {"mount": clog.Mount(PORT, (20.0, 0.0, -15.0), (0.0, 0.0, 0.0), (0, 0, 0, 1), True,
                                                    -90.0, 25.0, FPS)}],
                         ids=["stale-pose", "stale-gimbal", "stale-gimbal-query", "no-mount-offset"])
def test_oracle_fails_each_timing_or_mount_bug(bug):
    log, frames, steps = synthetic_run(**bug)
    oracle = cc.check_run(log, frames, steps)["oracle"]
    assert not oracle["pass"]
    assert oracle["unscored"] == 0                                    # failed on the transform or clock, not on gaps
    assert oracle["max_angle_deg"] > co.ANGLE_TOLERANCE_DEG or oracle["max_location_cm"] > co.LOCATION_TOLERANCE_CM


def test_stale_query_fails_on_the_rebuilt_clock():
    log, frames, steps = synthetic_run(query_lag_s=0.020)
    oracle = cc.check_run(log, frames, steps)["oracle"]
    assert oracle["clock_mismatch"] == oracle["scored"] > 0


def test_off_grid_capture_times_fail():
    log, frames, steps = synthetic_run(jitter_us=5000)
    report = cc.check_run(log, frames, steps)
    assert not report["grid"]["pass"] and report["grid"]["off_grid"] > 0


def test_repeated_picture_and_bad_joins_fail():
    log, frames, steps = synthetic_run(repeat_frame=40)
    assert not cc.check_run(log, frames, steps)["repeated_pictures"]["pass"]
    log, frames, steps = synthetic_run()
    frames[10]["crc"] ^= 1
    assert not cc.check_run(log, frames, steps)["joins"]["pass"]


@pytest.mark.parametrize("mutate", ["recorded_twice", "sent_twice", "two_runs", "lost_reuses_sent", "lost_twice",
                                    "reconnect_reuses_frame"])
def test_duplicate_identities_fail(mutate):
    log, frames, steps = synthetic_run()
    if mutate == "recorded_twice":
        frames.append(dict(frames[5]))
    elif mutate == "sent_twice":
        log.slots.append(dict(log.slots[5]))
    elif mutate == "two_runs":
        log.slots[-1] = dict(log.slots[-1], run="run2")
        log.mounts["run2"] = MOUNT
    elif mutate == "lost_reuses_sent":
        log.slots[20] = dict(log.slots[20], outcome="lost:readback-failed", frame=log.slots[21]["frame"])
        frames = [f for f in frames if f["frame"] != 20]
    elif mutate == "lost_twice":
        log.slots[20] = dict(log.slots[20], outcome="lost:readback-failed")
        log.slots[21] = dict(log.slots[21], outcome="lost:readback-failed", frame=log.slots[20]["frame"])
        frames = [f for f in frames if f["frame"] not in (20, 21)]
    else:
        frames[-1] = dict(frames[-2], conn=2)                         # after a reconnect, a frame index again
    report = cc.check_run(log, frames, steps)
    assert not report["identity"]["pass"] and not report["pass"]


def _reconnect_with_unrecorded(log, frames, close, missing):
    """The recorder closes after frame ``close - 1`` was sent: frame ``close`` ends lost:disconnect, the frames after
    it go to connection 2, and the ``missing`` sent frames are not in the recording."""
    slot = next(s for s in log.slots if s.get("frame") == str(close))
    slot["outcome"] = "lost:disconnect"
    frames = [dict(f, conn=1 if f["frame"] < close else 2) for f in frames if f["frame"] != close]
    return [f for f in frames if f["frame"] not in missing]


def test_the_frame_in_flight_at_a_recorder_close_may_be_missing():
    log, frames, steps = synthetic_run()
    frames = _reconnect_with_unrecorded(log, frames, 60, {59})
    joins = cc.check_run(log, frames, steps)["joins"]
    assert joins["pass"] and joins["in_flight_at_close"] == [59]


@pytest.mark.parametrize("where", ["two_at_one_close", "mid_connection", "at_the_end", "after_the_close"])
def test_other_unrecorded_sent_frames_fail_the_joins(where):
    log, frames, steps = synthetic_run()
    missing = {"two_at_one_close": {58, 59}, "mid_connection": {30}, "at_the_end": {frames[-1]["frame"]},
               "after_the_close": {61}}[where]                         # 61: the new connection's first frame
    frames = _reconnect_with_unrecorded(log, frames, 60, missing)
    joins = cc.check_run(log, frames, steps)["joins"]
    assert not joins["pass"] and joins["unrecorded"]


def test_a_reconnect_without_a_logged_close_excuses_nothing():
    log, frames, steps = synthetic_run()
    frames = [dict(f, conn=1 if f["frame"] < 60 else 2) for f in frames if f["frame"] != 59]
    joins = cc.check_run(log, frames, steps)["joins"]
    assert not joins["pass"] and joins["unrecorded"] == [59]


def test_a_reconnect_of_the_same_run_is_one_recording():
    log, frames, steps = synthetic_run()
    for f in frames[60:]:
        f["conn"] = 2
    report = cc.check_run(log, frames, steps)
    assert report["identity"]["pass"] and report["identity"]["connections"] == 2 and report["joins"]["pass"]


def test_coverage_counts_missed_ranges_and_finds_holes():
    log, frames, steps = synthetic_run()
    gone = {"5", "6", "7", "9"}
    log.slots = [s for s in log.slots if s["slot"] not in gone]
    log.slots.append({"outcome": "missed:late-tick", "run": "run1", "gen": "1", "origin_us": "500000", "slot": "5",
                      "count": "3"})
    frames = [f for f in frames if str(f["frame"]) not in gone]
    report = cc.check_run(log, frames, steps)
    assert report["coverage"]["missed"] == 3
    assert not report["coverage"]["pass"]                             # slot 9 is neither sent, lost nor missed
    log.slots.append({"outcome": "lost:socket-busy", "run": "run1", "gen": "1", "origin_us": "500000", "slot": "9",
                      "board_us": str(um.slot_time_us(500000, 9, FPS))})
    assert cc.check_run(log, frames, steps)["coverage"]["pass"]


def test_coverage_fails_an_unfinished_tail_or_a_missing_stream_end():
    log, frames, steps = synthetic_run()
    log.stream_ends[(PORT, 1)] = [log.stream_ends[(PORT, 1)][0] + 3]  # three consumed slots never ended
    assert not cc.check_run(log, frames, steps)["coverage"]["pass"]
    log.stream_ends.clear()
    assert not cc.check_run(log, frames, steps)["coverage"]["pass"]


def test_coverage_fails_a_started_generation_without_outcomes_or_end():
    log, frames, steps = synthetic_run()
    assert cc.check_run(log, frames, steps)["coverage"]["pass"]
    log.generations.add((PORT, 2))                                    # a new stream began; the log stops before
    assert not cc.check_run(log, frames, steps)["coverage"]["pass"]   # any of its outcomes or its end


# --- reprojection --------------------------------------------------------------------------------------------------
def board_view_run(lag_frames: int, bad_corners: int = 0):
    """A camera sweeping over the board; the detected corners come from the camera lag_frames earlier, and the
    first bad_corners of each frame are 10 px off."""
    camera = zr10_camera()
    corners = co.board_corners_world(BOARD, 1.0, -1.0)
    log = clog.UnrealLog(board=BOARD)
    frames, poses = [], []
    for slot in range(120):
        t = slot / FPS
        poses.append(((-600.0 + 200.0 * t, 30.0 * math.sin(2 * t), 900.0), um.rotator_to_quat(-80.0, 5.0 * t, 0.0)))
    for slot in range(len(poses)):
        loc, quat = poses[slot]
        shown_loc, shown_quat = poses[max(0, slot - lag_frames)]
        px = co.project(corners, shown_loc, shown_quat, camera)
        detected = [[i, float(u), float(v)] for i, (u, v) in enumerate(px)
                    if 0 <= u < camera.width and 0 <= v < camera.height]
        for c in detected[:bad_corners]:
            c[1] += 10.0
        log.slots.append({"outcome": "sent", "frame": str(slot), "cam_loc": ",".join(map(repr, loc)),
                          "cam_quat": ",".join(map(repr, quat))})
        frames.append({"frame": slot, "corners": detected})
    return log, {f["frame"]: f for f in frames}, camera


def test_reprojection_passes_on_time_and_fails_one_frame_late():
    log, frames, camera = board_view_run(lag_frames=0)
    report = co.check_reprojection(log, log.slots, frames, lambda _s: camera)
    assert report["pass"], report
    assert report["flips"] == (1.0, -1.0) and report["p95_px"] < 1e-6
    log, frames, camera = board_view_run(lag_frames=1)
    report = co.check_reprojection(log, log.slots, frames, lambda _s: camera)
    assert not report["pass"]
    assert report["own_median_px"] > report["neighbour_median_px"]


def test_reprojection_p95_is_over_corners_not_frame_means():
    # Two corners of every frame 10 px off: each frame's mean stays small, but over 5 % of the corners are wrong.
    log, frames, camera = board_view_run(lag_frames=0, bad_corners=2)
    counts = [n for f in frames.values() if (n := len(f["corners"])) >= co.MIN_REPROJECTION_CORNERS]
    assert 2 * len(counts) / sum(counts) > 0.05
    assert np.median([20.0 / n for n in counts]) < co.REPROJECTION_P95_PX
    report = co.check_reprojection(log, log.slots, frames, lambda _s: camera)
    assert report["p95_px"] == pytest.approx(10.0, abs=1e-6) and not report["pass"]


# --- logs ----------------------------------------------------------------------------------------------------------
def test_parse_log_and_emulator_log():
    lines = [
        "[2026.10.20-10.00.00:000][  1]LogTemp: NavPyCameraMount sys_id=1 run=abc receiver_port=17110 pivot_cm=1,2,3 "
        "camera_in_pitch_cm=4,5,6 camera_in_pitch_quat=0,0,0,1 stabilize_roll=1 compose=1 min_pitch=-90 max_pitch=25 "
        "fps=30",
        "[2026.10.20-10.00.00:000][  1]LogTemp: NavPyBoardPose loc=10,20,30 quat=0,0,0,1 scale=2.4,3.2,1 "
        "width_cm=240 height_cm=320",
        "[2026.10.20-10.00.00:000][  1]LogTemp: NavPyPoseAdmit port=17110 gen=2 admitted=1 board_us=20000 seq=7 "
        "receipt_qpc=1234.5678901 loc=1,2,3 quat=0,0,0,1",
        "[2026.10.20-10.00.00:000][  1]LogTemp: NavPyRenderClock port=17110 event=stream_end gen=2 next_slot=40",
        "[2026.10.20-10.00.00:000][  1]LogTemp: NavPyCaptureSlot outcome=missed:late-tick sys_id=1 run=abc gen=2 "
        "origin_us=0 slot=7 count=2",
    ]
    log = clog.parse_log(lines)
    assert log.mounts["abc"] == clog.Mount(17110, (1, 2, 3), (4, 5, 6), (0, 0, 0, 1), True, -90, 25, 30)
    assert log.board == clog.BoardPose((10, 20, 30), (0, 0, 0, 1), 240, 320)
    assert log.admits == [clog.Admit(17110, 2, True, 20000, 1234.5678901, (1, 2, 3), (0, 0, 0, 1))]
    assert log.stream_ends == {(17110, 2): [40]}
    assert log.slots == [{"outcome": "missed:late-tick", "sys_id": "1", "run": "abc", "gen": "2", "origin_us": "0",
                          "slot": "7", "count": "2"}]
    step = {"event": "step", "t": 12.5, "seq": 4, "true": {"yaw": 1, "pitch": -30, "roll": 0.5, "zoom": 1,
                                                          "motion_mode": 2}}
    steps = clog.parse_emulator_log([json.dumps({"event": "start"}), json.dumps(step)])
    assert len(steps) == 1 and steps[0].t == 12.5 and steps[0].pitch == -30


def test_gimbal_query_falls_back_to_the_nearest_step_on_a_broken_bracket():
    steps = [clog.GimbalStep(10.000, 1, 0.0, 0.0, 0.0, True), clog.GimbalStep(10.040, 3, 40.0, 0.0, 0.0, True)]
    step, valid = co.gimbal_at(steps, [s.t for s in steps], 10.010)
    assert not valid and step.yaw == 0.0


def test_consecutive_steps_of_two_sessions_are_not_a_bracket():
    # The plugin clears its history on a new gimbal session, so a restart never brackets across the sessions.
    steps = [clog.GimbalStep(10.000, 1, 0.0, 0.0, 0.0, True, session="a"),
             clog.GimbalStep(10.020, 2, 40.0, 0.0, 0.0, True, session="b")]
    bracket = co.gimbal_bracket(steps, [s.t for s in steps], 10.015)
    assert not bracket.valid and bracket.older is bracket.newer is steps[1]


def test_dropped_poses_are_not_sources():
    log, frames, steps = synthetic_run(dropped=True)
    assert sum(not a.admitted for a in log.admits) > 0
    assert cc.check_run(log, frames, steps)["oracle"]["pass"]
    # Used as sources, the dropped poses would move the vehicle 10 m between samples.
    log.admits = [clog.Admit(a.port, a.gen, True, a.board_us, a.receipt_qpc, a.location, a.quat) for a in log.admits]
    assert not cc.check_run(log, frames, steps)["oracle"]["pass"]


# --- record --------------------------------------------------------------------------------------------------------
def frame_bytes(index: int, pixels: bytes) -> bytes:
    header = NpycHeader(NPYC_VERSION_2, 64, 48, NPYC_PIXEL_FORMAT_BGR8, index, 0.0, len(pixels),
                        board_time_us=1000 * index, pose_sequence=index, pose_flags=1)
    return encode_header(header) + pixels


class StallingSocket:
    """Serves the bytes in small chunks with a receive timeout before every chunk (a stalled sender)."""

    def __init__(self, data: bytes, chunk: int):
        self.data, self.chunk = data, chunk
        self.stall = False

    def recv(self, size: int) -> bytes:
        self.stall = not self.stall
        if self.stall:
            raise socket.timeout
        part, self.data = self.data[:min(size, self.chunk)], self.data[min(size, self.chunk):]
        return part


def test_read_frames_keeps_frame_boundaries_across_timeouts():
    pixels = bytes(range(256)) * (64 * 48 * 3 // 256)
    conn = StallingSocket(frame_bytes(0, pixels) + frame_bytes(1, pixels), chunk=1000)
    got = []
    stop = rec.read_frames(conn, deadline=float("inf"),
                          on_frame=lambda header, data: got.append((header.frame_index, data)))
    assert stop == "closed" and got == [(0, pixels), (1, pixels)]


class StreamingSocket:
    """Serves the bytes in small chunks and never times out (a sender that keeps up); counts the bytes taken."""

    def __init__(self, data: bytes, chunk: int):
        self.data, self.chunk, self.taken = data, chunk, 0

    def recv(self, size: int) -> bytes:
        part = self.data[self.taken:self.taken + min(size, self.chunk)]
        self.taken += len(part)
        return part


@pytest.mark.parametrize("expire_at, stop", [
    ("frame", "deadline"),              # between frames
    ("prefix", "cut"),                  # inside the next prefix
    ("pixels", "cut"),                  # inside the next frame's pixels
])
def test_read_frames_stops_at_the_deadline_while_bytes_keep_coming(expire_at, stop):
    pixels = bytes(64 * 48 * 3)
    one = frame_bytes(0, pixels)
    header_size = len(one) - len(pixels)
    limit = {"frame": len(one), "prefix": len(one) + 10, "pixels": len(one) + header_size + 100}[expire_at]
    conn = StreamingSocket(one * 3, chunk=10)
    got = []
    clock = lambda: 1.0 if conn.taken >= limit else 0.0                # the time runs out at that byte
    assert rec.read_frames(conn, 0.5, lambda *a: got.append(a), clock=clock) == stop
    assert len(got) == 1 and conn.taken == limit                      # no read after the deadline


def test_record_keeps_header_crc_and_corners(tmp_path):
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    out = tmp_path / "frames.jsonl"
    result, listening = {}, threading.Event()
    thread = threading.Thread(target=lambda: result.update(rec.record(("127.0.0.1", port), out, seconds=3.0,
                                                                     listening=listening)))
    thread.start()
    assert listening.wait(5.0)
    pixels = bytes(range(256)) * (64 * 48 * 3 // 256)
    with socket.create_connection(("127.0.0.1", port), timeout=1.0) as sender:
        for index in range(3):
            sender.sendall(frame_bytes(index, pixels))
    thread.join()
    rows = [json.loads(line) for line in out.read_text().splitlines()]
    assert sorted(r["frame"] for r in rows) == [0, 1, 2]
    assert all(r["crc"] == zlib.crc32(pixels) and r["corners"] == [] and r["conn"] == 1 for r in rows)
    assert {r["board_us"] for r in rows} == {0, 1000, 2000} and result["frames"] == 3
