"""The static range check (tools/unreal/static_range_check.py): its poses put the board centre on the fixed camera's
optical axis at the asked slant range, and its pixel-per-square figure is the board's projected size."""
from __future__ import annotations

import math
import queue

import numpy as np
import pytest

from tools.tuning.dock_pass import DOCK_BOARD_YEGHVARD
from tools.tuning.hitl_bridge.unreal_pose import MAX_JSON_INTEGER
from tools.unreal import dock_pass_geometry as geo
from tools.unreal import dock_pass_report as rep
from tools.unreal import static_range_check as src

BOARD = tuple(float(v) for v in DOCK_BOARD_YEGHVARD.split(","))


@pytest.mark.parametrize("slant_m", [10.0, 30.0, 80.0])
@pytest.mark.parametrize("heading_deg", [0.0, 90.0, 225.0])
def test_the_pose_looks_at_the_board_centre_from_the_slant_range(slant_m, heading_deg):
    camera = geo.zr10_camera()
    truth = src.pose_for_range(BOARD, slant_m, heading_deg)
    view = geo.board_view(truth, BOARD, camera)
    assert (truth.roll, truth.pitch, truth.yaw) == (0.0, 0.0, heading_deg)
    assert view.slant_m == pytest.approx(slant_m, abs=1e-3)
    centre = camera.k @ view.centre_cam
    assert centre[:2] / centre[2] == pytest.approx([camera.cx, camera.cy], abs=0.05)
    # the board lies flat, so the camera sits the mount's depression above it (less the camera's mount offset)
    _, _, down = geo.CAMERA_OFFSET_BODY_M
    assert view.agl_m == pytest.approx(slant_m * math.sin(math.radians(-geo.MOUNT_PITCH_DEG)) + down, abs=1e-3)
    assert view.in_view


def test_pixels_per_square_is_the_focal_length_times_the_square_over_the_range():
    camera = geo.zr10_camera()
    slant = 25.0
    view = geo.board_view(src.pose_for_range(BOARD, slant, 0.0), BOARD, camera)
    across, along = src.pixels_per_square(view.corners_px)
    # across the line of sight the square is seen face on; along it, foreshortened by the 35 deg depression
    assert across == pytest.approx(camera.fx * src.SQUARE_M / slant, rel=0.02)
    assert along == pytest.approx(camera.fy * src.SQUARE_M / slant * math.sin(math.radians(35.0)), rel=0.05)


def test_the_zr10_sees_the_board_at_more_than_twice_the_pixels_of_the_90_deg_camera():
    truth = src.pose_for_range(BOARD, 20.0, 0.0)
    wide = src.pixels_per_square(geo.board_view(truth, BOARD, geo.UNREAL_FOV90).corners_px)
    narrow = src.pixels_per_square(geo.board_view(truth, BOARD, geo.zr10_camera()).corners_px)
    assert np.divide(narrow, wide) == pytest.approx([1407.42 / 640.0, 1401.91 / 640.0], rel=1e-3)


def test_a_new_pose_starts_at_the_next_sequence_sent():
    sender = src.PoseSender(("127.0.0.1", 9), src.YEGHVARD_UNDULATION_M, first_sequence=0)
    assert sender.set(src.pose_for_range(BOARD, 10.0, 0.0)) == 0
    sender._sequence = 41                           # 41 datagrams went out for the first pose
    assert sender.set(src.pose_for_range(BOARD, 20.0, 0.0)) == 41


def test_a_run_starts_above_every_sequence_an_earlier_run_sent(monkeypatch):
    # Unreal stamps frames with whatever sequence it applied last, from any sender: a run starting at 0 would take
    # an earlier run's still-showing pose for its own first pose
    start_ns = 1_760_000_000 * 10 ** 9
    monkeypatch.setattr(src.time, "time_ns", lambda: start_ns)
    earlier = src.PoseSender(("127.0.0.1", 9), src.YEGHVARD_UNDULATION_M)
    earlier_last = earlier.set(src.pose_for_range(BOARD, 10.0, 0.0)) + 3600 * 50   # it then ran an hour at 50 Hz
    monkeypatch.setattr(src.time, "time_ns", lambda: start_ns + 3600 * 10 ** 9)
    later = src.PoseSender(("127.0.0.1", 9), src.YEGHVARD_UNDULATION_M)
    first = later.set(src.pose_for_range(BOARD, 10.0, 0.0))
    assert earlier_last < first <= MAX_JSON_INTEGER


def _queue(*sequences: int) -> queue.Queue:
    frames: queue.Queue = queue.Queue()
    for sequence in sequences:
        frames.put((sequence, np.full((2, 2), sequence % 256, np.uint8)))
    return frames


def test_frames_of_an_earlier_pose_are_dropped():
    got = src.frames_of_pose(_queue(1000, 1001, 2000, 2001, 2002), first_sequence=2000, count=2, timeout_s=1.0)
    assert [int(f[0, 0]) for f in got] == [2000 % 256, 2001 % 256]


def test_the_wait_ends_at_the_timeout_even_while_earlier_frames_keep_coming():
    now = [0.0]

    def clock() -> float:
        now[0] += 0.4                                 # each look at the clock is 0.4 s later
        return now[0]
    got = src.frames_of_pose(_queue(*range(100)), first_sequence=10_000, count=3, timeout_s=1.0, clock=clock)
    assert got == []


@pytest.mark.parametrize("module", [src, rep])
def test_both_tools_score_with_the_zoom_unreal_was_given(module, monkeypatch, tmp_path):
    seen = {}

    def fake_camera(name: str, zoom: float = 1.0):
        seen["camera"] = (name, zoom)
        raise SystemExit(0)
    monkeypatch.setattr(module, "camera_named", fake_camera)
    args = ["--out", str(tmp_path), "--camera", "zr10", "--zoom", "2"]
    if module is rep:
        args += ["--run-json", "x", "--link-csv", "y", "--frames", "z"]
    with pytest.raises(SystemExit):
        module.main(args)
    assert seen["camera"] == ("zr10", 2.0)


def test_the_90_deg_camera_has_no_zoom():
    assert geo.camera_named("fov90") is geo.UNREAL_FOV90
    assert geo.camera_named("zr10", 2.0).fx == pytest.approx(2 * 1407.42)
    with pytest.raises(ValueError):
        geo.camera_named("fov90", 2.0)
