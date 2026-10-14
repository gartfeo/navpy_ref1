"""The dock-pass detection report (tools/unreal/dock_pass_report.py): where the board is in the fixed body camera
from the truth pose, the board pose from its ChArUco corners, and the per-pass counting; and the live check's
--save-detected frames it reads."""
from __future__ import annotations

import math
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from tools.unreal import dock_pass_geometry as geo
from tools.unreal import dock_pass_report as rep
from tools.unreal.npyc_live_check import DETECTED_DIR, Recorder

BOARD = (40.3120563, 44.45815163, 1296.865)


def _truth_south_of_board(distance_m: float, height_m: float, yaw_deg: float = 0.0) -> geo.Truth:
    north_per_deg, _ = geo.metres_per_degree(BOARD[0])
    return geo.Truth(BOARD[0] - distance_m / north_per_deg, BOARD[1], BOARD[2] + height_m, 0.0, 0.0, yaw_deg)


def test_a_level_aircraft_sees_the_board_where_the_mount_pitch_puts_it(monkeypatch):
    monkeypatch.setattr(geo, "CAMERA_OFFSET_BODY_M", (0.0, 0.0, 0.0))
    # 20 m above and 20 m short of the board: 45 deg below the horizon, 10 deg below the camera's axis
    view = geo.board_view(_truth_south_of_board(20.0, 20.0), BOARD)
    k = geo.camera_matrix()
    assert view.in_view and view.slant_m == pytest.approx(20.0 * math.sqrt(2), rel=1e-3)
    assert view.agl_m == pytest.approx(20.0)
    centre = k @ view.centre_cam
    assert centre[0] / centre[2] == pytest.approx(geo.IMAGE_W / 2, abs=0.5)
    assert centre[1] / centre[2] == pytest.approx(geo.IMAGE_H / 2 + k[0, 0] * math.tan(math.radians(10.0)), abs=0.5)
    # the board's up normal points back at the camera's top (y up is -y in the image)
    assert view.normal_cam[1] < 0
    # flying away from it, or past it, the board is out of the picture
    assert not geo.board_view(_truth_south_of_board(20.0, 20.0, yaw_deg=180.0), BOARD).in_view
    assert not geo.board_view(_truth_south_of_board(-5.0, 20.0), BOARD).in_view


def test_the_camera_sits_its_mount_offset_ahead_of_and_below_the_vehicle(monkeypatch):
    forward, _, down = geo.CAMERA_OFFSET_BODY_M
    mounted = geo.board_view(_truth_south_of_board(20.0, 20.0), BOARD)
    monkeypatch.setattr(geo, "CAMERA_OFFSET_BODY_M", (0.0, 0.0, 0.0))
    moved = geo.board_view(_truth_south_of_board(20.0 - forward, 20.0 - down), BOARD)
    assert np.allclose(mounted.centre_cam, moved.centre_cam, atol=1e-3)
    assert mounted.slant_m == pytest.approx(moved.slant_m)          # the camera's range to the board ...
    assert mounted.agl_m == pytest.approx(20.0)                     # ... and the aircraft's height above it


def test_the_camera_offset_is_the_bench_measurement():
    # four static bench poses, ChArUco solvePnP (dock_pass_geometry.CAMERA_OFFSET_BODY_M): 2.33-2.61 m forward,
    # -0.04-0.01 m right, 0.10-0.38 m down
    forward, right, down = geo.CAMERA_OFFSET_BODY_M
    assert 2.33 <= forward <= 2.61 and -0.04 <= right <= 0.01 and 0.10 <= down <= 0.38


def test_corners_in_one_row_or_column_give_no_pose():
    per_row = geo.BOARD_SPEC.squares_x - 1
    assert not geo.spans_grid(np.arange(6) * per_row)            # one column
    assert not geo.spans_grid(np.arange(per_row))                # one row
    assert geo.spans_grid(np.array([0, 1, per_row, per_row + 1]))


def _render_board(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """The Unreal board texture (5 x 7, half a square margin) seen by the report's camera at this board pose."""
    board = geo.build_charuco_board(geo.BOARD_SPEC)
    px_per_square = 100
    margin = px_per_square // 2
    texture = board.generateImage((5 * px_per_square + 2 * margin, 7 * px_per_square + 2 * margin), marginSize=margin)
    side_m = geo.BOARD_SPEC.square_mm / 1000.0
    # texture pixel -> board metres (OpenCV board frame: origin at the grid corner)
    tex = np.array([[0, 0], [texture.shape[1], 0], [texture.shape[1], texture.shape[0]], [0, texture.shape[0]]],
                   dtype=np.float32)
    metres = (tex - margin) / px_per_square * side_m
    image_pts, _ = cv2.projectPoints(np.column_stack([metres, np.zeros(4)]).astype(float), rvec, tvec,
                                     geo.camera_matrix(), None)
    h = cv2.getPerspectiveTransform(tex, image_pts.reshape(-1, 2).astype(np.float32))
    return cv2.warpPerspective(texture, h, (geo.IMAGE_W, geo.IMAGE_H), borderValue=128)


def test_the_board_pose_from_its_corners_matches_the_rendered_pose():
    rvec = np.array([0.3, 0.1, -0.2])
    tvec = np.array([-1.0, -1.4, 6.0])
    pose = geo.detected_pose(_render_board(rvec, tvec))
    assert pose is not None and pose["count"] >= geo.MIN_PNP_CORNERS
    rot, _ = cv2.Rodrigues(rvec)
    centre = rot @ np.array([1.0, 1.4, 0.0]) + tvec
    assert np.linalg.norm(pose["centre"] - centre) < 0.05
    normal = rot @ np.array([0.0, 0.0, 1.0])
    normal = -normal if normal @ centre > 0 else normal
    assert math.degrees(math.acos(min(1.0, float(pose["normal"] @ normal)))) < 1.0


def test_a_pose_needs_corners_spanning_the_grid_and_a_solvable_pnp(monkeypatch):
    image = _render_board(np.array([0.3, 0.1, -0.2]), np.array([-1.0, -1.4, 6.0]))
    monkeypatch.setattr(geo, "spans_grid", lambda ids: False)
    assert geo.detected_pose(image) is None
    monkeypatch.undo()

    def unsolvable(*args, **kwargs):
        raise cv2.error("degenerate")
    monkeypatch.setattr(geo.cv2, "solvePnP", unsolvable)
    assert geo.detected_pose(image) is None


def test_no_board_no_pose():
    assert geo.detected_pose(np.full((geo.IMAGE_H, geo.IMAGE_W, 3), 128, np.uint8)) is None


def test_truth_is_matched_by_board_time_only_within_the_tolerance():
    states = {1_000_000: geo.Truth(*BOARD, 0, 0, 0), 1_020_000: geo.Truth(*BOARD, 0, 0, 0)}
    times = sorted(states)
    assert rep.truth_at(times, states, 1_020_000 + rep.TRUTH_MATCH_US) is states[1_020_000]
    assert rep.truth_at(times, states, 1_010_000) is None


def _row(index: int, board_us: int, flags: int = 3, detected: str = "") -> dict:
    return {"frame_index": str(index), "board_time_us": str(board_us), "pose_flags": str(flags),
            "detected": detected, "latency_ms": "120.0"}


def test_frames_are_scored_per_pass_window_with_exact_poses_only():
    truth = _truth_south_of_board(20.0, 20.0)
    times = [1_000_000, 2_000_000, 3_000_000]
    states = {t: truth for t in times}
    rows = [_row(1, 1_000_000, detected="1"), _row(2, 2_000_000, detected="0"), _row(3, 2_000_000, flags=1),
            _row(4, 3_000_000, detected="1")]
    recs = rep.score_frames(rows, times, states, BOARD, [(900, 2100)], None)
    assert [r["frame_index"] for r in recs] == [1, 2]        # 3: pose not exact; 4: outside every window
    summary = rep.summarize_group(recs)
    assert (summary["in_view"], summary["in_view_ran"], summary["detected"], summary["rate"]) == (2, 2, 1, 0.5)


def test_link_gate_reads_the_runs_health():
    run = {"vehicle": {"link_health": {"ok": True, "problems": [],
                                       "logs": {"a.BIN": {"stale_fraction": 0.03, "age_max_us": 12000}}}}}
    gate = rep.link_gate(run)
    assert gate["within_stale_5pct_age_50ms"] and gate["age_max_ms"] == 12.0
    assert rep.link_gate({}) == {"available": False}
    incomplete = {"vehicle": {"link_health": {"ok": False, "problems": ["HIL records lack ['LS']"],
                                              "logs": {"a.BIN": {"error": "HIL records lack ['LS']"}}}}}
    assert not rep.link_gate(incomplete)["within_stale_5pct_age_50ms"]
    mixed = {"vehicle": {"link_health": {"ok": False, "problems": [],
                                         "logs": {"a.BIN": {"stale_fraction": 0.03, "age_max_us": 12000},
                                                  "b.BIN": {"error": "HIL records lack ['LS']"}}}}}
    assert not rep.link_gate(mixed)["within_stale_5pct_age_50ms"]
    no_frames = {"vehicle": {"link_health": {"ok": False, "problems": [],
                                             "logs": {"a.BIN": {"stale_fraction": None, "age_max_us": None}}}}}
    assert not rep.link_gate(no_frames)["within_stale_5pct_age_50ms"]


def test_the_live_check_saves_detected_frames_unmarked(tmp_path):
    image = _render_board(np.array([0.3, 0.1, -0.2]), np.array([-1.0, -1.4, 6.0]))
    frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    recorder = Recorder(tmp_path, save_every=1000, detect_every=1, save_detected=True)
    recorder.rows[7] = {"detected": "", "confidence": ""}
    recorder._last_header = SimpleNamespace(frame_index=7)
    recorder.publish(frame, None)
    stop = threading.Event()
    stop.set()
    recorder.detect_loop(stop)
    saved = cv2.imread(str(tmp_path / DETECTED_DIR / "000007.png"))
    assert recorder.rows[7]["detected"] == 1 and np.array_equal(saved, frame)
