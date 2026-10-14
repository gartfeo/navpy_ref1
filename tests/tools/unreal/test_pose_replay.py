"""tools/unreal/pose_replay.py: the replay's slot rule, its pace and stamp metrics and the Unreal log reader."""
from __future__ import annotations

import math

import pytest

from tools.tuning.hitl_bridge.physics import PlantState
from tools.unreal import pose_replay as pr
from tools.unreal import pose_stamp_log as psl


def _pose(board_us: int, lat: float = 40.0, alt: float = 1000.0) -> pr.Pose:
    return pr.Pose(board_us, PlantState(lat, 44.0, alt, 0.0, 0.0, 0.0, *([0.0] * 11)))


def test_the_replay_keeps_the_bridges_first_pose_of_each_slot():
    poses = [_pose(t) for t in range(0, 100_000, 4_000)]
    assert [p.board_time_us for p in pr.sent_poses(poses)] == [0, 20_000, 40_000, 60_000, 80_000]


def test_a_steady_pace_is_one_and_repeats_are_counted():
    steady = pr.pace_stats([0.0, 0.02, 0.04, 0.06], [0, 20_000, 40_000, 60_000])
    assert steady["max"] == pytest.approx(0.0, abs=1e-9) and steady["repeated_share"] == 0.0
    # A 60 Hz game showing 50 Hz poses: some ticks repeat the pose, some jump two.
    jerky = pr.pace_stats([0.0, 1 / 60, 2 / 60, 3 / 60], [0, 0, 20_000, 40_000])
    assert jerky["repeated_share"] == pytest.approx(1 / 3, abs=1e-3) and jerky["max"] == pytest.approx(1.0)


def test_freezes_are_reported_as_gaps_and_left_out_of_the_pace():
    frames = [pr.Frame(0.0, 0, 3), pr.Frame(0.05, 50_000, 3), pr.Frame(3.05, 3_050_000, 5), pr.Frame(3.1, 3_100_000, 5)]
    stats = pr.frame_stats(frames)
    assert stats["gaps_over_s"] == [3.0] and stats["longest_gap_s"] == 3.0
    assert (stats["stamped"], stats["interpolated"], stats["pace"]["n"]) == (4, 2, 2)


LOG = """\
[2026.10.14-18.00.00:000][  1]LogTemp: Display: NavPyPoseSample board_us=1000000 seq=1 loc=0.000,0.000,100.000 quat=0.0000000,0.0000000,0.0000000,1.0000000
[2026.10.14-18.00.00:010][  1]LogTemp: Display: NavPyPoseSample board_us=1020000 seq=2 loc=200.000,0.000,100.000 quat=0.0000000,0.0000000,0.7071068,0.7071068
[2026.10.14-18.00.00:016][  2]LogTemp: Display: NavPyPoseApplied board_us=1010000 seq=1 valid=1 exact=0 interp=1 loc=100.000,0.000,100.000 quat=0.0000000,0.0000000,0.3826834,0.9238795
[2026.10.14-18.00.00:033][  3]LogTemp: Display: NavPyPoseApplied board_us=1015000 seq=1 valid=1 exact=0 interp=1 loc=160.000,0.000,100.000 quat=0.0000000,0.0000000,0.5555702,0.8314696
[2026.10.14-18.00.10:000][600]LogTemp: Display: NavPyMoveCost moves=600 mean_ms=0.0500 max_ms=0.2000
"""


def test_the_log_is_read_and_each_applied_pose_is_checked_against_its_stamp():
    samples, applied, moves = psl.parse_unreal_log(LOG.splitlines())
    assert [s.board_time_us for s in samples] == [1_000_000, 1_020_000]
    assert [a.board_time_us for a in applied] == [1_010_000, 1_015_000] and applied[0].interpolated
    assert applied[1].log_s - applied[0].log_s == pytest.approx(0.017)
    assert moves == [{"moves": 600, "mean_ms": 0.05, "max_ms": 0.2}]
    check = psl.stamp_check(samples, applied)
    assert check["checked"] == 2
    # The first is exactly the midpoint; the second is 10 cm off the 3/4 point (150 cm) at the right attitude.
    assert check["position_cm"]["max"] == pytest.approx(10.0, abs=1e-3)
    assert check["attitude_deg"]["max"] == pytest.approx(0.0, abs=1e-3)


def test_interpolating_the_sent_poses_is_measured_against_the_rows_between_them():
    # The real path bulges 1 m up between two 20 ms poses: interpolating the two misses it by 1 m.
    rows = [_pose(0), _pose(10_000, alt=1001.0), _pose(20_000)]
    error = pr.path_error([rows[0], rows[2]], rows, [10_000, 0])
    assert error["max"] == pytest.approx(1.0) and error["n"] == 2


def test_slerp_takes_the_short_way():
    half = psl._slerp((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, -math.sqrt(0.5), -math.sqrt(0.5)), 0.5)
    assert psl._angle_deg(half, (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))) == pytest.approx(0.0, abs=1e-6)
