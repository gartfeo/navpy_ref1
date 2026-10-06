from __future__ import annotations

import io
import json
import sys
from argparse import Namespace
from collections import deque
from pathlib import Path
from unittest.mock import patch

import pytest
from pymavlink import mavutil

from scripts import eval_navigation_cases as evaluator


def _mission() -> list[evaluator.MissionItem]:
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    return [
        evaluator.MissionItem(0, waypoint, 0, 40.0, 44.0, 500.0),
        evaluator.MissionItem(1, waypoint, 3, 40.001, 44.001, 135.0),
        evaluator.MissionItem(2, 178, 2, None, None, None),
        evaluator.MissionItem(3, waypoint, 3, 40.002, 44.002, 135.0),
        evaluator.MissionItem(4, waypoint, 3, 40.003, 44.003, 135.0),
        evaluator.MissionItem(11, waypoint, 3, 40.004, 44.004, 135.0),
    ]


def _expectation(target_alt_m: float = 0.0) -> evaluator.TargetExpectation:
    return evaluator.resolve_target_expectation(
        _mission(),
        target_wp=4,
        target_rel_alt_m=target_alt_m,
        home_abs_alt_m=500.0,
    )


def _valid_evidence(target_alt_m: float = 0.0) -> evaluator.SelectionEvidence:
    location = _expectation(target_alt_m).location
    return evaluator.parse_selection_evidence(
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        f"2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        f"{location.lat_deg:.6f}, {location.lon_deg:.6f}, "
        f"{location.abs_alt_m:.1f} [{location.lat_deg:.6f}, "
        f"{location.lon_deg:.6f}, {location.abs_alt_m:.1f}], term: N/A\n"
    )


def _logged_location(evidence: evaluator.SelectionEvidence) -> evaluator.TargetLocation | None:
    return evaluator.selected_location_from_evidence(evidence, home_abs_alt_m=500.0)


def _gate(
    expectation: evaluator.TargetExpectation,
    evidence: evaluator.SelectionEvidence,
    selected_location: evaluator.TargetLocation | None,
    *,
    configured_rel_alt_m: float | None = None,
) -> evaluator.EvidenceGateResult:
    if configured_rel_alt_m is None:
        configured_rel_alt_m = expectation.location.rel_alt_m
    return evaluator.validate_pre_snap_evidence(
        expectation,
        evidence,
        selected_location,
        configured_rel_alt_m=configured_rel_alt_m,
        coordinate_tolerance_m=0.5,
        altitude_tolerance_m=0.5,
    )


def test_cli_parses_target_alts_and_matrix_controls():
    args = evaluator.parse_args(
        [
            "--target-alts",
            "0,140",
            "--target-wp",
            "5",
            "--speedups",
            "3,10",
            "--navigation-speedups",
            "1,10",
            "--winds",
            "0,8",
            "--directions",
            "90,270",
            "--max-distance",
            "0.75",
        ]
    )

    assert args.target_alts == [0, 140]
    assert args.target_wp == 5
    assert args.speedups == [3.0, 10.0]
    assert args.navigation_speedups == [1.0, 10.0]
    assert args.winds == [0.0, 8.0]
    assert args.directions == [90, 270]
    assert args.max_distance == 0.75
    assert args.python == Path(sys.executable).resolve()


def test_cli_accepts_fractional_sitl_speedups_but_rejects_nonpositive_values():
    args = evaluator.parse_args(
        ["--speedups", "0.5,3", "--navigation-speedups", "match"]
    )

    assert args.speedups == [0.5, 3.0]
    assert args.navigation_speedups is None
    with pytest.raises(SystemExit):
        evaluator.parse_args(["--speedups", "0"])


def test_cli_uses_strict_coordinate_defaults():
    args = evaluator.parse_args([])

    assert args.navigation_speedups is None
    assert args.coordinate_tolerance_m == 0.5


def test_cli_defaults_target_alts_to_ground_only():
    args = evaluator.parse_args([])

    assert args.target_alts == [0]


def test_cli_rejects_fractional_target_alt_because_navpy_target_alt_is_integer():
    with pytest.raises(SystemExit):
        evaluator.parse_args(["--target-alts", "135.5"])


def test_pinned_certification_command_still_parses_to_ground_target():
    # T-04-26: every certification command states its target explicitly, and
    # this argv shape is pinned in four unexecuted certification plans
    # (04-05..04-08) with ~295 SITL flights outstanding. Those plans must
    # spell the target as "--target-alts 0"; the old "--target-modes ground"
    # spelling was removed. Do not delete or weaken this test without
    # re-certifying those plans.
    args = evaluator.parse_args(
        [
            "--target-alts",
            "0",
            "--target-wp",
            "4",
            "--vision-profile",
            "siyi_zr10",
            "--winds",
            "0",
            "--speedups",
            "10",
            "--navigation-speedups",
            "10",
            "--max-attempts",
            "1",
            "--max-distance",
            "0.5",
        ]
    )

    assert args.target_alts == [0]
    assert args.target_wp == 4
    assert args.vision_profile == "siyi_zr10"
    assert args.winds == [0.0]
    assert args.speedups == [10]
    assert args.navigation_speedups == [10.0]
    assert args.max_attempts == 1
    assert args.max_distance == 0.5


def test_removed_target_modes_flag_is_rejected():
    with pytest.raises(SystemExit):
        evaluator.parse_args(["--target-modes", "ground"])


def test_matrix_does_not_duplicate_zero_wind_for_each_direction():
    cases = list(
        evaluator.matrix_cases(
            [0, 135], [10], None, [0.0, 8.0], [90, 270]
        )
    )

    assert len(cases) == 6
    assert len([case for case in cases if case.wind_speed_mps == 0.0]) == 2
    assert {case.navigation_speedup for case in cases} == {10.0}


def test_navigation_speedups_are_an_independent_matrix_dimension():
    cases = list(
        evaluator.matrix_cases([0], [3, 10], [1.0, 5.0], [0.0], [0])
    )

    assert [
        (case.speedup, case.navigation_speedup) for case in cases
    ] == [(3, 1.0), (3, 5.0), (10, 1.0), (10, 5.0)]


def test_target_wp_resolves_as_nav_waypoint_ordinal_not_raw_mission_seq():
    expectation = _expectation()

    assert expectation.target_wp == 4
    assert expectation.mission_seq == 11
    assert expectation.expected_task_id == 1
    assert expectation.expected_obj_id == 0
    assert expectation.location.lat_deg == 40.004
    assert expectation.location.lon_deg == 44.004
    assert expectation.location.rel_alt_m == 0.0
    assert expectation.location.abs_alt_m == 500.0


def test_valid_selection_identity_coordinate_and_altitude_pass_before_snap():
    expectation = _expectation()
    evidence = _valid_evidence()

    result = _gate(expectation, evidence, _logged_location(evidence))

    assert result.passed
    assert result.errors == ()
    assert result.coordinate_error_m == pytest.approx(0.0)
    assert result.altitude_error_m == pytest.approx(0.0)
    assert evidence.target_lat_deg == pytest.approx(expectation.location.lat_deg)
    assert evidence.target_lon_deg == pytest.approx(expectation.location.lon_deg)
    assert evidence.target_abs_alt_m == pytest.approx(expectation.location.abs_alt_m)


def test_target_coordinate_is_taken_from_first_navigation_command_after_selection():
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:39:57,700 INFO t_l (0.0m): 1.0, 2.0, 3.0 [x]\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 500.0 [40.004000, 44.004000, 500.0]\n"
        "2026-07-10 15:39:57,900 INFO cmd, t_l (0.0m): "
        "50.0, 60.0, 700.0 [50.0, 60.0, 700.0]\n"
    )

    assert evidence.target_lat_deg == 40.004
    assert evidence.target_lon_deg == 44.004
    assert evidence.target_abs_alt_m == 500.0


def test_selection_evidence_does_not_cross_into_a_later_target_episode():
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:40:00,000 INFO TARGET: T2 (tracking obj_id=1)\n"
        "2026-07-10 15:40:00,010 INFO cmd, t_l (0.0m): "
        "50.0, 60.0, 700.0 [50.0, 60.0, 700.0]\n"
    )

    assert evidence.task_id == 1
    assert evidence.obj_id == 0
    assert evidence.target_lat_deg is None
    assert evidence.target_lon_deg is None
    assert evidence.target_abs_alt_m is None


def test_target_switch_before_snap_is_rejected_from_the_accepted_episode():
    log_text = (
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 500.0 [x]\n"
        "2026-07-10 15:40:00,000 INFO TARGET: T2 (tracking obj_id=1)\n"
        "2026-07-10 15:40:01,000 INFO SNAP(VISION-NAV-PN): 3d=0.1\n"
    )
    evidence = evaluator.parse_selection_evidence(log_text)

    error = evaluator.target_episode_binding_error(log_text, evidence)

    assert error is not None
    assert "target switched before certified SNAP" in error


def test_accepted_target_episode_requires_its_later_navigation_snap():
    log_text = (
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 500.0 [x]\n"
    )
    evidence = evaluator.parse_selection_evidence(log_text)

    assert (
        evaluator.target_episode_binding_error(log_text, evidence)
        == evaluator._TARGET_SNAP_MISSING
    )


def test_single_accepted_target_episode_binds_to_later_navigation_snap(tmp_path):
    log_text = (
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 500.0 [x]\n"
        "2026-07-10 15:40:01,000 INFO SNAP(VISION-NAV-PN): 3d=0.1\n"
    )
    evidence = evaluator.parse_selection_evidence(log_text)
    navigation_path = tmp_path / "uav_121_navigation.log"
    navigation_path.write_text(log_text, encoding="utf-8")

    assert evaluator.target_episode_binding_error(log_text, evidence) is None
    assert (
        evaluator.await_target_snap_binding(
            navigation_path,
            evidence,
            timeout_s=0.0,
        )
        is None
    )


def test_selection_catalog_is_bound_to_selected_task():
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:39:20,000 INFO T2: wp:9(seq:22);\n"
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 500.0 [40.004000, 44.004000, 500.0]\n"
    )

    assert evidence.catalog_wp == 4
    assert evidence.catalog_seq == 11


def test_navpy_log_paths_are_case_local(tmp_path):
    compact, navigation = evaluator.navpy_log_paths(tmp_path / "case-a", 121)

    expected_dir = tmp_path / "case-a" / evaluator.NAVPY_LOG_SUBDIR
    assert compact == expected_dir / "uav_121_navigation_compact.csv"
    assert navigation == expected_dir / "uav_121_navigation.log"


def test_saved_eval_chat_is_used_for_teardown():
    with patch.object(evaluator.subprocess, "run") as run:
        evaluator.stop_own_stack(Path("python.exe"), chat=40)

    command = run.call_args.args[0]
    assert command[-2:] == ["--chat", "40"]
    assert "--eval" not in command


def test_prelaunch_cleanup_remains_owner_scoped_to_eval_band():
    with patch.object(evaluator.subprocess, "run") as run:
        evaluator.stop_own_stack(Path("python.exe"))

    command = run.call_args.args[0]
    assert command[-1] == "--eval"


class _FakeLauncher:
    """A swarm_run child: a pid, and an exit code once it has exited."""

    def __init__(self, pid=4242, exits_with=None):
        self.pid = pid
        self.returncode = exits_with

    def poll(self):
        return self.returncode


def _entry(pid=4242, *, verified=None, chat=40, speedup=1, error=None,
           measured_rates=None, launch_token=None):
    entry = {"chat_index": chat, "sitl_pid": pid, "sitl_verified": verified,
             "sitl_speedup": speedup, "sitl_error": error}
    if measured_rates is not None:
        entry["sitl_measured_rates"] = measured_rates
    if launch_token is not None:
        entry["sitl_launch_token"] = launch_token
    return entry


def _wait(entries, process, *, speedup=None, timeout_s=5.0, stderr_log=None,
          launch_token=None):
    """Drive wait_for_eval_chat over a scripted sequence of registry reads.

    The last read repeats, so a test that expects a wait lets the deadline or
    the launcher decide the outcome instead of running out of script. The clock
    is fake and only sleeps move it, so a watchdog test reaches its deadline
    immediately instead of busy-spinning for real seconds.
    """
    reads = list(entries)
    clock = [0.0]

    def find_for_owner(owner, label=None):
        return reads.pop(0) if len(reads) > 1 else reads[0]

    def sleep(seconds):
        clock[0] += seconds

    with patch.object(evaluator.reg, "find_for_owner", find_for_owner), \
         patch.object(evaluator.time, "time", lambda: clock[0]), \
         patch.object(evaluator.time, "sleep", sleep):
        return evaluator.wait_for_eval_chat(process, timeout_s, speedup=speedup,
                                            stderr_log=stderr_log,
                                            launch_token=launch_token)


def test_launch_wait_returns_only_once_the_verdict_is_in():
    # Entry presence used to be enough. It only means a slot was claimed.
    assert _wait([_entry(verified=None), _entry(verified=None),
                  _entry(verified=True)], _FakeLauncher()).chat == 40


def test_launch_wait_ignores_another_runs_entry():
    # A previous run's verdict on a reused slot must not read as ours; the
    # supervisor pid is what says whose launch an entry describes.
    with pytest.raises(RuntimeError, match="no verdict within"):
        _wait([_entry(pid=9999, verified=True)], _FakeLauncher(pid=4242),
              timeout_s=0.5)


def test_launch_wait_accepts_matching_token_from_venv_interpreter_child():
    verdict = _wait(
        [_entry(pid=9999, verified=True, launch_token="case-token")],
        _FakeLauncher(pid=4242),
        launch_token="case-token",
    )
    assert verdict.chat == 40


def test_launch_wait_rejects_another_runs_token():
    with pytest.raises(RuntimeError, match="no verdict within"):
        _wait(
            [_entry(pid=4242, verified=True, launch_token="other-token")],
            _FakeLauncher(pid=4242),
            launch_token="case-token",
            timeout_s=0.5,
        )


def test_launch_wait_rejects_missing_token_even_when_pid_matches():
    with pytest.raises(RuntimeError, match="no verdict within"):
        _wait(
            [_entry(pid=4242, verified=True)],
            _FakeLauncher(pid=4242),
            launch_token="case-token",
            timeout_s=0.5,
        )


def test_launch_wait_reports_the_exit_code_over_another_runs_verdict():
    # The foreign entry is not ours to read a verdict from, so the only fact
    # about OUR launch is that it is gone.
    with pytest.raises(RuntimeError, match="exited code=1"):
        _wait([_entry(pid=9999, verified=True)],
              _FakeLauncher(pid=4242, exits_with=1))


def test_launch_wait_reports_the_verification_failure():
    with pytest.raises(RuntimeError, match="SIM_SPEEDUP mismatch"):
        _wait([_entry(verified=False, error="SIM_SPEEDUP mismatch: sys_id 121=10")],
              _FakeLauncher())


def test_launch_wait_prefers_the_stored_reason_over_the_exit_code():
    # The launcher records its verdict, then retracts and exits, so both facts
    # can be visible at once. The reason is the more useful one.
    with pytest.raises(RuntimeError, match="never streamed"):
        _wait([_entry(verified=False, error="sys_ids [121] never streamed")],
              _FakeLauncher(exits_with=1))


def test_launch_wait_reports_a_launcher_that_exited_without_a_verdict():
    with pytest.raises(RuntimeError, match="exited code=1"):
        _wait([None], _FakeLauncher(exits_with=1))


def test_launch_wait_refuses_a_verified_swarm_whose_launcher_exited():
    # A verified swarm_run blocks forever holding its swarm. If it exited, the
    # swarm went with it, whatever the entry still says.
    with pytest.raises(RuntimeError, match="exited code=0"):
        _wait([_entry(verified=True)], _FakeLauncher(exits_with=0))


def test_launch_wait_rejects_a_non_eval_chat():
    with pytest.raises(RuntimeError, match="non-eval chat"):
        _wait([_entry(verified=True, chat=0)], _FakeLauncher())


def test_launch_wait_rejects_a_swarm_verified_at_another_speed():
    with pytest.raises(RuntimeError, match="asked for 10"):
        _wait([_entry(verified=True, speedup=1)], _FakeLauncher(), speedup=10)


def test_launch_wait_accepts_the_speed_the_case_asked_for():
    assert _wait([_entry(verified=True, speedup=10)], _FakeLauncher(),
                 speedup=10).chat == 40


def test_launch_wait_carries_the_measured_rate_the_launcher_published():
    # The REQUESTED speed is a setting; only the launcher's measurement says
    # what the swarm actually runs at, and a case has to be able to record it.
    verdict = _wait([_entry(verified=True, speedup=10,
                            measured_rates={"121": 10.06})],
                    _FakeLauncher(), speedup=10)
    assert verdict.measured_rates == {"121": 10.06}
    assert verdict.measured_rate == pytest.approx(10.06)


def test_launch_wait_reports_no_measured_rate_when_the_launcher_published_none():
    # An older launcher records no rates. That must read as "unmeasured", never
    # as the requested value -- inferring it back would rebuild the exact
    # silent-wrong-speed hole the measured gate exists to close.
    verdict = _wait([_entry(verified=True, speedup=10)], _FakeLauncher(),
                    speedup=10)
    assert verdict.measured_rates == {}
    assert verdict.measured_rate is None


def test_launch_wait_refuses_to_pick_a_rate_from_a_multi_vehicle_swarm():
    # Two vehicles, two clocks. Attributing one to the case would be a guess.
    verdict = _wait([_entry(verified=True, speedup=10,
                            measured_rates={"121": 10.0, "122": 9.8})],
                    _FakeLauncher(), speedup=10)
    assert verdict.measured_rate is None


def test_launch_wait_gives_up_on_a_launcher_that_never_decides():
    # Alive, owns the slot, verdict never arrives: only the watchdog ends this.
    with pytest.raises(RuntimeError, match="no verdict within"):
        _wait([_entry(verified=None)], _FakeLauncher(), timeout_s=0.5)


def test_launch_wait_is_not_bounded_by_the_heartbeat_timeout():
    # swarm_run's own worst case runs well past --heartbeat-timeout (120s), so
    # reusing that budget here would fail healthy launches.
    assert evaluator.LAUNCH_VERDICT_TIMEOUT_S > 120.0
    with patch.object(evaluator, "wait_for_eval_chat") as wait, \
         patch.object(evaluator.subprocess, "Popen"), \
         patch.object(evaluator.Path, "write_text"), \
         patch.object(evaluator.Path, "open"):
        evaluator.start_swarm(Path("python.exe"), Path("case"), 7)
    assert wait.call_args.args[1] == evaluator.LAUNCH_VERDICT_TIMEOUT_S
    assert wait.call_args.kwargs["speedup"] == 7


def test_fallback_location_selection_is_rejected_even_when_task_id_is_one():
    expectation = _expectation(135.0)
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:37:28,320 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:38:34,821 INFO SimT(2): WP21\n"
        "2026-07-10 15:38:35,831 INFO TARGET: T1 (tracking obj_id=1)\n"
        "2026-07-10 15:38:35,850 INFO cmd, t_l (0.0m): "
        "40.5, 44.5, 500.0 [40.5, 44.5, 500.0]\n"
    )

    result = _gate(expectation, evidence, None)

    assert not result.passed
    assert any("default OOI was selected" in error for error in result.errors)
    assert evidence.task_id == 1
    assert evidence.obj_id == 1


def test_wrong_task_id_is_rejected_even_when_obj_id_and_coordinate_match():
    expectation = _expectation()
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T2 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 500.0 [40.004000, 44.004000, 500.0]\n"
    )

    result = _gate(expectation, evidence, _logged_location(evidence))

    assert not result.passed
    assert any("expected T1, got T2" in error for error in result.errors)


def test_wrong_selected_target_coordinate_is_rejected():
    expectation = _expectation()
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004100, 44.004000, 500.0 [40.004100, 44.004000, 500.0]\n"
    )

    result = _gate(expectation, evidence, _logged_location(evidence))

    assert not result.passed
    assert result.coordinate_error_m > 10.0
    assert any("coordinate differs" in error for error in result.errors)


def test_wrong_selected_target_altitude_is_rejected():
    expectation = _expectation()
    evidence = evaluator.parse_selection_evidence(
        "2026-07-10 15:39:24,408 INFO T1: wp:4(seq:11);\n"
        "2026-07-10 15:39:57,765 INFO TARGET: T1 (tracking obj_id=0)\n"
        "2026-07-10 15:39:57,794 INFO cmd, t_l (0.0m): "
        "40.004000, 44.004000, 505.0 [40.004000, 44.004000, 505.0]\n"
    )

    result = _gate(expectation, evidence, _logged_location(evidence))

    assert not result.passed
    assert result.altitude_error_m == 5.0
    assert any("altitude differs" in error for error in result.errors)


def test_coordinate_scorer_uses_segment_closest_point():
    target = evaluator.TargetLocation(40.0, 44.0, 100.0, 600.0)
    scorer = evaluator.CoordinateScorer(target)
    scorer.add(evaluator.PositionSample(39.999998, 44.0, 600.0, 100.0, 1.0, 10.0))
    scorer.add(evaluator.PositionSample(40.000002, 44.0, 600.0, 100.0, 1.03, 10.03))

    assert scorer.result is not None
    assert scorer.result.dist_3d_m == pytest.approx(0.0, abs=1e-9)


def test_coordinate_scorer_never_interpolates_across_sparse_curved_path_gap():
    target = evaluator.TargetLocation(40.0, 44.0, 100.0, 600.0)
    scorer = evaluator.CoordinateScorer(target)
    ten_metres_lat = 10.0 / evaluator.EARTH_RADIUS_M * 180.0 / 3.141592653589793
    scorer.add(
        evaluator.PositionSample(40.0 - ten_metres_lat, 44.0, 600.0, 100.0, 0.0, 0.0)
    )
    scorer.add(
        evaluator.PositionSample(40.0 + ten_metres_lat, 44.0, 600.0, 100.0, 10.0, 10.0)
    )

    assert scorer.result is not None
    assert scorer.result.dist_3d_m == pytest.approx(10.0, rel=0.01)
    assert not scorer.certifiable
    assert "source-time gap" in scorer.certification_error
    assert "spatial gap" in scorer.certification_error


def test_coordinate_scorer_requires_observed_high_rate_cadence():
    target = evaluator.TargetLocation(40.0, 44.0, 100.0, 600.0)
    scorer = evaluator.CoordinateScorer(target)
    for index, north_offset_m in enumerate((-0.4, 0.0, 0.4)):
        dlat = north_offset_m / evaluator.EARTH_RADIUS_M * 180.0 / 3.141592653589793
        scorer.add(
            evaluator.PositionSample(
                40.0 + dlat,
                44.0,
                600.0,
                100.0,
                1.0 + index * 0.033,
                20.0 + index * 0.033,
            )
        )

    assert scorer.certifiable
    assert scorer.sample_count == 3
    assert scorer.max_source_gap_s == pytest.approx(0.033)


def _linear_sample(north_offset_m, source_time_s):
    """A sample `north_offset_m` north of the scorer target at a given clock."""
    dlat = north_offset_m / evaluator.EARTH_RADIUS_M * 180.0 / 3.141592653589793
    return evaluator.PositionSample(40.0 + dlat, 44.0, 600.0, 100.0, 0.0, source_time_s)


def _linear_scorer():
    return evaluator.CoordinateScorer(evaluator.TargetLocation(40.0, 44.0, 100.0, 600.0))


def test_out_of_order_arrival_is_reordered_not_treated_as_broken_telemetry():
    # Replays the field values captured live: a 20 ms backwards source step
    # carrying a 0.544 m delta, i.e. two genuine neighbours delivered swapped.
    # This used to void an entire run on a single UDP hiccup.
    scorer = _linear_scorer()
    for offset, source in ((-1.088, 325.904), (0.0, 325.944), (-0.544, 325.924)):
        scorer.add(_linear_sample(offset, source))
    for index in range(1, 6):
        scorer.add(_linear_sample(index * 0.544, 325.944 + index * 0.020))

    assert scorer.certifiable, scorer.certification_error
    assert scorer.reordered_sample_count == 1
    # The swapped frame is scored in its chronological slot, not discarded.
    assert scorer.late_sample_count == 0
    assert scorer.max_source_gap_s == pytest.approx(0.020, abs=1e-6)


def test_reordered_samples_are_scored_in_source_time_order():
    ordered = _linear_scorer()
    for index, offset in enumerate((0.0, 0.5, 1.0, 1.5)):
        ordered.add(_linear_sample(offset, 100.0 + index * 0.02))

    swapped = _linear_scorer()
    for offset, source in ((0.0, 100.0), (1.0, 100.04), (0.5, 100.02), (1.5, 100.06)):
        swapped.add(_linear_sample(offset, source))

    # Arrival order must change neither the verdict nor the geometry.
    assert swapped.certifiable
    assert swapped.sample_count == ordered.sample_count
    assert swapped.max_spatial_gap_m == pytest.approx(ordered.max_spatial_gap_m)
    assert swapped.result.dist_3d_m == pytest.approx(ordered.result.dist_3d_m)


def test_duplicate_frame_is_counted_not_fatal():
    scorer = _linear_scorer()
    for index in range(4):
        sample = _linear_sample(index * 0.5, 100.0 + index * 0.02)
        scorer.add(sample)
        if index == 1:
            scorer.add(sample)  # link fan-out re-delivers one frame verbatim

    assert scorer.certifiable, scorer.certification_error
    assert scorer.duplicate_sample_count == 1
    assert scorer.sample_count == 4


def test_same_timestamp_with_different_position_is_rejected():
    scorer = _linear_scorer()
    scorer.add(_linear_sample(0.0, 100.0))
    scorer.add(_linear_sample(0.5, 100.02))
    scorer.add(_linear_sample(9.0, 100.02))  # contradicts its own timestamp

    assert not scorer.certifiable
    assert "share a source timestamp" in scorer.certification_error


def test_source_clock_reset_fails_closed_rather_than_dropping_silently():
    # The count must not keep climbing on discarded frames, or a clock reset
    # would look like a healthy run.  This is the hole in a plain drop-stale rule.
    scorer = _linear_scorer()
    for index in range(5):
        scorer.add(_linear_sample(index * 0.5, 500.0 + index * 0.02))
    for index in range(5):
        scorer.add(_linear_sample(index * 0.5, 1.0 + index * 0.02))

    assert not scorer.certifiable
    assert "backwards beyond the reorder window" in scorer.certification_error


def test_sample_count_reports_scored_samples_not_arrivals():
    scorer = _linear_scorer()
    scorer.add(_linear_sample(0.0, 100.0))
    scorer.add(evaluator.PositionSample(40.0, 44.0, 600.0, 100.0, 0.0, None))

    assert scorer.received_sample_count == 2
    assert scorer.sample_count == 1
    assert not scorer.certifiable


def test_live_edge_anchor_uses_newest_source_time_not_last_arrival():
    class Message:
        lat = 400_000_000
        lon = 440_000_000
        alt = 600_000
        relative_alt = 100_000

        def __init__(self, time_boot_ms):
            self.time_boot_ms = time_boot_ms

        def get_type(self):
            return "GLOBAL_POSITION_INT"

        def get_srcSystem(self):
            return 121

    class Master:
        def __init__(self):
            # The newest frame arrives before an older one, as the link reorders.
            self.messages = deque((Message(1000), Message(1040), Message(1020)))

        def recv_match(self, **_kwargs):
            return self.messages.popleft() if self.messages else None

    anchors = deque(maxlen=2)
    assert evaluator._drain_position_messages(
        Master(), sysid=121, live_anchors=anchors, scorer=None
    )
    assert len(anchors) == 1
    assert anchors[0].source_time_s == pytest.approx(1.040)


def test_snap_parser_reads_compact_accuracy():
    parsed = evaluator.parse_snap_summary(
        "15:40:08.394,SNAP(VISION-NAV-PN),3d=11.9(h=11.9; v=0.8)"
    )

    assert parsed == {"dist_3d_m": 11.9, "h_m": 11.9, "v_m": 0.8}


def test_commands_are_portable_isolated_and_apply_case_target_variables():
    # A fixed fake interpreter keeps the anti-hardcoding assertion below
    # environment-independent: on worktrees whose .venv is a junction into
    # C:\repos\navpy\.venv, resolving sys.executable would itself contain the
    # banned prefix and mask whether the builders inject a hardcoded path.
    python = Path("C:/isolated/eval-venv/Scripts/python.exe")
    swarm = evaluator.build_swarm_command(python, 10)
    args = Namespace(
        network_type="none",
        detector_type="sim",
        vision_profile="ideal_360",
        pitch_controller="vision-nav-pn",
        del_throttle=-1,
        del_angle=0,
    )
    navpy = evaluator.build_navpy_command(
        python,
        sysid=121,
        nav_device="tcp:127.0.0.1:6960",
        speedup=10,
        navigation_speedup=1.0,
        target_wp=4,
        target_rel_alt_m=135.0,
        args=args,
    )

    assert swarm[0] == str(python)
    assert "--eval" in swarm
    assert swarm[swarm.index("--instances") + 1] == "1"
    assert not any("C:\\repos\\navpy\\.venv" in token for token in swarm + navpy)
    assert navpy[navpy.index("-twps") + 1] == "4,"
    assert navpy[navpy.index("-talt") + 1] == "135"
    assert navpy[navpy.index("-gsu") + 1] == "1"
    # The terminal roll-envelope raise was removed from NavPy (ef8b9a55), so
    # navpy.main rejects -trl and the companion never launches. Keep the
    # builder from re-introducing the dead argument.
    assert "-trl" not in navpy


def test_monitor_connection_uses_mission_protocol_component(monkeypatch):
    recorded = {}

    class FakeMaster:
        target_system = 0
        target_component = 0

        def wait_heartbeat(self, timeout):
            recorded["timeout"] = timeout
            return type(
                "Heartbeat",
                (),
                {
                    "get_srcSystem": lambda self: 121,
                    "get_srcComponent": lambda self: 1,
                },
            )()

        def close(self):
            recorded["closed"] = True

    def fake_connection(device, **kwargs):
        recorded["device"] = device
        recorded["kwargs"] = kwargs
        return FakeMaster()

    monkeypatch.setattr(evaluator.mavutil, "mavlink_connection", fake_connection)

    master = evaluator.wait_for_heartbeat("udp:0.0.0.0:15590", 12.0)

    assert master is not None
    assert recorded["device"] == "udp:0.0.0.0:15590"
    assert recorded["kwargs"] == {
        "source_system": 250,
        "source_component": mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    }
    assert recorded["timeout"] == 12.0
    assert master.target_system == 121
    assert master.target_component == 1


def test_mission_download_accepts_int_or_float_item_message_types():
    class Message:
        count = 1
        seq = 0
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        frame = 3
        x = 400_000_000
        y = 440_000_000
        z = 135.0

        def __init__(self, message_type):
            self._message_type = message_type

        def get_type(self):
            return self._message_type

        def get_srcSystem(self):
            return 121

    class Mav:
        def mission_request_list_send(self, *_args):
            pass

        def mission_request_int_send(self, *_args):
            pass

    class Master:
        target_system = 121
        mav = Mav()

        def __init__(self):
            self.calls = []

        def recv_match(self, *, type, blocking, timeout):
            self.calls.append(type)
            if type == "MISSION_COUNT":
                return Message("MISSION_COUNT")
            assert type == ["MISSION_ITEM_INT", "MISSION_ITEM"]
            return Message("MISSION_ITEM_INT")

    master = Master()

    mission = evaluator.download_mission(master, timeout_s=0.1, retries=1)

    assert len(mission) == 1
    assert mission[0].lat_deg == 40.0
    assert mission[0].lon_deg == 44.0
    assert ["MISSION_ITEM_INT", "MISSION_ITEM"] in master.calls


def test_home_resolution_prefers_fixed_gps_over_transient_home():
    class Message:
        def __init__(self, **fields):
            self.__dict__.update(fields)

        @staticmethod
        def get_srcSystem():
            return 121

    class Mav:
        def __init__(self):
            self.commands = []

        def command_long_send(self, *_args):
            self.commands.append(_args[2])

    class Master:
        target_system = 121
        target_component = 1

        def __init__(self):
            self.mav = Mav()

        @staticmethod
        def recv_match(*, type, **_kwargs):
            if type == "GPS_RAW_INT":
                return Message(
                    fix_type=3,
                    lat=403297412,
                    lon=444552111,
                    alt=1_294_940,
                )
            if type == "HOME_POSITION":
                return Message(
                    latitude=0,
                    longitude=0,
                    altitude=6,
                )
            return None

    master = Master()

    home_alt_m = evaluator.resolve_home_abs_alt_m(master, timeout_s=0.01)

    assert home_alt_m == pytest.approx(1294.94)
    assert master.mav.commands == []


def test_home_resolution_waits_past_gps_without_a_3d_fix():
    class Gps:
        def __init__(self, *, fix_type, lat, lon, alt):
            self.fix_type = fix_type
            self.lat = lat
            self.lon = lon
            self.alt = alt

        @staticmethod
        def get_srcSystem():
            return 121

    class Master:
        target_system = 121
        target_component = 1

        def __init__(self):
            self.samples = iter((
                Gps(fix_type=1, lat=0, lon=0, alt=-2),
                Gps(
                    fix_type=3,
                    lat=403297412,
                    lon=444552111,
                    alt=1_294_900,
                ),
            ))

        def recv_match(self, *, type, **_kwargs):
            if type == "GPS_RAW_INT":
                return next(self.samples)
            return None

    home_alt_m = evaluator.resolve_home_abs_alt_m(
        Master(),
        timeout_s=0.1,
    )

    assert home_alt_m == pytest.approx(1294.9)


def test_home_resolution_rejects_transient_home_when_fixed_gps_is_unavailable():
    class Home:
        latitude = 403297412
        longitude = 444552111
        altitude = 6

        @staticmethod
        def get_srcSystem():
            return 121

    class Mav:
        def __init__(self):
            self.commands = []

        def command_long_send(self, *_args):
            self.commands.append(_args[2])

    class Master:
        target_system = 121
        target_component = 1

        def __init__(self):
            self.mav = Mav()

        @staticmethod
        def recv_match(*, type, **_kwargs):
            return Home() if type == "HOME_POSITION" else None

    master = Master()

    with pytest.raises(
        RuntimeError,
        match="no fixed GPS_RAW_INT before timeout",
    ):
        evaluator.resolve_home_abs_alt_m(master, timeout_s=0.01)

    assert master.mav.commands == []


def test_coordinate_scorer_requests_high_rate_position_on_its_link():
    recorded = {}

    class Mav:
        def command_long_send(self, *args):
            recorded["args"] = args

    class Ack:
        command = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED

        def get_srcSystem(self):
            return 121

    class Master:
        target_system = 121
        target_component = 1
        mav = Mav()

        def recv_match(self, **_kwargs):
            return Ack()

    master = Master()

    assert evaluator.request_coordinate_score_stream(master)

    args = recorded["args"]
    assert args[0:4] == (
        121,
        1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
    )
    assert args[4] == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
    assert args[5] == round(1_000_000 / evaluator.COORDINATE_SCORE_RATE_HZ)


def test_coordinate_stream_request_rejects_nonaccepted_ack():
    class Mav:
        def command_long_send(self, *_args):
            pass

    class Ack:
        command = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        result = mavutil.mavlink.MAV_RESULT_DENIED

        def get_srcSystem(self):
            return 121

    class Master:
        target_system = 121
        target_component = 1
        mav = Mav()

        def recv_match(self, **_kwargs):
            return Ack()

    assert not evaluator.request_coordinate_score_stream(Master(), timeout_s=0.01)


def test_coordinate_stream_request_fails_when_ack_times_out():
    class Mav:
        def command_long_send(self, *_args):
            pass

    class Master:
        target_system = 121
        target_component = 1
        mav = Mav()

        def recv_match(self, **_kwargs):
            return None

    assert not evaluator.request_coordinate_score_stream(Master(), timeout_s=0.001)


def test_backlogged_position_drain_cannot_create_live_source_anchor():
    class Message:
        lat = 400_000_000
        lon = 440_000_000
        alt = 600_000
        relative_alt = 100_000

        def __init__(self, time_boot_ms):
            self.time_boot_ms = time_boot_ms

        def get_type(self):
            return "GLOBAL_POSITION_INT"

        def get_srcSystem(self):
            return 121

    class Master:
        def __init__(self):
            self.messages = deque(Message(index * 33) for index in range(500))

        def recv_match(self, **_kwargs):
            return self.messages.popleft() if self.messages else None

    class CountingScorer:
        def __init__(self):
            self.samples = []

        def add(self, sample):
            self.samples.append(sample)

    scorer = CountingScorer()
    anchors = deque(
        [
            evaluator.PositionStreamAnchor(1.0, 1.0),
            evaluator.PositionStreamAnchor(2.0, 2.0),
        ],
        maxlen=2,
    )

    caught_up = evaluator._drain_position_messages(
        Master(),
        sysid=121,
        live_anchors=anchors,
        scorer=scorer,
    )

    assert not caught_up
    assert len(scorer.samples) == 500
    assert list(anchors) == []


def test_empty_reached_position_drain_anchors_only_its_newest_source_row():
    class Message:
        lat = 400_000_000
        lon = 440_000_000
        alt = 600_000
        relative_alt = 100_000

        def __init__(self, time_boot_ms):
            self.time_boot_ms = time_boot_ms

        def get_type(self):
            return "GLOBAL_POSITION_INT"

        def get_srcSystem(self):
            return 121

    class Master:
        def __init__(self):
            self.messages = deque((Message(1000), Message(1033)))

        def recv_match(self, **_kwargs):
            return self.messages.popleft() if self.messages else None

    anchors = deque(maxlen=2)

    caught_up = evaluator._drain_position_messages(
        Master(),
        sysid=121,
        live_anchors=anchors,
        scorer=None,
    )

    assert caught_up
    assert len(anchors) == 1
    assert anchors[0].source_time_s == pytest.approx(1.033)


def test_position_stream_must_remain_at_live_edge_through_snap():
    fresh = [evaluator.PositionStreamAnchor(10.0, 100.0)]

    assert evaluator._position_stream_is_live(fresh, 100.1)
    assert not evaluator._position_stream_is_live(fresh, 100.5)
    assert not evaluator._position_stream_is_live([], 100.1)


def test_stop_own_stack_targets_only_this_worktree_eval_slot(monkeypatch):
    recorded = {}

    def fake_run(command, **kwargs):
        recorded["command"] = command
        recorded["kwargs"] = kwargs

    monkeypatch.setattr(evaluator.subprocess, "run", fake_run)
    evaluator.stop_own_stack(Path(sys.executable).resolve())

    assert recorded["command"][-1] == "--eval"
    assert "--chat" not in recorded["command"]
    assert "--all" not in recorded["command"]
    assert Path(recorded["kwargs"]["cwd"]) == evaluator.WORKTREE


FAILED_LINE = ("SITL chat 40: FAILED to bring up a verified swarm after 3 "
               "attempts: sys_ids [121] never streamed")


def _err_log(tmp_path, text):
    log = tmp_path / "swarm.err.log"
    log.write_text(text, encoding="utf-8")
    return log


def test_a_released_verdict_is_recovered_from_the_launcher_log(tmp_path):
    # The reason the registry no longer has. A terminal failure records its
    # verdict, tears down, then retracts — and an eval slot owns nothing else,
    # so that retraction RELEASES the entry and the reason goes with it. An
    # evaluator that was busy across that window sees only a dead child.
    with pytest.raises(RuntimeError, match="never streamed"):
        _wait([None], _FakeLauncher(exits_with=1),
              stderr_log=_err_log(tmp_path, FAILED_LINE))


def test_the_exit_code_is_still_reported_alongside_the_log(tmp_path):
    with pytest.raises(RuntimeError, match="exited code=1"):
        _wait([None], _FakeLauncher(exits_with=1),
              stderr_log=_err_log(tmp_path, FAILED_LINE))


def test_the_watchdog_also_quotes_the_launcher_log(tmp_path):
    # A launcher that hangs without publishing anything: whatever it managed to
    # say is the only lead there is.
    with pytest.raises(RuntimeError, match="never streamed"):
        _wait([_entry(verified=None)], _FakeLauncher(), timeout_s=0.5,
              stderr_log=_err_log(tmp_path, FAILED_LINE))


def test_a_stored_verdict_is_still_preferred_over_the_log(tmp_path):
    # The registry verdict is the authority; the log is only the fallback. If
    # both are present the verdict decides, and the log must not be consulted at
    # all — asserting only that the stored reason appears would still pass if the
    # log were appended to it.
    def _must_not_be_called(_):
        raise AssertionError("the log was consulted on the authoritative path")

    with patch.object(evaluator, "launcher_failure_hint", _must_not_be_called):
        with pytest.raises(RuntimeError) as excinfo:
            _wait([_entry(verified=False, error="stored reason")], _FakeLauncher(),
                  stderr_log=_err_log(tmp_path, "log reason"))
    assert "failed verification: stored reason" in str(excinfo.value)
    assert "log reason" not in str(excinfo.value)


def test_a_missing_log_degrades_to_the_bare_exit_code(tmp_path):
    # Captured launcher output is known to lose bytes on Windows, so an absent
    # or unreadable log must cost a better message and nothing else.
    with pytest.raises(RuntimeError, match=r"exited code=1$"):
        _wait([None], _FakeLauncher(exits_with=1),
              stderr_log=tmp_path / "does-not-exist.log")


def test_an_empty_log_degrades_to_the_bare_exit_code(tmp_path):
    with pytest.raises(RuntimeError, match=r"exited code=1$"):
        _wait([None], _FakeLauncher(exits_with=1),
              stderr_log=_err_log(tmp_path, "   \n  "))


def test_no_log_path_at_all_is_not_an_error(tmp_path):
    # wait_for_eval_chat is callable without a log; it just loses the hint.
    with pytest.raises(RuntimeError, match=r"exited code=1$"):
        _wait([None], _FakeLauncher(exits_with=1))


def test_a_long_log_is_tailed_not_dumped(tmp_path):
    # A whole SITL boot log in one exception is unreadable; the failure is at
    # the end, so quote the end.
    noise = "x" * 5000
    with pytest.raises(RuntimeError) as excinfo:
        _wait([None], _FakeLauncher(exits_with=1),
              stderr_log=_err_log(tmp_path, noise + "\n" + FAILED_LINE))
    message = str(excinfo.value)
    assert "never streamed" in message
    assert len(message) < 1500
    assert "..." in message


class _CountingLog:
    """A log path that records how much was actually READ off the disk."""

    def __init__(self, data: bytes):
        self._data = data
        self.reads: list[int] = []

    def open(self, mode="rb"):
        outer = self

        class _Handle(io.BytesIO):
            def read(self, size=-1):
                outer.reads.append(size)
                return super().read(size)

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        return _Handle(self._data)

    def __str__(self):
        return "swarm.err.log"


def test_only_a_bounded_tail_is_read_off_disk():
    # A short MESSAGE is not the same as a bounded READ. The watchdog can fire
    # while a wedged child is still writing, so slurping the whole log to quote
    # its last 800 bytes could exhaust memory — the diagnostic would then be
    # worse than the bare message it set out to improve.
    log = _CountingLog(b"x" * 5_000_000 + FAILED_LINE.encode())
    with pytest.raises(RuntimeError, match="never streamed"):
        _wait([None], _FakeLauncher(exits_with=1), stderr_log=log)
    assert log.reads, "nothing was read"
    assert all(0 < size <= evaluator.LAUNCHER_LOG_TAIL_BYTES for size in log.reads), \
        f"unbounded read: {log.reads}"


def test_start_swarm_hands_the_wait_its_stderr_log(tmp_path):
    # The path has to actually reach the waiter, and be the file the child's
    # stderr was redirected into.
    with patch.object(evaluator, "wait_for_eval_chat") as wait, \
         patch.object(evaluator.subprocess, "Popen"), \
         patch.object(evaluator.Path, "write_text"):
        evaluator.start_swarm(Path("python.exe"), tmp_path, 7)
    assert wait.call_args.kwargs["stderr_log"] == tmp_path / "swarm.err.log"


def test_start_swarm_correlates_the_venv_child_with_a_launch_token(tmp_path):
    with patch.object(evaluator.secrets, "token_hex", return_value="case-token"), \
         patch.object(evaluator, "wait_for_eval_chat") as wait, \
         patch.object(evaluator.subprocess, "Popen") as popen, \
         patch.object(evaluator.Path, "write_text"):
        evaluator.start_swarm(Path("python.exe"), tmp_path, 7)

    command = popen.call_args.args[0]
    assert command[command.index("--launch-token") + 1] == "case-token"
    assert wait.call_args.kwargs["launch_token"] == "case-token"


def test_run_case_uses_facade_process_lifecycle_seams(tmp_path):
    args = evaluator.parse_args(["--python", sys.executable, "--timeout", "0.1"])
    case = evaluator.MatrixCase(0, 1, 1.0, 0.0, 0)
    with patch.object(evaluator, "stop_own_stack") as stop_stack, \
         patch.object(evaluator, "start_swarm", side_effect=RuntimeError("seam")) \
         as start_swarm, \
         patch.object(evaluator, "_terminate_child") as terminate_child:
        row = evaluator.run_case(0, case, 1, tmp_path, args)

    start_swarm.assert_called_once()
    assert stop_stack.call_count == 2
    assert terminate_child.call_count == 2
    assert row["error"] == "seam"


# --------------------------------------------------------------------------
# Certificate mode (--repetitions): repeatability without selection bias.
# --------------------------------------------------------------------------


def _cli(*extra: str) -> Namespace:
    return evaluator.parse_args(["--python", sys.executable, *extra])


def test_default_run_keeps_two_attempts_and_is_not_certificate_mode():
    args = _cli()
    assert args.repetitions is None
    assert args.max_attempts == evaluator.DEFAULT_MAX_ATTEMPTS


def test_repetitions_forces_one_attempt_per_run():
    # A retry keeps flying until a run passes and reports that one. In a
    # repeatability certificate that is selection bias, so certificate mode
    # allows exactly one attempt and every run stands.
    args = _cli("--repetitions", "10")
    assert args.max_attempts == 1


def test_repetitions_refuses_an_explicit_retry_request_rather_than_ignoring_it():
    with pytest.raises(SystemExit):
        _cli("--repetitions", "10", "--max-attempts", "3")


def test_repetitions_allows_the_redundant_but_honest_max_attempts_one():
    assert _cli("--repetitions", "10", "--max-attempts", "1").repetitions == 10


def test_repetitions_must_be_at_least_one():
    with pytest.raises(SystemExit):
        _cli("--repetitions", "0")


def test_certificate_mode_pins_the_canonical_speed_and_zero_wind():
    # The certificate is defined AT a configuration. Running it at another speed
    # or in wind produces a number that cannot be compared to the pinned one.
    with pytest.raises(SystemExit):
        _cli("--repetitions", "10", "--speedups", "3")
    with pytest.raises(SystemExit):
        _cli("--repetitions", "10", "--winds", "8")
    assert _cli("--repetitions", "10", "--speedups",
                str(evaluator.cert.CERTIFICATE_SPEEDUP)).repetitions == 10


def test_certificate_speed_is_the_launcher_default_not_a_second_opinion():
    assert evaluator.cert.CERTIFICATE_SPEEDUP == 10


def test_the_evaluator_no_longer_writes_sim_speedup_after_launch():
    """swarm_run owns sim speed and PROVES it by measurement.

    A post-launch write from here could move the swarm off the rate the launch
    gate just certified, and nothing re-measures afterwards -- so the run would
    silently be at a different speed than its own launch record claims.
    """
    scripts_dir = Path(evaluator.__file__).parent
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in scripts_dir.glob("eval_navigation_*.py")
    )
    assert 'set_param(master, "SIM_SPEEDUP"' not in source
    # It is still READ back, which is the check that has evidence behind it.
    assert "SIM_SPEEDUP" in evaluator.cert.CERTIFICATE_READBACK_PARAMS


def test_certificate_pins_wind_and_turbulence_but_not_sensor_noise():
    names = [name for name, _ in evaluator.cert.CERTIFICATE_WIND_PARAMS]
    assert names == ["SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_TURB"]
    assert all(value == 0.0 for _, value in evaluator.cert.CERTIFICATE_WIND_PARAMS)
    # Sensor noise stays ON: the benchmark this certificate is compared against
    # ran with it, so disabling it would make the two numbers incomparable.
    assert not any("NOISE" in name for name in evaluator.cert.CERTIFICATE_READBACK_PARAMS)


def test_identity_is_only_gathered_in_certificate_mode():
    assert evaluator.certificate_identity(
        object(), mission=[], sysid=121, enabled=False) is None


class _ParamValue:
    def __init__(self, name, value, index, count):
        self.param_id = name
        self.param_value = value
        self.param_index = index
        self.param_count = count

    def get_srcSystem(self):
        return 121


class _ParamMaster:
    """A vehicle that answers PARAM_REQUEST_LIST, then PARAM_REQUEST_READ.

    *drop* names indices the initial list will not deliver; *answer_retry*
    decides whether the by-index re-request is honoured, so one fake covers both
    "recovered on retry" and "still missing".
    """

    target_system = 121
    target_component = 1

    def __init__(self, count=4, drop=(), answer_retry=True):
        self._count = count
        self._answer_retry = answer_retry
        self.requested_indices = []
        self.messages = [
            _ParamValue(f"P{i}", float(i), i, count)
            for i in range(count) if i not in set(drop)
        ]
        outer = self

        class Mav:
            def param_request_list_send(self, *_args):
                return None

            def param_request_read_send(self, _sys, _comp, _name, index):
                outer.requested_indices.append(index)
                if outer._answer_retry:
                    outer.messages.append(
                        _ParamValue(f"P{index}", float(index), index, outer._count))

        self.mav = Mav()

    def recv_match(self, **_kwargs):
        return self.messages.pop(0) if self.messages else None


def test_full_parameter_download_stops_on_the_vehicles_own_count():
    master = _ParamMaster(count=2)
    master.messages.append(_ParamValue("C", 3.0, 2, 2))  # one row too many
    snapshot = evaluator.download_all_params(master)
    assert snapshot.values == {"P0": 0.0, "P1": 1.0}
    assert snapshot.complete
    # The extra row was never consumed: the download ended at param_count.
    assert len(master.messages) == 1


def test_a_dropped_param_is_re_requested_by_index_and_recovered():
    # UDP drops PARAM_VALUEs routinely. A quiet link is not proof the download
    # finished, so the gap is re-asked for rather than assumed away.
    master = _ParamMaster(count=4, drop=(2,))
    snapshot = evaluator.download_all_params(master, idle_timeout_s=0.0)
    assert master.requested_indices == [2]
    assert snapshot.complete
    assert snapshot.received_count == 4
    assert snapshot.retried_indices == 1
    assert snapshot.missing_indices == ()


def test_a_still_missing_param_is_recorded_as_incomplete_not_hashed_silently():
    """The defect this exists for: a partial snapshot pinned as the whole one.

    An idle-timeout exit looks exactly like a completed download, so a snapshot
    missing a dropped SIM_* would hash to a stable, plausible, WRONG value and
    the certificate would pin it as "the configuration".
    """
    master = _ParamMaster(count=4, drop=(1, 3), answer_retry=False)
    snapshot = evaluator.download_all_params(
        master, idle_timeout_s=0.0, retry_timeout_s=0.0)
    assert not snapshot.complete
    assert snapshot.missing_indices == (1, 3)
    assert snapshot.received_count == 2
    identity = evaluator.cert.parameter_snapshot_identity(snapshot)
    assert identity["complete"] is False
    assert identity["vehicle_param_count"] == 4
    assert identity["parameter_count"] == 2
    assert identity["missing_indices"] == [1, 3]


def test_an_incomplete_snapshot_cannot_hash_like_a_complete_one():
    partial = evaluator.cert.ParameterSnapshot(
        values={"P0": 0.0, "P2": 2.0}, vehicle_param_count=4,
        missing_indices=(1, 3))
    pretend_complete = evaluator.cert.ParameterSnapshot(
        values={"P0": 0.0, "P2": 2.0}, vehicle_param_count=2)
    assert pretend_complete.complete and not partial.complete
    assert (evaluator.cert.parameter_snapshot_identity(partial)["sha256"]
            != evaluator.cert.parameter_snapshot_identity(pretend_complete)["sha256"])


def test_a_vehicle_that_never_states_a_count_is_unknown_not_complete():
    # No param_count means nothing to check against. "We do not know" must not
    # render as "we have everything".
    master = _ParamMaster(count=2)
    master.messages = [_ParamValue("A", 1.0, 0, 0), _ParamValue("B", 2.0, 1, 0)]
    snapshot = evaluator.download_all_params(master, idle_timeout_s=0.0)
    assert snapshot.vehicle_param_count is None
    assert not snapshot.complete


class _PositionMessage:
    lat = 400_000_000
    lon = 440_000_000
    alt = 500_000
    relative_alt = 100_000

    def __init__(self, time_boot_ms):
        self.time_boot_ms = time_boot_ms

    def get_type(self):
        return "GLOBAL_POSITION_INT"

    def get_srcSystem(self):
        return 121


class _PositionMaster:
    def __init__(self, *boot_ms):
        self.messages = deque(_PositionMessage(ms) for ms in boot_ms)

    def recv_match(self, **_kwargs):
        return self.messages.popleft() if self.messages else None


def test_the_rate_tracker_is_fed_the_same_samples_as_the_scorer():
    tracker = evaluator.cert.ClockRateTracker(min_span_s=0.0)
    evaluator._drain_position_messages(
        _PositionMaster(1000, 2000), sysid=121,
        live_anchors=deque(maxlen=2), scorer=None, rate_tracker=tracker,
    )
    assert tracker.result.sample_count == 2


def test_the_in_run_rate_is_stamped_on_the_monotonic_clock():
    """A wall-clock STEP mid-run must not move the measured rate.

    The rate is a source span over a wall span, so an NTP correction landing in
    the divisor would silently rescale it, and nothing in the artifact would
    show it happened. time.monotonic() cannot step.
    """
    monotonic = iter([10.0, 11.0, 12.0, 13.0])
    # Wall time jumps an hour backwards between the two samples, as an NTP
    # correction would. It must not reach the tracker at all.
    wall = iter([1000.0, 1001.0, -2600.0, -2599.0, -2598.0, -2597.0])
    tracker = evaluator.cert.ClockRateTracker(min_span_s=0.0)
    with patch.object(evaluator.time, "monotonic", lambda: next(monotonic)), \
         patch.object(evaluator.time, "time", lambda: next(wall)):
        evaluator._drain_position_messages(
            _PositionMaster(1000, 11000), sysid=121,
            live_anchors=deque(maxlen=2), scorer=None, rate_tracker=tracker,
        )
    result = tracker.result
    # 10 sim seconds over 1 monotonic second. Had the wall stamps been used the
    # span would have been negative and the rate unmeasurable.
    assert result.rate == pytest.approx(10.0)
    assert result.error is None


def _certificate_main(tmp_path, monkeypatch, *, repetitions, failing_repetition):
    """Drive main()'s certificate branch with run_case stubbed.

    Everything external is replaced: no SITL, no NavPy, no MAVLink. What is
    exercised is the loop itself -- how many times it calls run_case, which rows
    it keeps, and what it writes -- which is the part no other test covers and
    the part the no-selection guarantee lives in.
    """
    calls = []

    def fake_run_case(index, case, attempt, run_dir, args):
        calls.append((index, attempt))
        passed = attempt != failing_repetition
        return {
            "index": index,
            "attempt": attempt,
            "repetition": attempt,
            "certificate_mode": True,
            "name": f"run-{index}-{attempt}",
            "passed": passed,
            "identity_gate_passed": passed,
            "certificate_invalid_reason": "",
            "error": "" if passed else "no SNAP found before timeout",
            "dist_3d_m": 0.20 + 0.05 * attempt if passed else 3.5,
            "coordinate_dist_3d_m": 0.22 + 0.05 * attempt if passed else 3.6,
            "measured_clock_rate": 10.0 + 0.01 * attempt,
            "speedup": evaluator.cert.CERTIFICATE_SPEEDUP,
            "identity_navpy_commit": "abc123",
            "identity_mission_sha": "mission-sha",
            "identity_parameters_sha": "parameters-sha",
            "identity_autopilot": "firmware-build",
        }

    monkeypatch.setattr(evaluator, "run_case", fake_run_case)
    monkeypatch.setattr(evaluator, "WORKTREE", tmp_path)
    code = evaluator.main([
        "--python", sys.executable,
        "--repetitions", str(repetitions),
        "--speedups", str(evaluator.cert.CERTIFICATE_SPEEDUP),
        "--winds", "0",
    ])
    run_dirs = list((tmp_path / ".sitl-runs").iterdir())
    assert len(run_dirs) == 1
    return code, calls, run_dirs[0]


def test_certificate_mode_flies_every_repetition_even_after_a_failure(
        tmp_path, monkeypatch):
    # No early exit. The ordinary matrix loop stops at the first PASS; a
    # certificate that did the same would report however many runs it took to
    # get lucky, under the name of N.
    _code, calls, _run_dir = _certificate_main(
        tmp_path, monkeypatch, repetitions=3, failing_repetition=2)
    assert calls == [(0, 1), (0, 2), (0, 3)]


def test_a_failed_repetition_stays_in_the_result_set(tmp_path, monkeypatch):
    # No substitution: the failed run is final.csv AND in the aggregate, and the
    # command exits non-zero because of it.
    code, _calls, run_dir = _certificate_main(
        tmp_path, monkeypatch, repetitions=3, failing_repetition=2)
    assert code == 1
    final = (run_dir / "final.csv").read_text(encoding="utf-8").splitlines()
    assert len(final) == 4  # header + 3 runs
    assert sum("no SNAP found before timeout" in line for line in final) == 1
    summary = json.loads(
        (run_dir / "certificate-0.json").read_text(encoding="utf-8"))["summary"]
    assert summary["runs"] == 3
    assert summary["passed_runs"] == 2
    assert summary["failed_runs"] == 1
    assert [run["repetition"] for run in summary["per_run"]] == [1, 2, 3]
    # The failed run's miss is IN the spread, which is the whole point.
    assert summary["metrics"]["snap_dist_3d_m"]["count"] == 3
    assert summary["metrics"]["snap_dist_3d_m"]["max"] == pytest.approx(3.5)


def test_certificate_writes_one_summary_artifact_per_case(tmp_path, monkeypatch):
    _code, _calls, run_dir = _certificate_main(
        tmp_path, monkeypatch, repetitions=2, failing_repetition=None)
    assert (run_dir / "certificate-0.json").is_file()
    assert (run_dir / "attempts.jsonl").is_file()
    payload = json.loads((run_dir / "certificate-0.json").read_text(encoding="utf-8"))
    assert payload["case"]["speedup"] == evaluator.cert.CERTIFICATE_SPEEDUP
    assert payload["summary"]["runs"] == 2


def test_an_all_passing_certificate_exits_zero(tmp_path, monkeypatch):
    code, calls, _run_dir = _certificate_main(
        tmp_path, monkeypatch, repetitions=4, failing_repetition=None)
    assert code == 0
    assert len(calls) == 4


def test_the_ordinary_matrix_loop_still_stops_at_the_first_pass(
        tmp_path, monkeypatch):
    # The retry path is unchanged for non-certificate runs: attempt 1 passes, so
    # attempt 2 never happens.
    calls = []

    def fake_run_case(index, case, attempt, run_dir, args):
        calls.append((index, attempt))
        return {"index": index, "attempt": attempt, "passed": True,
                "identity_gate_passed": True, "error": "", "name": "n"}

    monkeypatch.setattr(evaluator, "run_case", fake_run_case)
    monkeypatch.setattr(evaluator, "WORKTREE", tmp_path)
    code = evaluator.main(["--python", sys.executable])
    assert code == 0
    assert calls == [(0, 1)]
