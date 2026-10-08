from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import eval_gcs_navigation_demo as evaluator
from scripts import eval_gcs_demo_process as demo_process
from scripts import eval_gcs_demo_runtime as demo_runtime
from scripts import eval_gcs_demo_stack_ops as stack_ops


def _clock(index: int, step_s: float) -> str:
    total = 12 * 3600 + index * step_s
    hour = int(total // 3600) % 24
    minute = int(total % 3600 // 60)
    second = total % 60
    return f"{hour:02d}:{minute:02d}:{second:06.3f}"


def _write_episode(
    log_dir: Path,
    sys_id: int,
    rolls: list[float],
    *,
    snap_m: float,
    role: str = "owner",
    wall_step_s: float = 0.01,
    source_step_s: float = 0.1,
) -> None:
    poi_coordinate = {
        4: (40.1000, 44.1000),
        5: (40.0005, 44.0000),
        6: (40.0006, 44.0000),
    }.get(sys_id, (40.1000, 44.1000))
    truth_rows = "\n".join(
        "2026-07-10 12:00:02,000 INFO cmd, t_l (0.0m): "
        f"{poi_coordinate[0]:.6f}, {poi_coordinate[1]:.6f}, 100.0 [truth]"
        for _roll in rolls
    )
    if role == "owner":
        poi_setup = (
            "P1: wp:3(seq:10); P2: wp:4(seq:11); P3: wp:7(seq:14);\n"
            "Rebroadcast task 2, waiting for 2 peers\n"
            "Rebroadcast task 3, waiting for 2 peers\n"
            "GUIDED_LOITER cmd=DO_REPOSITION center=40.1000,44.1000,115 "
            "radius=500m\n"
            "Self-detect orbit: poi=40.1000,44.1000 orbit_r=500m"
        )
    else:
        poi_setup = (
            "No POI is set\nGUIDED_LOITER cmd=DO_REPOSITION "
            f"center=40.{sys_id:04d},44,100 radius=500m"
        )
    navigation = (
        f"2026-07-10 11:59:59,900 INFO Heartbeat from system {sys_id} received.\n"
        "2026-07-10 12:00:00,000 INFO Args (non-default): "
        "vision_profile='siyi_zr10'\n"
        "2026-07-10 12:00:00,004 INFO Using vision profile 'siyi_zr10' "
        "from vision_profiles.json\n"
        "2026-07-10 12:00:00,006 INFO Mount 'siyi_zr10': 2560x1440, "
        "pitch=-14.0, fixed=False\n"
        "2026-07-10 12:00:00,010 INFO AAS params (non-default): "
        "AAS_DEL_CTRL=2.0, AAS_NAV_AUTO_CM=0.0\n"
        f"2026-07-10 12:00:00,020 INFO {poi_setup}\n"
        "2026-07-10 12:00:00,030 INFO SIYI ZR10 simulator started "
        "(sim_speed=10.0)\n"
        "2026-07-10 12:00:00,500 INFO GimbalNavigation(siyi_zr10): "
        "start_tracking obj_id=0 (prev=None, mode=LOCK)\n"
        "2026-07-10 12:00:00,510 INFO POI: P1 (tracking obj_id=11)\n"
        "2026-07-10 12:00:01,000 INFO CONFIRMING: P1\n"
        "2026-07-10 12:00:01,010 INFO Sending confirm request for P1.\n"
        "2026-07-10 12:00:01,020 INFO POI 1 confirmed by ground station.\n"
        "2026-07-10 12:00:01,030 INFO INIT: NAV MODE\n"
        f"{truth_rows}\n"
        "2026-07-10 12:00:02,990 INFO RESET: PASSED POI\n"
        f"2026-07-10 12:00:03,000 INFO SNAP(VISION-NAV-PN): 3d={snap_m:.1f}\n"
    )
    (log_dir / f"uav_{sys_id}_navigation.log").write_text(navigation, encoding="utf-8")

    compact_rows = [
        "ts,dist,h_dist,v_dist,cmd_r,cmd_p,yaw_err,pitch_err,act_r,act_p,x_err,y_err"
    ]
    for index, roll in enumerate(rolls):
        compact_rows.append(
            f"{_clock(index, wall_step_s)},{100-index:.1f},99,10,{roll:.2f},"
            "-10,0,0,0,0,0,0"
        )
    compact_rows.append(
        f"{_clock(len(rolls), wall_step_s)},SNAP(VISION-NAV-PN),"
        f"3d={snap_m:.1f}(h=0.1; v={snap_m:.1f}),,,,,,,,,"
    )
    (log_dir / f"uav_{sys_id}_navigation_compact.csv").write_text(
        "\n".join(compact_rows) + "\n", encoding="utf-8"
    )

    debug_rows = ["ts,event,payload,,,,,,,,,"]
    for index, roll in enumerate(rolls):
        dt = "" if index == 0 else f"{wall_step_s * 1000:.0f}"
        timestamp = _clock(index, wall_step_s)
        debug_rows.append(
            f"{timestamp},EVENT:FINAL_APPROACH_CMD,source=camera;generation=4;"
            f"task=1;obj=11;obs_ts={1000 + index * source_step_s:.3f};"
            f"dt_wall_ms={dt};body_bearing_deg=2.5;cmd_roll={roll:.2f};"
            "cmd_pitch=-10;cmd_thr=0.55;issued=True;passed=False,,,,,,,,,"
        )
    pass_index = len(rolls)
    debug_rows.append(
        f"{_clock(pass_index, wall_step_s)},EVENT:FINAL_APPROACH_CMD,"
        "source=camera;generation=4;task=1;obj=11;"
        f"obs_ts={1000 + pass_index * source_step_s:.3f};"
        f"dt_wall_ms={wall_step_s * 1000:.0f};body_bearing_deg=2.5;"
        "cmd_roll=;cmd_pitch=;cmd_thr=;issued=False;passed=True,,,,,,,,,"
    )
    debug_rows.append(
        f"{_clock(len(rolls), wall_step_s)},EVENT:SNAP_COMPONENTS,"
        f"algorithm=VISION-NAV-PN;dist_3d_m={snap_m:.6f};"
        f"h_m=0.1;v_m={snap_m:.6f},,,,,,,,,"
    )
    (log_dir / f"uav_{sys_id}_navigation_debug.csv").write_text(
        "\n".join(debug_rows) + "\n", encoding="utf-8"
    )
    (log_dir / f"uav_{sys_id}_confirmation_t1_120001_meta.json").write_text(
        json.dumps(
            {
                "bbox_cxcywh": [320.0, 240.0, 50.0, 10.0],
                "class_id": 0,
                "confirmation_degraded": False,
            }
        ),
        encoding="utf-8",
    )


def _write_resolved_plan(log_dir: Path) -> None:
    (log_dir / "demo_mission_plan.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "manifest_schema_version": 1,
                "owner_slot": "owner",
                "vehicles": [
                    {"slot": "owner", "role": "owner", "sys_id": 4},
                    {"slot": "peer-1", "role": "peer", "sys_id": 5},
                    {"slot": "peer-2", "role": "peer", "sys_id": 6},
                ],
                "pois": [
                    {"task_id": 1, "nav_waypoint_ordinal": 3, "lat": 40.1000, "lon": 44.1000},
                    {"task_id": 2, "nav_waypoint_ordinal": 4, "lat": 40.0005, "lon": 44.0},
                    {"task_id": 3, "nav_waypoint_ordinal": 7, "lat": 40.0006, "lon": 44.0},
                ]
            }
        ),
        encoding="utf-8",
    )
    (log_dir / "demo_command_bounds.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "vehicles": [
                    {
                        "sys_id": sys_id,
                        "pitch_min_deg": -40.0,
                        "pitch_max_deg": 20.0,
                        "roll_limit_deg": 45.0,
                        "throttle": 0.55,
                    }
                    for sys_id in (4, 5, 6)
                ],
            }
        ),
        encoding="utf-8",
    )
    (log_dir / "gcs_backend.log").write_text(
        "\n".join((
            "INFO Task assign request: sender=4 receiver=5 task_id=2 "
            "at (40.000500, 44.000000)",
            "INFO Task assign request: sender=4 receiver=6 task_id=3 "
            "at (40.000600, 44.000000)",
            "INFO Task assign response: sender=5 receiver=4 task_id=2 accepted=True",
            "INFO Task assign response: sender=6 receiver=4 task_id=3 accepted=True",
        ))
        + "\n",
        encoding="utf-8",
    )


def _approval_records(
    sys_ids: tuple[int, int, int] = (4, 5, 6),
) -> list[dict[str, object]]:
    return [
        {
            "sys_id": sys_id,
            "task_id": 1,
            "is_confirmed": True,
            "action": "approve",
            "round_uid": "10:1",
            "request_observed": True,
            "response": "approved",
            "request_observed_at_unix_s": 1000.0,
            "approved_at_unix_s": 1015.1,
            "requested_delay_s": 15.0,
            "actual_delay_s": 15.1,
            "nav_before_approval": False,
        }
        for sys_id in sys_ids
    ]


def test_203401_like_oscillation_fails_even_with_good_truth_snap(tmp_path):
    # UAV3's second 203401 final-approach episode, rounded from the real debug CSV.
    # Give it a deliberately passing 0.25 m truth SNAP: certification must still fail
    # on the oscillation/saturation evidence rather than pass on miss alone.
    rolls = [
        24.76, 25.46, 20.96, 11.23, -0.50, -10.88, -25.65, -42.95,
        -45.0, -45.0, -32.99, 22.79, 45.0, 45.0, 45.0, 45.0, 30.10,
        -30.71, -25.63, 3.07, 8.28, 17.43, 6.85, -0.29, -0.56, 0.47,
        1.20, 1.71, 3.63,
    ]
    _write_episode(tmp_path, 1, rolls, snap_m=0.25)

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is False
    assert report.metrics.snap_distance_m == pytest.approx(0.25)
    assert report.metrics.significant_reversals == 4
    assert report.metrics.saturation_fraction > 0.20
    assert report.metrics.max_roll_step_deg > 50.0
    assert any("roll reversals" in error for error in report.errors)
    assert any("roll saturation fraction" in error for error in report.errors)
    assert any("roll-command step" in error for error in report.errors)
    assert not any("truth SNAP distance" in error for error in report.errors)


def test_smooth_trace_with_regular_10x_observations_passes(tmp_path):
    rolls = [15, 14, 12, 9, 6, 3, 1, -1, -3, -4, -5, -5]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is True, report.errors
    assert report.metrics.sample_count == len(rolls)
    assert report.metrics.median_wall_gap_s == pytest.approx(0.01)
    assert report.metrics.max_source_gap_s == pytest.approx(0.1)
    assert report.metrics.observed_speedup == pytest.approx(10.0)
    assert report.metrics.significant_reversals == 0
    assert report.metrics.saturation_fraction == 0.0


def _add_navigation_speedup_evidence(
    log_dir: Path,
    sys_id: int,
    *,
    requested: float,
    restored: float = 10.0,
) -> None:
    path = log_dir / f"uav_{sys_id}_navigation.log"
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "vision_profile='siyi_zr10'\n",
        f"vision_profile='siyi_zr10', nav_sim_speedup={requested}\n",
        1,
    )
    text = text.replace(
        "INFO INIT: NAV MODE\n",
        f"INFO SIM_SPEEDUP={requested}\n"
        "2026-07-10 12:00:01,030 INFO INIT: NAV MODE\n",
        1,
    )
    text = text.replace(
        "INFO RESET: PASSED POI\n",
        "INFO RESET: PASSED POI\n"
        f"2026-07-10 12:00:02,995 INFO SIM_SPEEDUP={restored}\n",
        1,
    )
    path.write_text(text, encoding="utf-8")


def test_positive_gsu_certifies_requested_call_cadence(tmp_path):
    rolls = [15, 14, 12, 9, 6, 3, 1, -1, -3, -4, -5, -5]
    _write_episode(
        tmp_path,
        1,
        rolls,
        snap_m=0.4,
        wall_step_s=0.04,
        source_step_s=0.1,
    )
    _add_navigation_speedup_evidence(tmp_path, 1, requested=2.5)

    report = evaluator.analyze_vehicle(
        tmp_path,
        1,
        "owner",
        approval_count=1,
        navigation_speedup=2.5,
    )

    assert report.passed is True, report.errors
    assert report.metrics.observed_speedup == pytest.approx(2.5)


def test_positive_gsu_rejects_unchanged_launch_call_cadence(tmp_path):
    rolls = [15, 14, 12, 9, 6, 3, 1, -1, -3, -4, -5, -5]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    _add_navigation_speedup_evidence(tmp_path, 1, requested=2.5)

    report = evaluator.analyze_vehicle(
        tmp_path,
        1,
        "owner",
        approval_count=1,
        navigation_speedup=2.5,
    )

    assert report.passed is False
    assert report.metrics.observed_speedup == pytest.approx(10.0)
    assert any("final-approach observation speedup" in error for error in report.errors)


def test_missing_nav_controller_pass_reset_fails(tmp_path):
    """A repeatable demo must finish through NavController's pass gate."""
    rolls = [10, 8, 6, 4, 2, 0, -1, -2]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    navigation_path = tmp_path / "uav_1_navigation.log"
    navigation_path.write_text(
        navigation_path.read_text(encoding="utf-8").replace(
            "2026-07-10 12:00:02,990 INFO RESET: PASSED POI\n", ""
        ),
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any("coordinate pass reset" in error for error in report.errors)


def test_half_meter_truth_boundary_is_strict(tmp_path):
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.5)

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is False
    assert any("is not < 0.50m" in error for error in report.errors)


def test_operator_confirmation_source_must_reach_profile_pixels(tmp_path):
    rolls = [8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    metadata = tmp_path / "uav_1_confirmation_t1_120001_meta.json"
    payload = json.loads(metadata.read_text(encoding="utf-8"))
    payload["bbox_cxcywh"] = [320.0, 240.0, 5.0, 3.0]
    metadata.write_text(json.dumps(payload), encoding="utf-8")

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is False
    assert report.metrics.confirmation_source_size_px == pytest.approx(math.hypot(5, 3))
    # class-0 (medium/large) recognition gate relaxed 48 -> 36 for the demo.
    assert report.metrics.confirmation_required_size_px == 36.0
    assert any("confirmation source" in error for error in report.errors)


def test_max_zoom_best_available_can_certify_smaller_operator_source(tmp_path):
    rolls = [8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    metadata = tmp_path / "uav_1_confirmation_t1_120001_meta.json"
    payload = json.loads(metadata.read_text(encoding="utf-8"))
    payload["bbox_cxcywh"] = [320.0, 240.0, 30.0, 10.0]
    metadata.write_text(json.dumps(payload), encoding="utf-8")
    navigation = tmp_path / "uav_1_navigation.log"
    navigation.write_text(
        navigation.read_text(encoding="utf-8")
        + "CONFIRM gate best available at max zoom for P1\n",
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is True, report.errors


def test_final_approach_event_gap_is_not_mislabeled_as_raw_camera_cadence(tmp_path):
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(
        tmp_path,
        1,
        rolls,
        snap_m=0.1,
        wall_step_s=0.1,
        source_step_s=1.0,
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is True, report.errors
    assert report.metrics.max_source_gap_s == pytest.approx(1.0)
    assert not any("raw source" in error for error in report.errors)


def test_bootstrap_speed_marker_does_not_determine_navigation_speed(tmp_path):
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    path = tmp_path / "uav_1_navigation.log"
    mission_text = path.read_text(encoding="utf-8").replace(
        "(sim_speed=10.0)", "(sim_speed=1.0)"
    )
    stale_configuration_start = (
        "2026-07-10 10:00:00,000 INFO Heartbeat from system 1 received.\n"
        "2026-07-10 10:00:01,000 INFO SIYI ZR10 simulator started "
        "(sim_speed=10.0)\n"
    )
    path.write_text(stale_configuration_start + mission_text, encoding="utf-8")

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is True, report.errors
    assert report.metrics.observed_speedup == pytest.approx(10.0)


def test_any_navigation_speed_write_fails_when_gsu_disabled(tmp_path):
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    navigation_path = tmp_path / "uav_1_navigation.log"
    navigation_path.write_text(
        navigation_path.read_text(encoding="utf-8")
        + "2026-07-10 12:00:03,100 INFO SIM_SPEEDUP=1.0\n",
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any("unexpected navigation SIM_SPEEDUP" in error for error in report.errors)


def test_wrong_vision_profile_fails_sensor_gate(tmp_path):
    """A resolved profile other than siyi_zr10 must not certify."""
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    navigation_path = tmp_path / "uav_1_navigation.log"
    navigation_path.write_text(
        navigation_path.read_text(encoding="utf-8").replace(
            "Using vision profile 'siyi_zr10'",
            "Using vision profile 'siyi_a8'",
        ),
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any(
        "resolved vision profile 'siyi_a8' != siyi_zr10" in error
        for error in report.errors
    )


def test_wrong_mount_pitch_fails_sensor_gate(tmp_path):
    """The mount geometry (pitch) is part of the certified sensor config."""
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    navigation_path = tmp_path / "uav_1_navigation.log"
    navigation_path.write_text(
        navigation_path.read_text(encoding="utf-8").replace(
            "pitch=-14.0, fixed=False",
            "pitch=-9.0, fixed=False",
        ),
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any(
        "resolved mount pitch -9.0 != -14.0" in error for error in report.errors
    )


def test_ideal_360_profile_fails_sensor_gate(tmp_path):
    """The simulator-only ideal_360 upper bound must never certify a run."""
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    navigation_path = tmp_path / "uav_1_navigation.log"
    navigation_path.write_text(
        navigation_path.read_text(encoding="utf-8")
        .replace(
            "Using vision profile 'siyi_zr10'",
            "Using vision profile 'ideal_360'",
        )
        .replace(
            "vision_profile='siyi_zr10'",
            "vision_profile='ideal_360'",
        ),
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any(
        "resolved vision profile 'ideal_360' != siyi_zr10" in error
        for error in report.errors
    )
    assert any(
        "args vision_profile='ideal_360'" in error for error in report.errors
    )


def test_ideal_360_flag_inside_real_profile_fails_sensor_gate(tmp_path):
    """The ideal_360 DETECTOR flag can be enabled inside an otherwise-correct
    profile — name and mount markers stay intact. The detector's activation
    marker must still fail the run (the marker the runtime actually emits)."""
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    navigation_path = tmp_path / "uav_1_navigation.log"
    # All profile/mount markers remain siyi_zr10; only the runtime's
    # static-ideal activation line is present, as it would be with the flag on.
    navigation_path.write_text(
        navigation_path.read_text(encoding="utf-8").replace(
            "Mount 'siyi_zr10': 2560x1440, pitch=-14.0, fixed=False\n",
            "Mount 'siyi_zr10': 2560x1440, pitch=-14.0, fixed=False\n"
            "2026-07-10 12:00:00,007 INFO DetectorSim(siyi_zr10): "
            "static ideal 360 enabled\n",
        ),
        encoding="utf-8",
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any(
        "ideal_360 static sensor active" in error for error in report.errors
    )


def test_restart_companions_stops_all_then_starts_all_sysids():
    """The mission-launch recycle must stop every companion, wait for them to
    be down, then start all configured sysids (so the fresh processes re-read
    the re-applied params). Covers the ac6ac99e production path."""
    state = {"running": True}
    calls: list[tuple[str, object]] = []

    class FakeApi:
        def post_json(self, path, payload=None, *, timeout=30.0):
            calls.append((path, payload))
            if path.endswith("stop-all"):
                state["running"] = False
            elif path.endswith("start-all"):
                state["running"] = True
            return 200, {}

        def get_json(self, path, *, timeout=30.0):
            assert path.endswith("navpy-sim/status")
            return {
                "instances": [
                    {"sys_id": s, "running": state["running"]}
                    for s in (7, 8, 9)
                ]
            }

    context = SimpleNamespace(api=FakeApi(), sys_ids=(7, 8, 9))
    stack_ops.restart_companions(context, timeout_s=5.0)

    paths = [p for p, _ in calls]
    assert paths == [
        "/api/control/navpy-sim/stop-all",
        "/api/control/navpy-sim/start-all",
    ]
    start_payload = next(pl for p, pl in calls if p.endswith("start-all"))
    assert {i["sys_id"] for i in start_payload["instances"]} == {7, 8, 9}


def test_legacy_pitch_state_rows_do_not_change_atomic_command_score(tmp_path):
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)
    before = evaluator.analyze_vehicle(tmp_path, 1, "owner", approval_count=1)
    debug_path = tmp_path / "uav_1_navigation_debug.csv"
    debug_text = debug_path.read_text(encoding="utf-8")
    legacy_row = (
        "12:00:00.050,EVENT:FINAL_APPROACH_PITCH_STATE,"
        "lateral_source=body_fixed_visual_p;pn_available=False,,,,,,,,,\n"
    )
    debug_path.write_text(
        debug_text.replace("\n", f"\n{legacy_row}", 1),
        encoding="utf-8",
    )

    after = evaluator.analyze_vehicle(tmp_path, 1, "owner", approval_count=1)

    assert before.passed is True, before.errors
    assert after.passed is True, after.errors
    assert after.metrics == before.metrics


def test_all_issued_saturation_is_scored_without_acquisition_exemption(tmp_path):
    rolls = [45, 45, 45, 45, 45, 45, 45, 45, 12, 8, 5, 3, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)

    report = evaluator.analyze_vehicle(tmp_path, 1, "owner", approval_count=1)

    assert report.passed is False
    assert report.metrics.max_saturation_run == 8
    assert report.metrics.saturation_fraction == pytest.approx(8 / 14)


def test_trailing_final_saturation_run_is_scored(tmp_path):
    # A trailing saturated run is still issued command evidence and must fail
    # the same consecutive-run gate as a run in the middle of the episode.
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 1, 1, 2, 1, 1, 2, 1, 1,
             45, 45, 45, 45]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert report.metrics.max_saturation_run == 4
    assert report.metrics.saturation_fraction <= 0.20
    assert any(
        "consecutive saturated commands" in error for error in report.errors
    )


def test_mid_episode_saturation_run_still_fails_run_gate(tmp_path):
    """A long saturated run in the middle of PN tracking remains a failure."""
    rolls = [5, 4, 45, 45, 45, 45, 45, 4, 3, 2, 1, 1, 2, 1, 1, 2, 1, 1, 2, 1,
             1, 2, 1, 1, 2, 1]
    _write_episode(tmp_path, 1, rolls, snap_m=0.4)

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1,
    )

    assert report.passed is False
    assert any(
        "consecutive saturated commands" in error for error in report.errors
    )


def test_final_approach_clock_ratio_rejects_1x_despite_10x_bootstrap_marker(tmp_path):
    rolls = [10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
    _write_episode(
        tmp_path,
        1,
        rolls,
        snap_m=0.1,
        wall_step_s=0.1,
        source_step_s=0.1,
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is False
    assert report.metrics.observed_speedup == pytest.approx(1.0)
    assert any("final-approach observation speedup" in error for error in report.errors)


def test_nonfinite_truth_snap_is_reported_explicitly(tmp_path):
    _write_episode(tmp_path, 1, [], snap_m=float("inf"))

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is False
    assert math.isinf(report.metrics.snap_distance_m)
    assert any("truth SNAP distance is non-finite" in error for error in report.errors)
    assert any("observation count 0" in error for error in report.errors)


def test_navigation_nonfinite_snap_survives_empty_compact_and_debug_logs(tmp_path):
    _write_episode(tmp_path, 1, [], snap_m=float("inf"))
    (tmp_path / "uav_1_navigation_compact.csv").write_text(
        "ts,dist,h_dist,v_dist,cmd_r,cmd_p,yaw_err,pitch_err,act_r,act_p,x_err,y_err\n",
        encoding="utf-8",
    )
    (tmp_path / "uav_1_navigation_debug.csv").write_text(
        "ts,event,payload,,,,,,,,,\n", encoding="utf-8"
    )

    report = evaluator.analyze_vehicle(
        tmp_path, 1, "owner", approval_count=1
    )

    assert report.passed is False
    assert math.isinf(report.metrics.snap_distance_m)
    assert any("truth SNAP distance is non-finite" in error for error in report.errors)
    assert any("compact SNAP episode" in error for error in report.errors)


def test_exact_three_uav_run_passes_owner_and_peer_workflow_gates(tmp_path):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    approvals = _approval_records()

    report = evaluator.analyze_run(
        tmp_path, sys_ids=(4, 5, 6), approvals=approvals
    )

    assert report.passed is True
    assert [vehicle.role for vehicle in report.vehicles] == ["owner", "peer", "peer"]


def _write_acked_backend_log(log_dir: Path) -> None:
    """The same auction in the acked format: copies, a reject, APPLIED."""
    (log_dir / "gcs_backend.log").write_text(
        "\n".join((
            "INFO Task assign request: sender=4 receiver=5 task_id=2 "
            "at (40.000500, 44.000000) uid=40:10",
            "INFO Task assign request: sender=4 receiver=5 task_id=2 "
            "at (40.000500, 44.000000) uid=40:12",
            "INFO Task assign request: sender=4 receiver=6 task_id=3 "
            "at (40.000600, 44.000000) uid=40:11",
            "INFO Task assign response: sender=6 receiver=4 task_id=3 "
            "accepted=False uid=60:3",
            "INFO Task assign request: sender=4 receiver=6 task_id=3 "
            "at (40.000600, 44.000000) uid=40:20",
            "INFO Task assign response: sender=5 receiver=4 task_id=2 "
            "accepted=True uid=50:7",
            "INFO Task assign ack: owner=4 helper=5 task_id=2 status=APPLIED "
            "ref=50:7 uid=40:13",
            "INFO Task assign response: sender=6 receiver=4 task_id=3 "
            "accepted=True uid=60:8",
            "INFO Task assign ack: owner=4 helper=6 task_id=3 status=APPLIED "
            "ref=60:8 uid=40:21",
        ))
        + "\n",
        encoding="utf-8",
    )


def _log_assignment(log_dir: Path, sys_id: int, line: str, *, after: bool = False) -> None:
    path = log_dir / f"uav_{sys_id}_navigation.log"
    approach = "GUIDED_LOITER cmd=DO_REPOSITION center=40."
    text = path.read_text(encoding="utf-8")
    start = text.index(approach)
    end = text.index("\n", start) + 1
    if after:
        text = text[:end] + f"{line}\n" + text[end:]
    else:
        text = text[:start] + f"{line}\n" + text[start:]
    path.write_text(text, encoding="utf-8")


def _acked_run(tmp_path: Path) -> None:
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    _write_acked_backend_log(tmp_path)


def test_acked_run_passes_when_peers_fly_after_their_owner_applied(tmp_path):
    _acked_run(tmp_path)
    _log_assignment(tmp_path, 5, "Task 2 assigned by owner 4")
    _log_assignment(tmp_path, 6, "Task 3 assigned by owner 4")

    report = evaluator.analyze_run(
        tmp_path, sys_ids=(4, 5, 6), approvals=_approval_records()
    )

    assert report.passed is True, report.errors + [
        error for vehicle in report.vehicles for error in vehicle.errors
    ]
    assert report.global_assignments == {4: 1, 5: 2, 6: 3}


@pytest.mark.parametrize(
    ("after", "message"),
    [
        (None, "peer never logged 'Task T assigned by owner O'"),
        (True, "peer started GUIDED_LOITER before 'Task T assigned by owner O'"),
    ],
)
def test_acked_run_fails_a_peer_that_flew_before_its_owner_applied(
    tmp_path, after, message,
):
    _acked_run(tmp_path)
    _log_assignment(tmp_path, 5, "Task 2 assigned by owner 4")
    if after is not None:
        _log_assignment(tmp_path, 6, "Task 3 assigned by owner 4", after=after)

    report = evaluator.analyze_run(
        tmp_path, sys_ids=(4, 5, 6), approvals=_approval_records()
    )

    assert report.passed is False
    peer_5, peer_6 = report.vehicles[1], report.vehicles[2]
    assert not any("assigned by owner" in error for error in peer_5.errors)
    assert message in peer_6.errors


@pytest.mark.parametrize(
    ("surface", "old", "new"),
    [
        ("approval", "", ""),
        ("navigation", "CONFIRMING: P1", "CONFIRMING: P99"),
        (
            "navigation",
            "POI 1 confirmed by ground station.",
            "POI 99 confirmed by ground station.",
        ),
        ("navigation", "POI: P1", "POI: P99"),
        ("debug", "task=1;obj=11", "task=99;obj=11"),
    ],
)
def test_exact_run_binds_approval_confirmation_selection_and_commands(
    tmp_path,
    surface,
    old,
    new,
):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    approvals = _approval_records()
    if surface == "approval":
        approvals[1]["task_id"] = 99
    else:
        suffix = "navigation.log" if surface == "navigation" else "navigation_debug.csv"
        path = tmp_path / f"uav_5_{suffix}"
        path.write_text(
            path.read_text(encoding="utf-8").replace(old, new),
            encoding="utf-8",
        )

    report = evaluator.analyze_run(
        tmp_path,
        sys_ids=(4, 5, 6),
        approvals=approvals,
    )

    assert report.passed is False
    assert any(
        "local POI identity chain differs" in error
        or "final-approach command identity" in error
        for error in report.errors
    )


def test_owner_first_auction_acceptance_proves_peer_dispatch(tmp_path):
    """A successful first auction has no rebroadcast marker (live run 002351)."""
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    owner_log = tmp_path / "uav_4_navigation.log"
    owner_log.write_text(
        owner_log.read_text(encoding="utf-8")
        .replace("Rebroadcast task 2, waiting for 2 peers", "Task 2 accepted by 5")
        .replace("Rebroadcast task 3, waiting for 2 peers", "Task 3 accepted by 6"),
        encoding="utf-8",
    )
    approvals = _approval_records()

    report = evaluator.analyze_run(
        tmp_path, sys_ids=(4, 5, 6), approvals=approvals,
    )

    assert report.passed is True, report.errors


def test_exact_run_rejects_both_peers_at_same_poi_coordinate(tmp_path):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    peer_six = tmp_path / "uav_6_navigation.log"
    peer_six.write_text(
        peer_six.read_text(encoding="utf-8").replace("40.0006", "40.0005"),
        encoding="utf-8",
    )
    approvals = _approval_records()

    report = evaluator.analyze_run(
        tmp_path, sys_ids=(4, 5, 6), approvals=approvals
    )

    assert report.passed is False
    assert any("final-approach truth POI differs" in error for error in report.errors)


def test_exact_run_rejects_distinct_but_wrong_peer_coordinates(tmp_path):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    for sys_id in (5, 6):
        path = tmp_path / f"uav_{sys_id}_navigation.log"
        path.write_text(
            path.read_text(encoding="utf-8").replace(
                f"40.000{sys_id}", f"41.000{sys_id}"
            ),
            encoding="utf-8",
        )
    approvals = _approval_records()

    report = evaluator.analyze_run(
        tmp_path, sys_ids=(4, 5, 6), approvals=approvals,
    )

    assert report.passed is False
    assert any(
        "final-approach truth POI differs" in error
        for error in report.errors
    )


def test_exact_run_rejects_peer_centers_swapped_against_approved_tasks(tmp_path):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    peer_five = tmp_path / "uav_5_navigation.log"
    peer_six = tmp_path / "uav_6_navigation.log"
    peer_five.write_text(
        peer_five.read_text(encoding="utf-8").replace("40.0005", "40.0006"),
        encoding="utf-8",
    )
    peer_six.write_text(
        peer_six.read_text(encoding="utf-8").replace("40.0006", "40.0005"),
        encoding="utf-8",
    )

    report = evaluator.analyze_run(
        tmp_path,
        sys_ids=(4, 5, 6),
        approvals=_approval_records(),
    )

    assert report.passed is False
    assert any("final-approach truth POI differs" in error for error in report.errors)


def test_exact_run_validates_commands_against_captured_autopilot_bounds(tmp_path):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    debug = tmp_path / "uav_5_navigation_debug.csv"
    debug.write_text(
        debug.read_text(encoding="utf-8")
        .replace("cmd_roll=12.00", "cmd_roll=46.00", 1)
        .replace("cmd_pitch=-10;cmd_thr=0.55", "cmd_pitch=-50;cmd_thr=0.75"),
        encoding="utf-8",
    )

    report = evaluator.analyze_run(
        tmp_path,
        sys_ids=(4, 5, 6),
        approvals=_approval_records(),
    )

    assert report.passed is False
    peer_errors = report.vehicles[1].errors
    assert any("ROLL_LIMIT_DEG" in error for error in peer_errors)
    assert any("captured bounds" in error for error in peer_errors)
    assert any("captured runtime value" in error for error in peer_errors)


def test_explicit_empty_approvals_do_not_fall_back_to_disk(tmp_path):
    rolls = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(tmp_path, sys_id, rolls, snap_m=0.4, role=role)
    _write_resolved_plan(tmp_path)
    (tmp_path / "operator_approvals.json").write_text(
        json.dumps({"approvals": _approval_records()}),
        encoding="utf-8",
    )

    report = evaluator.analyze_run(
        tmp_path,
        sys_ids=(4, 5, 6),
        approvals=[],
    )

    assert report.passed is False
    assert report.approvals == []
    assert all(vehicle.metrics.sample_count for vehicle in report.vehicles)


def test_configuration_keeps_explicit_ten_x_navigation_and_no_multicontainer_scope():
    settings = evaluator.build_settings(18123)

    assert settings["simulation"]["simulated_vehicle_count"] == 3
    assert settings["flight"]["uavs_per_set"] == 3
    assert settings["launch"]["launch_type"] == "container"
    assert "container_gap_s" not in settings["launch"]
    assert settings["camera"]["vision_profile"] == "siyi_zr10"
    assert evaluator.aas_params_for(4, 4)["targ_wps"] == 76
    assert evaluator.aas_params_for(5, 4)["targ_wps"] == 0
    assert evaluator.aas_params_for(4, 4)["del_ctrl"] == 2
    assert evaluator.aas_params_for(4, 4)["del_dir"] is True
    assert evaluator.aas_params_for(4, 4)["use_trn"] is True
    assert evaluator.aas_params_for(4, 4)["targ_alt"] == 0.0
    assert evaluator.aas_params_for(4, 4)["nav_last_wp"] == 2
    assert evaluator.aas_params_for(4, 4)["nav_auto_cm"] is False
    assert evaluator.aas_params_for(4, 4)["nav_cm_fl"] is False
    assert evaluator.aas_params_for(4, 4)["nav_cwt"] == 30.0
    assert evaluator.aas_params_for(4, 4)["nav_oneshot"] is True
    assert evaluator.full_param_changes() == [
        {"name": "ROLL_LIMIT_DEG", "value": 45.0},
        {"name": "SIM_SPEEDUP", "value": 10.0},
    ]
    command = evaluator.launch_command()
    assert command[-1] == "--no-browser"
    assert "--chat" not in command
    assert "-gsu" not in command
    assert evaluator.NAVIGATION_SPEEDUP_OVERRIDE == 0.0
    assert evaluator.GateLimits().max_snap_distance_m == 0.5
    assert evaluator.GateLimits().max_significant_reversals == 1


def test_cli_defaults_to_manual_operator_delay_and_half_meter_truth_gate():
    args = evaluator.parse_args([])

    assert args.confirmation_delay_s == 15.0
    assert args.max_snap_distance_m == 0.5


def test_second_evaluator_for_same_worktree_is_rejected(tmp_path):
    lock_path = tmp_path / "evaluator.lock"

    with evaluator.exclusive_evaluator_lock(lock_path):
        with pytest.raises(evaluator.RegressionError, match="already running"):
            with evaluator.exclusive_evaluator_lock(lock_path):
                pass

    # Releasing the first process/file handle makes the same worktree usable again.
    with evaluator.exclusive_evaluator_lock(lock_path):
        pass


def test_owned_stack_always_uses_scoped_stop_when_launch_fails(monkeypatch):
    stopped = []

    def fail_launch(env, timeout_s):
        raise evaluator.RegressionError("launch failed")

    monkeypatch.setattr(demo_process, "launch_stack", fail_launch)
    monkeypatch.setattr(
        demo_process,
        "stop_stack",
        lambda env: stopped.append(evaluator.stop_command()),
    )

    with pytest.raises(evaluator.RegressionError, match="launch failed"):
        demo_process.with_owned_stack({}, 1.0, lambda context: None)

    assert stopped == [evaluator.stop_command()]


def test_owned_stack_rejects_preexisting_generation_without_stopping_it(
    monkeypatch,
):
    stopped = []
    monkeypatch.setattr(
        demo_process,
        "launch_stack",
        lambda *_: (_ for _ in ()).throw(
            demo_process.PreexistingOwnedStackError("already exists")
        ),
    )
    monkeypatch.setattr(
        demo_process,
        "stop_stack",
        lambda *_: stopped.append(True),
    )

    with pytest.raises(evaluator.RegressionError, match="already exists"):
        demo_process.with_owned_stack({}, 1.0, lambda _context: None)

    assert stopped == []


def test_verified_stack_entry_is_bound_to_exact_launch_token(monkeypatch):
    entry = {
        "sitl_launch_token": "ours",
        "sitl_verified": True,
        "sitl": True,
        "sitl_speedup": 10.0,
        "sitl_pid": 1234,
        "sitl_pid_start": 55.0,
        "sysids": [1, 2, 3],
    }
    monkeypatch.setattr(demo_process, "owned_entry", lambda _env: entry)

    assert demo_process._verified_launch_entry({}, "foreign") is None
    assert demo_process._verified_launch_entry({}, "ours") is entry

    entry["sitl_verified"] = None
    assert demo_process._verified_launch_entry({}, "ours") is None
    entry["sitl_verified"] = False
    entry["sitl_error"] = "rate mismatch"
    with pytest.raises(evaluator.RegressionError, match="rate mismatch"):
        demo_process._verified_launch_entry({}, "ours")


def test_live_run_remembers_every_configured_sysid_set(monkeypatch, tmp_path):
    """A slot sequence A -> B -> A must run A, not configure it twice."""
    sysid_sequence = [
        (1, 2, 3),
        (4, 5, 6),
        (1, 2, 3),
    ]
    configured = []
    missions = []

    def with_owned_stack(env, timeout_s, action):
        del env, timeout_s
        return action(SimpleNamespace(sys_ids=sysid_sequence.pop(0)))

    def configure(context, timeout_s):
        del timeout_s
        configured.append(context.sys_ids)

    def run_mission(context, log_dir, timeout_s, confirmation_delay_s):
        del log_dir, timeout_s, confirmation_delay_s
        missions.append(context.sys_ids)
        return evaluator.MissionArtifacts(
            context.sys_ids,
            _approval_records(context.sys_ids),
            "backend evidence",
        )

    monkeypatch.setattr(demo_runtime, "with_owned_stack", with_owned_stack)
    monkeypatch.setattr(demo_runtime, "configure_vehicle_params", configure)
    monkeypatch.setattr(demo_runtime, "run_mission", run_mission)

    result = demo_runtime.execute_live_run(
        tmp_path,
        timeout_s=1.0,
        confirmation_delay_s=0.0,
    )

    assert result.sys_ids == (1, 2, 3)
    assert configured == [(1, 2, 3), (4, 5, 6)]
    assert missions == [(1, 2, 3)]
