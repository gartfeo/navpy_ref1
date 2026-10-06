from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import eval_ideal_three_uav_analysis as analysis
from scripts import eval_ideal_three_uav as exact_cli
from scripts import eval_ideal_three_uav_runtime as runtime
from scripts.eval_gcs_demo_config import aas_params_for
from scripts.eval_gcs_demo_models import MissionArtifacts
from scripts.eval_gcs_demo_upload import (
    mission_assignment,
    mission_assignment_digest,
    shifted_mission_assignments,
)
from tests.scripts.test_eval_gcs_navigation_demo import (
    _approval_records,
    _write_episode,
    _write_resolved_plan,
)


def _write_source_time_evidence(path: Path) -> None:
    (path / "stack_launch_evidence.json").write_text(
        json.dumps({
            "launch_token": "fixture-token",
            "sitl": True,
            "sitl_verified": True,
            "sitl_speedup": 10.0,
            "sitl_pid": 9001,
            "sitl_pid_start": 123.0,
            "sysids": [4, 5, 6],
            "backend": 8123,
        }),
        encoding="utf-8",
    )
    source_dir = path / "source-time"
    source_dir.mkdir()
    restart_instances = []
    for sys_id in (4, 5, 6):
        process_pid = 4000 + sys_id
        restart_instances.append({
            "sys_id": sys_id,
            "status": "ready",
            "old_pid": 1000 + sys_id,
            "pid": 2000 + sys_id,
            "runtime": {
                "process_pid": process_pid,
                "scheduler_rate_hz": 50.0,
                "mission_items": 16,
                "targ_wps": aas_params_for(sys_id, 4)["targ_wps"],
                "nav_last_wp": 2,
                "nav_min_wp_seq": 9,
            },
        })
        prefix = f"_uav_{sys_id}_pid_{process_pid}.csv"
        attitude = ["wall_s,boot_ms"] + [
            f"{100.0 + index * 0.0025:.4f},{1000 + index * 25}"
            for index in range(20)
        ]
        detector = ["wall_s,boot_s,outcome,frame_ts"] + [
            f"{100.0 + index * 0.0025:.4f},{1.0 + index * 0.025:.3f},"
            f"emitted,{1.0 + index * 0.025:.3f}"
            for index in range(20)
        ]
        observation = ["wall_s,obs_ts,src_now_s,outcome"] + [
            f"{100.0 + index * 0.0025:.4f},{1.0 + index * 0.025:.3f},"
            f"{1.0 + index * 0.025:.3f},fresh"
            for index in range(20)
        ]
        worker = ["wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome"] + [
            f"{100.0 + index * 0.002:.4f},0.5,"
            f"{1.0 + index * 0.02:.3f},{1.0 + index * 0.02:.3f},measured,fresh"
            for index in range(20)
        ]
        for stream, rows in (
            ("attitude_arrival", attitude),
            ("detector_pose", detector),
            ("observation", observation),
            ("worker", worker),
        ):
            (source_dir / f"pose_cadence_{stream}{prefix}").write_text(
                "\n".join(rows) + "\n",
                encoding="utf-8",
            )
    nav_start_bindings = [
        {
            "sys_id": sys_id,
            "nav_waypoint_ordinal": 2,
            "mission_sequence": 9,
            "mission_count": 16,
        }
        for sys_id in (4, 5, 6)
    ]
    post_restart_status = [
        {
            "sys_id": instance["sys_id"],
            "pid": instance["pid"],
            "running": True,
            "ready": True,
            "exit_code": None,
            "runtime": instance["runtime"],
        }
        for instance in restart_instances
    ]
    (path / "mission_upload_evidence.json").write_text(
        json.dumps({
            "restart_instances": restart_instances,
            "nav_start_bindings": nav_start_bindings,
            "post_restart_status": post_restart_status,
        }),
        encoding="utf-8",
    )


def _downloaded_mission(sys_id: int) -> dict:
    return {
        "sys_id": sys_id,
        "waypoints": [
            {
                "lat": 40.0 + index * 0.001,
                "lon": 44.0 + index * 0.001,
                "alt": 100.0,
                "mission_sequence": index + 2,
                "command": 16,
                "nav_waypoint_ordinal": index + 1,
            }
            for index in range(7)
        ],
        "altitude_m": 100.0,
        "mission_count": 9,
        "corridor_end_index": None,
        "search_pattern": "distributed",
        "dock_classes": ["medium"],
        "polygon": [],
        "corridor_backbone": [],
        "launch_point": {"lat": 40.0, "lon": 44.0},
        "fallback_delivery_location": None,
    }


@pytest.mark.parametrize("pattern", ["distributed", "corridor"])
def test_mission_translation_preserves_search_pattern(pattern):
    mission = _downloaded_mission(4)
    mission["search_pattern"] = pattern
    assignment = mission_assignment(mission, expected_sys_id=4, zone_index=0)
    assert assignment["search_pattern"] == pattern
    assert "tactic" not in assignment


def test_mission_translation_changes_digest_and_every_coordinate():
    mission = _downloaded_mission(4)
    mission["fallback_delivery_location"] = {"lat": 40.0, "lon": 44.0, "type": "other"}
    original = [
        mission_assignment(
            mission,
            expected_sys_id=4,
            zone_index=0,
        )
    ]

    shifted = shifted_mission_assignments(original)

    assert mission_assignment_digest(shifted) != mission_assignment_digest(original)
    assert shifted[0]["waypoints"][0]["lat"] == 40.0002
    assert shifted[0]["waypoints"][0]["lon"] == 43.9998
    assert shifted[0]["launch_point"]["lat"] == 40.0002
    assert original[0]["waypoints"][0]["lat"] == 40.0
    assert original[0]["dock_classes"] == mission["dock_classes"]
    assert "poi_classes" not in original[0]
    assert "default_poi" not in original[0]
    assert original[0]["fallback_delivery_location"] == mission["fallback_delivery_location"]
    assert shifted[0]["fallback_delivery_location"] == {"lat": 40.0002, "lon": 43.9998, "type": "other"}


def test_candidate_digest_binds_untracked_file_content():
    first = exact_cli._candidate_content_sha256(
        "abc",
        b"tracked",
        [("new.py", b"one")],
    )
    second = exact_cli._candidate_content_sha256(
        "abc",
        b"tracked",
        [("new.py", b"two")],
    )

    assert first != second


def test_nav_start_binding_resolves_ordinal_two_to_sequence_nine():
    missions = {}
    for sys_id in (4, 5, 6):
        mission = _downloaded_mission(sys_id)
        mission["mission_count"] = 16
        mission["waypoints"][1]["mission_sequence"] = 9
        missions[sys_id] = mission

    class Api:
        @staticmethod
        def get_json(path, **_kwargs):
            sys_id = int(path.split("/")[3])
            return missions[sys_id]

    context = SimpleNamespace(sys_ids=(4, 5, 6), api=Api())

    bindings = runtime.capture_nav_start_bindings(
        context,
        {4: 2, 5: 2, 6: 2},
    )

    assert [binding["mission_sequence"] for binding in bindings] == [9, 9, 9]
    assert [binding["mission_count"] for binding in bindings] == [16, 16, 16]


def test_run_ideal_mission_restarts_ready_after_confirmed_configuration(
    monkeypatch,
    tmp_path: Path,
):
    calls: list[str] = []
    original = [{"version": "original"}]
    shifted = [{"version": "shifted"}]
    context = SimpleNamespace(
        sys_ids=(4, 5, 6),
        api=SimpleNamespace(),
        backend_tail=SimpleNamespace(read_new=lambda: "", text="backend"),
    )

    monkeypatch.setattr(runtime, "wait_for_vehicles", lambda *_: calls.append("vehicles"))
    monkeypatch.setattr(runtime, "wait_for_companions", lambda *_: calls.append("companions"))
    monkeypatch.setattr(runtime, "capture_mission_assignments", lambda *_: calls.append("capture") or original)
    monkeypatch.setattr(runtime, "shifted_mission_assignments", lambda *_: calls.append("shift") or shifted)
    monkeypatch.setattr(runtime, "mission_assignment_digest", lambda value: value[0]["version"])

    def upload(_context, value):
        calls.append(f"upload_{value[0]['version']}")

    monkeypatch.setattr(runtime, "upload_mission_assignments", upload)
    nav_bindings = [
        {
            "sys_id": sys_id,
            "nav_waypoint_ordinal": 2,
            "mission_sequence": 9,
            "mission_count": 16,
        }
        for sys_id in (4, 5, 6)
    ]
    monkeypatch.setattr(
        runtime,
        "capture_nav_start_bindings",
        lambda *_: calls.append("nav_bindings") or nav_bindings,
    )
    monkeypatch.setattr(runtime, "configure_vehicle_params", lambda *_: calls.append("params") or object())
    monkeypatch.setattr(runtime, "verify_aas_profile", lambda *_: calls.append("verify"))
    monkeypatch.setattr(
        runtime,
        "restart_configured_companions_ready",
        lambda *_: calls.append("restart_ready") or [{"sys_id": value} for value in (4, 5, 6)],
    )
    monkeypatch.setattr(
        runtime,
        "capture_post_restart_status",
        lambda *_: calls.append("post_status") or [],
    )
    monkeypatch.setattr(runtime, "start_esp32_simulator", lambda *_: calls.append("esp32"))
    monkeypatch.setattr(runtime, "wait_for_missions", lambda *_: calls.append("missions"))
    monkeypatch.setattr(runtime, "resolve_live_plan", lambda *_: calls.append("resolve") or SimpleNamespace(sys_ids=(4, 5, 6)))
    monkeypatch.setattr(runtime, "persist_command_bounds", lambda *_: calls.append("bounds"))
    monkeypatch.setattr(runtime, "prepare_container_launch", lambda *_: calls.append("start"))
    monkeypatch.setattr(
        runtime,
        "approve_requests_until_snap",
        lambda *_: calls.append("approve") or _approval_records(),
    )
    # Replace the module-level ``time`` binding with a namespace so the fake
    # sleep is confined to the runtime module and never mutates the shared
    # stdlib ``time.sleep`` (which would busy-spin any leaked background
    # sleeper during this test).
    monkeypatch.setattr(
        runtime,
        "time",
        SimpleNamespace(sleep=lambda _seconds: calls.append("source_flush")),
    )

    result = runtime.run_ideal_mission(context, tmp_path, 30.0)

    assert result.sys_ids == (4, 5, 6)
    assert calls == [
        "vehicles",
        "companions",
        "capture",
        "shift",
        "upload_shifted",
        "nav_bindings",
        "params",
        "verify",
        "restart_ready",
        "post_status",
        "esp32",
        "missions",
        "resolve",
        "bounds",
        "start",
        "approve",
        "source_flush",
        "upload_original",
    ]


def test_execute_ideal_live_run_uses_one_owned_stack(monkeypatch, tmp_path: Path):
    calls: list[str] = []
    mission = MissionArtifacts((4, 5, 6), [], "")
    context = SimpleNamespace(
        backend_tail=SimpleNamespace(read_new=lambda: "", text=""),
        launch_evidence={"sitl_verified": True},
    )
    monkeypatch.setattr(runtime, "free_tcp_port", lambda: 12345)
    monkeypatch.setattr(runtime, "build_environment", lambda *_: {})
    monkeypatch.setattr(runtime, "run_ideal_mission", lambda *_: calls.append("mission") or mission)

    def with_stack(_environment, _timeout, action):
        calls.append("launch")
        return action(context)

    monkeypatch.setattr(runtime, "with_owned_stack", with_stack)

    run_dir = tmp_path / "run"
    run_dir.mkdir()
    result = runtime.execute_ideal_live_run(run_dir, 30.0)

    assert result is mission
    assert calls == ["launch", "mission"]


def _write_ideal_run(path: Path) -> list[dict]:
    path.mkdir(parents=True, exist_ok=True)
    _write_resolved_plan(path)
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(
            path,
            sys_id,
            [10.0 - index * 0.5 for index in range(10)],
            snap_m=0.4,
            role=role,
        )
        log_path = path / f"uav_{sys_id}_navigation.log"
        text = log_path.read_text(encoding="utf-8")
        text = text.replace("siyi_zr10", "ideal_360").replace(
            "2026-07-10 12:00:00,010 INFO",
            "2026-07-10 12:00:00,008 INFO DetectorSim(ideal_360): "
            "static ideal 360 enabled\n2026-07-10 12:00:00,010 INFO",
        )
        log_path.write_text(text, encoding="utf-8")
    _write_source_time_evidence(path)
    return _approval_records()


def test_ideal_analysis_uses_ready_python_pid_for_source_cadence(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    stale = tmp_path / "source-time" / "pose_cadence_worker_uav_4_pid_9999.csv"
    stale.write_text(
        "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome\n"
        "1,1,1,1,measured,fresh\n",
        encoding="utf-8",
    )

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    assert report.passed is True, report.errors
    owner = next(item for item in report.vehicles if item.sys_id == 4)
    assert owner.metrics.source_process_pid == 4004
    assert owner.metrics.scheduler_rate_hz == 50.0
    assert owner.metrics.command_wall_gap_p50_ms == pytest.approx(2.0)


def test_ideal_analysis_accepts_isolated_fixed_grid_phase_compression(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    path = (
        tmp_path
        / "source-time"
        / "pose_cadence_worker_uav_4_pid_4004.csv"
    )
    rows = path.read_text(encoding="utf-8").splitlines()
    # A late wake can start one fixed-grid slot at 18.900 ms and still finish
    # before the following nominal slot. Starting that distinct next slot
    # 0.438 ms later is phase compression, not replay of a missed slot.
    rows[10] = rows[10].replace("100.0180", "100.0189")
    rows[11] = rows[11].replace("100.0200", "100.0193384")
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    owner = next(item for item in report.vehicles if item.sys_id == 4)
    assert report.passed is True, owner.errors


def test_ideal_analysis_binds_audited_approval_to_final_approach_poi(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    approvals[1]["task_id"] = 99

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    assert report.passed is False
    assert any(
        "local POI identity chain differs" in error
        for error in report.errors
    )


def test_ideal_analysis_fails_when_ready_python_pid_has_no_worker_evidence(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    active = (
        tmp_path
        / "source-time"
        / "pose_cadence_worker_uav_5_pid_4005.csv"
    )
    active.unlink()
    stale = active.with_name("pose_cadence_worker_uav_5_pid_9999.csv")
    stale.write_text(
        "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome\n"
        + "\n".join(
            f"{index * 0.002},0.5,{index * 0.02},{index * 0.02},measured,fresh"
            for index in range(20)
        )
        + "\n",
        encoding="utf-8",
    )

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    peer = next(item for item in report.vehicles if item.sys_id == 5)
    assert report.passed is False
    assert any("active runtime pid 4005" in error for error in peer.errors)


def test_ideal_analysis_fails_slow_active_command_cadence(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    path = (
        tmp_path
        / "source-time"
        / "pose_cadence_worker_uav_6_pid_4006.csv"
    )
    rows = path.read_text(encoding="utf-8").splitlines()
    slow = [rows[0]] + [
        f"{100.0 + index * 0.0079:.4f},0.5,"
        f"{1.0 + index * 0.02:.3f},{1.0 + index * 0.02:.3f},measured,fresh"
        for index in range(20)
    ]
    path.write_text("\n".join(slow) + "\n", encoding="utf-8")

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    peer = next(item for item in report.vehicles if item.sys_id == 6)
    assert report.passed is False
    assert any("command wall-gap p50" in error for error in peer.errors)


@pytest.mark.parametrize(
    ("stream", "expected_error"),
    [
        ("detector_pose", "ideal frame maximum source gap"),
        ("observation", "final-approach observation maximum source gap"),
        ("worker", "fresh final-approach command maximum source gap"),
    ],
)
def test_ideal_analysis_rejects_ten_second_source_stage_gaps(
    tmp_path,
    stream,
    expected_error,
):
    approvals = _write_ideal_run(tmp_path)
    path = (
        tmp_path
        / "source-time"
        / f"pose_cadence_{stream}_uav_4_pid_4004.csv"
    )
    if stream == "detector_pose":
        rows = ["wall_s,boot_s,outcome,frame_ts"] + [
            f"{100.0 + index * 0.002:.4f},{1.0 + index * 10:.3f},"
            f"emitted,{1.0 + index * 10:.3f}"
            for index in range(20)
        ]
    elif stream == "observation":
        rows = ["wall_s,obs_ts,src_now_s,outcome"] + [
            f"{100.0 + index * 0.002:.4f},{1.0 + index * 10:.3f},"
            f"{1.0 + index * 10:.3f},fresh"
            for index in range(20)
        ]
    else:
        rows = ["wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome"] + [
            f"{100.0 + index * 0.002:.4f},0.5,"
            f"{1.0 + index * 10:.3f},{1.0 + index * 10:.3f},measured,fresh"
            for index in range(20)
        ]
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    vehicle = next(item for item in report.vehicles if item.sys_id == 4)
    assert report.passed is False
    assert any(expected_error in error for error in vehicle.errors)


def test_ideal_analysis_requires_ten_fresh_observations(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    path = (
        tmp_path
        / "source-time"
        / "pose_cadence_observation_uav_4_pid_4004.csv"
    )
    rows = ["wall_s,obs_ts,src_now_s,outcome"] + [
        f"{100.0 + index * 0.002:.4f},1.0,1.0,"
        f"{'fresh' if index == 0 else 'duplicate'}"
        for index in range(20)
    ]
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    vehicle = next(item for item in report.vehicles if item.sys_id == 4)
    assert report.passed is False
    assert any("fresh final-approach observations 1 < 10" in error for error in vehicle.errors)


def test_ideal_analysis_rejects_zero_command_wall_gaps(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    path = (
        tmp_path
        / "source-time"
        / "pose_cadence_worker_uav_4_pid_4004.csv"
    )
    rows = ["wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome"] + [
        f"100.0,0.5,{1.0 + index * 0.02:.3f},"
        f"{1.0 + index * 0.02:.3f},measured,fresh"
        for index in range(20)
    ]
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    vehicle = next(item for item in report.vehicles if item.sys_id == 4)
    assert report.passed is False
    assert any("nonpositive delta" in error for error in vehicle.errors)


@pytest.mark.parametrize(
    ("surface", "field", "value"),
    [
        ("stack", "sitl_verified", False),
        ("restart", "status", "failed"),
        ("restart", "old_pid", 2004),
        ("runtime", "mission_items", 0),
        ("runtime", "targ_wps", 99),
        ("runtime", "nav_last_wp", 3),
        ("runtime", "nav_min_wp_seq", 8),
        ("binding", "mission_sequence", 8),
        ("post", "pid", 9999),
    ],
)
def test_ideal_analysis_rejects_invalid_runtime_readiness_identity(
    tmp_path,
    surface,
    field,
    value,
):
    approvals = _write_ideal_run(tmp_path)
    path = tmp_path / "mission_upload_evidence.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    if surface == "stack":
        stack_path = tmp_path / "stack_launch_evidence.json"
        stack = json.loads(stack_path.read_text(encoding="utf-8"))
        stack[field] = value
        stack_path.write_text(json.dumps(stack), encoding="utf-8")
    elif surface == "restart":
        payload["restart_instances"][0][field] = value
    elif surface == "runtime":
        payload["restart_instances"][0]["runtime"][field] = value
    elif surface == "binding":
        payload["nav_start_bindings"][0][field] = value
    else:
        payload["post_restart_status"][0][field] = value
    path.write_text(json.dumps(payload), encoding="utf-8")

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    assert report.passed is False
    assert any("invalid active-runtime evidence" in error for error in report.errors)


def test_ideal_analysis_rejects_guided_failure_and_missing_third_snap(tmp_path):
    approvals = _write_ideal_run(tmp_path)
    owner = tmp_path / "uav_4_navigation.log"
    owner.write_text(
        owner.read_text(encoding="utf-8").replace(
            "2026-07-10 12:00:01,030 INFO INIT: NAV MODE",
            "2026-07-10 12:00:01,025 WARNING GUIDED not accepted within 5s\n"
            "2026-07-10 12:00:01,030 INFO INIT: NAV MODE",
        ),
        encoding="utf-8",
    )
    peer = tmp_path / "uav_6_navigation.log"
    peer.write_text(
        "\n".join(
            line
            for line in peer.read_text(encoding="utf-8").splitlines()
            if "SNAP(VISION-NAV" not in line
        )
        + "\n",
        encoding="utf-8",
    )

    report = analysis.analyze_ideal_run(
        tmp_path,
        approvals=approvals,
        max_snap_distance_m=1.0,
    )

    assert report.passed is False
    owner_report = next(item for item in report.vehicles if item.sys_id == 4)
    peer_report = next(item for item in report.vehicles if item.sys_id == 6)
    assert any("GUIDED not accepted" in error for error in owner_report.errors)
    assert any("navigation SNAP" in error for error in peer_report.errors)
