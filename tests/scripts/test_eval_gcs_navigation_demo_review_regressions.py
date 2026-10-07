"""Focused regressions for the source-driven three-UAV demo evaluator."""

from __future__ import annotations

import ast
import dataclasses
import importlib
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import eval_gcs_navigation_demo as evaluator


ROOT = Path(__file__).resolve().parents[2]


def test_evaluator_direct_script_bootstraps_repository_imports():
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "eval_gcs_navigation_demo.py"),
            "--help",
        ],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout.lower()


class _Clock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now

    def unix(self) -> float:
        return 1_000.0 + self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


def _final_approach_row(
    wall: str,
    *,
    obs: float,
    roll: str,
    issued: str = "True",
    passed: str = "False",
    source: str = "measured",
    generation: int = 4,
    task: int = 1,
    obj: int = 11,
    dt_wall_ms: str = "",
    body_bearing_deg: str = "2.5",
    pitch: str = "-12.0",
    throttle: str = "0.7",
) -> str:
    payload = ";".join(
        (
            f"source={source}",
            f"generation={generation}",
            f"task={task}",
            f"obj={obj}",
            f"obs_ts={obs}",
            f"dt_wall_ms={dt_wall_ms}",
            f"body_bearing_deg={body_bearing_deg}",
            f"cmd_roll={roll}",
            f"cmd_pitch={pitch}" if roll else "cmd_pitch=",
            f"cmd_thr={throttle}" if roll else "cmd_thr=",
            f"issued={issued}",
            f"passed={passed}",
        )
    )
    return f"{wall},EVENT:FINAL_APPROACH_CMD,{payload},,,,,,,,,\n"


def test_confirmation_ws_is_open_before_trigger_and_echoes_exact_round_uid():
    confirmation = importlib.import_module("scripts.eval_gcs_demo_confirmation")

    class Stream:
        entered = False

        def __init__(self) -> None:
            self.events = iter(
                (
                    b'{"type":"telemetry","vehicles":[]}',
                    b'{"type":"task_confirm_request","sys_id":7,"task_id":1,'
                    b'"task_type":"HEAVY","lat":40.0,"lon":44.0,"alt":10.0,'
                    b'"round_uid":"10:1"}',
                    '{"type":"task_confirm_request","sys_id":7,"task_id":1,'
                    '"task_type":"HEAVY","lat":40.0,"lon":44.0,"alt":10.0,'
                    '"round_uid":"10:1"}',
                    b'{"type":"task_confirm_request","sys_id":4,"task_id":1,'
                    b'"task_type":"HEAVY","lat":40.1,"lon":44.1,"alt":10.0,'
                    b'"round_uid":"10:1"}',
                    b'{"type":"task_confirm_request","sys_id":9,"task_id":1,'
                    b'"task_type":"HEAVY","lat":40.2,"lon":44.2,"alt":10.0,'
                    b'"round_uid":"10:1"}',
                )
            )

        def __enter__(self):
            self.entered = True
            return self

        def __exit__(self, *_args):
            self.entered = False

        def receive(self, _timeout_s: float):
            return next(self.events)

    class Api:
        def __init__(self) -> None:
            self.posts: list[tuple[str, dict]] = []

        def post_json(self, path: str, payload: dict, **_kwargs):
            self.posts.append((path, payload))
            return 200, {"status": "approved", "delivery": "sent"}

    stream = Stream()
    api = Api()
    clock = _Clock()
    observed: list[tuple[int, int]] = []

    approvals = confirmation.run_confirmation_workflow(
        event_stream=stream,
        api=api,
        expected_sys_ids={7, 4, 9},
        trigger=lambda: (_ for _ in ()).throw(AssertionError("not subscribed"))
        if not stream.entered
        else None,
        stop_when=lambda: len(api.posts) == 3,
        requested_delay_s=0.25,
        timeout_s=5.0,
        clock=clock,
        on_request_observed=lambda request: observed.append(
            (request.sys_id, request.local_task_id)
        ),
    )

    assert len(approvals) == 3
    assert [payload for _, payload in api.posts] == [
        {
            "sys_id": sys_id,
            "task_id": task_id,
            "is_confirmed": True,
            "action": "approve",
            "round_uid": uid,
        }
        for sys_id, task_id, uid in (
            (7, 1, "10:1"),
            (4, 1, "10:1"),
            (9, 1, "10:1"),
        )
    ]
    assert all(path == "/api/control/task_confirm" for path, _ in api.posts)
    assert observed == [(7, 1), (4, 1), (9, 1)]


def test_confirmation_rejects_legacy_uid_for_certification():
    confirmation = importlib.import_module("scripts.eval_gcs_demo_confirmation")
    with pytest.raises(ValueError, match="legacy"):
        confirmation.parse_task_confirm_event(
            b'{"type":"task_confirm_request","sys_id":7,"task_id":1,'
            b'"task_type":"HEAVY","lat":40.0,"lon":44.0,"alt":10.0,'
            b'"round_uid":"legacy"}'
        )


def test_final_approach_command_rows_are_atomic_ordered_and_retain_unissued(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    path = tmp_path / "navigation_debug.csv"
    path.write_text(
        "ts,event,payload,,,,,,,,,\n"
        + _final_approach_row("12:00:00.000", obs=1.0, roll="10.0")
        + _final_approach_row("12:00:00.010", obs=1.1, roll="", issued="False")
        + _final_approach_row(
            "12:00:00.020", obs=1.2, roll="", issued="False", passed="True"
        ),
        encoding="utf-8",
    )

    records = evidence.parse_final_approach_commands(path)

    assert len(records) == 3
    assert [record.obs_ts for record in records] == [1.0, 1.1, 1.2]
    assert [record.issued for record in records] == [True, False, False]
    assert records[0].source_key == ("measured", 4, 1, 11)
    assert records[1].cmd_roll is None
    assert records[2].passed is True


def test_final_approach_episode_allows_repeated_pass_suppression_before_snap(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    metrics = importlib.import_module("scripts.eval_gcs_demo_metrics")
    path = tmp_path / "navigation_debug.csv"
    path.write_text(
        _final_approach_row("12:00:00.000", obs=1.0, roll="10.0")
        + _final_approach_row(
            "12:00:00.010", obs=1.1, roll="", issued="False", passed="True"
        )
        + _final_approach_row(
            "12:00:00.020", obs=1.2, roll="", issued="False", passed="True"
        )
        + "12:00:00.030,EVENT:SNAP_COMPONENTS,algorithm=VISION-NAV-PN,,,,,,,,,\n",
        encoding="utf-8",
    )

    episodes = evidence.parse_final_approach_command_episodes(path)

    assert len(episodes) == 1
    assert [command.passed for command in episodes[0]] == [False, True, True]
    assert metrics.score_final_approach_commands(
        episodes[0],
        roll_limit_deg=45.0,
        saturation_margin_deg=0.5,
        significant_roll_deg=10.0,
    ).sample_count == 1
    assert metrics.final_approach_timing(episodes[0]).observed_speedup == pytest.approx(
        10.0
    )


def test_pass_suppressed_tail_cannot_mask_slow_issued_timing(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    metrics = importlib.import_module("scripts.eval_gcs_demo_metrics")
    path = tmp_path / "navigation_debug.csv"
    issued = "".join(
        _final_approach_row(
            f"12:00:00.{index * 50:03d}",
            obs=1.0 + index * 0.1,
            roll="10.0",
        )
        for index in range(10)
    )
    passed = "".join(
        _final_approach_row(
            f"12:00:00.{450 + index * 10:03d}",
            obs=1.9 + index * 0.1,
            roll="",
            issued="False",
            passed="True",
        )
        for index in range(1, 31)
    )
    path.write_text(issued + passed, encoding="utf-8")

    timing = metrics.final_approach_timing(evidence.parse_final_approach_commands(path))

    assert timing.observed_speedup == pytest.approx(2.0)
    assert timing.median_wall_gap_s == pytest.approx(0.05)


def test_final_approach_episode_rejects_an_issued_command_after_pass(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    path = tmp_path / "navigation_debug.csv"
    path.write_text(
        _final_approach_row("12:00:00.000", obs=1.0, roll="10.0")
        + _final_approach_row(
            "12:00:00.010", obs=1.1, roll="", issued="False", passed="True"
        )
        + _final_approach_row("12:00:00.020", obs=1.2, roll="11.0"),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="after pass"):
        evidence.parse_final_approach_command_episodes(path)


def test_final_approach_bool_tokens_are_strict(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    path = tmp_path / "navigation_debug.csv"
    path.write_text(
        _final_approach_row("12:00:00.000", obs=1.0, roll="10", issued="1"),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="issued"):
        evidence.parse_final_approach_commands(path)


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"dt_wall_ms": "-1"}, "dt_wall_ms"),
        ({"body_bearing_deg": "181"}, "body_bearing_deg"),
        ({"roll": "181"}, "cmd_roll"),
        ({"pitch": "-91"}, "cmd_pitch"),
        ({"throttle": "-0.1"}, "cmd_thr"),
        ({"throttle": "1.1"}, "cmd_thr"),
    ],
)
def test_final_approach_command_domains_are_strict(tmp_path, overrides, message):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    values = {"obs": 1.0, "roll": "10.0", **overrides}
    path = tmp_path / "navigation_debug.csv"
    path.write_text(_final_approach_row("12:00:00.000", **values), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        evidence.parse_final_approach_commands(path)


def test_source_gap_is_raw_and_observed_speedup_is_a_ratio(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    metrics = importlib.import_module("scripts.eval_gcs_demo_metrics")
    path = tmp_path / "navigation_debug.csv"
    path.write_text(
        _final_approach_row("12:00:00.000", obs=1.0, roll="10")
        + _final_approach_row("12:00:00.010", obs=1.1, roll="11")
        + _final_approach_row(
            "12:00:00.020", obs=1.2, roll="", issued="False", passed="True"
        ),
        encoding="utf-8",
    )
    timing = metrics.final_approach_timing(evidence.parse_final_approach_commands(path))
    assert timing.max_source_gap_s == pytest.approx(0.1)
    assert timing.observed_speedup == pytest.approx(10.0)


def test_saturation_touching_explicit_pass_is_still_scored(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    metrics = importlib.import_module("scripts.eval_gcs_demo_metrics")
    path = tmp_path / "navigation_debug.csv"
    path.write_text(
        "".join(
            _final_approach_row(
                f"12:00:00.0{index}0", obs=1.0 + index / 10, roll="45.0"
            )
            for index in range(3)
        )
        + _final_approach_row(
            "12:00:00.030", obs=1.3, roll="", issued="False", passed="True"
        ),
        encoding="utf-8",
    )
    score = metrics.score_final_approach_commands(
        evidence.parse_final_approach_commands(path),
        roll_limit_deg=45.0,
        saturation_margin_deg=0.5,
        significant_roll_deg=10.0,
    )
    assert score.sample_count == 3
    assert score.max_saturation_run == 3
    assert score.saturation_fraction == 1.0


@pytest.mark.parametrize(
    "values",
    [(), (1, 2), (1, 2, 3, 4), (1, 1, 2), (True, 2, 3), (0, 2, 3), (1, "2", 3)],
)
def test_three_uav_ids_reject_malformed_duplicate_empty_or_extra(values):
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    with pytest.raises((TypeError, ValueError)):
        models.ThreeUavIds.from_values(values)


def test_three_uav_ids_preserve_manifest_slot_order():
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    assert models.ThreeUavIds.from_values((7, 4, 9)).values == (7, 4, 9)


def test_malformed_approval_and_plan_artifacts_fail_explicitly(tmp_path):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
    approval_path = tmp_path / "operator_approvals.json"
    plan_path = tmp_path / "demo_mission_plan.json"
    approval_path.write_text("{not-json", encoding="utf-8")
    plan_path.write_text(json.dumps({"vehicles": []}), encoding="utf-8")
    with pytest.raises(ValueError):
        evidence.load_approval_records(approval_path)
    with pytest.raises(ValueError):
        scenario.load_resolved_plan(plan_path)


def _mission_row(ordinal: int, *, command: int) -> dict[str, float | int]:
    return {
        "lat": 40.0 + ordinal / 10_000,
        "lon": 44.0 + ordinal / 10_000,
        "alt": 100.0,
        "mission_sequence": ordinal + 1,
        "command": command,
        "nav_waypoint_ordinal": ordinal,
    }


def test_resolved_plan_requires_owner_identity_and_command_bearing_nav_rows(tmp_path):
    from pymavlink.dialects.v20.ardupilotmega import (
        MAV_CMD_NAV_LOITER_UNLIM,
        MAV_CMD_NAV_WAYPOINT,
    )

    models = importlib.import_module("scripts.eval_gcs_demo_models")
    scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
    ids = models.ThreeUavIds.from_values((7, 4, 9))
    rows = [_mission_row(ordinal, command=MAV_CMD_NAV_WAYPOINT) for ordinal in range(1, 8)]
    mission = {"sys_id": 7, "waypoints": rows}

    plan = scenario.resolve_demo_plan(ids, mission)

    assert [poi.nav_waypoint_ordinal for poi in plan.pois] == [3, 4, 7]
    assert [poi.lat for poi in plan.pois] == pytest.approx(
        [rows[index - 1]["lat"] for index in (3, 4, 7)]
    )
    path = tmp_path / "resolved-plan.json"
    scenario.persist_resolved_plan(plan, path)
    assert scenario.load_resolved_plan(path) == plan
    serialized = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(serialized["vehicles"], list)
    assert isinstance(serialized["pois"], list)
    with pytest.raises(ValueError, match="response sys_id"):
        scenario.resolve_demo_plan(ids, {**mission, "sys_id": 4})
    bad_rows = list(rows)
    bad_rows[3] = _mission_row(4, command=MAV_CMD_NAV_LOITER_UNLIM)
    with pytest.raises(ValueError, match="MAV_CMD_NAV_WAYPOINT"):
        scenario.resolve_demo_plan(ids, {"sys_id": 7, "waypoints": bad_rows})
    for malformed_sys_id in (True, 7.0):
        with pytest.raises((TypeError, ValueError), match="sys_id"):
            scenario.resolve_demo_plan(
                ids,
                {"sys_id": malformed_sys_id, "waypoints": rows},
            )


def test_confirmation_accepts_repeated_vehicle_local_task_ids(
    monkeypatch,
    tmp_path,
):
    from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

    analysis = importlib.import_module("scripts.eval_gcs_demo_analysis")
    mission = importlib.import_module("scripts.eval_gcs_demo_mission")
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
    ids = models.ThreeUavIds.from_values((7, 4, 9))
    rows = [
        _mission_row(ordinal, command=MAV_CMD_NAV_WAYPOINT)
        for ordinal in range(1, 8)
    ]
    plan = scenario.resolve_demo_plan(ids, {"sys_id": 7, "waypoints": rows})
    requested_pairs = ((7, 1), (4, 1), (9, 1))

    def confirmation_workflow(**kwargs):
        assert kwargs["expected_sys_ids"] == set(ids.values)
        for sys_id, task_id in requested_pairs:
            kwargs["on_request_observed"](
                mission.ConfirmationRequest(sys_id, task_id, f"10:{task_id}")
            )
        return [
            {"sys_id": sys_id, "task_id": task_id}
            for sys_id, task_id in requested_pairs
        ]

    monkeypatch.setattr(mission, "run_confirmation_workflow", confirmation_workflow)
    context = SimpleNamespace(
        sys_ids=ids.values,
        api=SimpleNamespace(websocket_url="ws://example"),
    )

    approvals = mission.approve_requests_until_snap(
        context,
        plan,
        tmp_path,
        30.0,
        0.0,
        event_stream_factory=lambda _url: object(),
    )
    counts, errors = analysis.approval_counts(approvals, plan)

    assert counts == {7: 1, 4: 1, 9: 1}
    assert errors == []


def test_assignment_audit_derives_global_map_from_accepted_owner_requests():
    from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

    assignment = importlib.import_module("scripts.eval_gcs_demo_assignment_audit")
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
    ids = models.ThreeUavIds.from_values((7, 4, 9))
    rows = [
        _mission_row(ordinal, command=MAV_CMD_NAV_WAYPOINT)
        for ordinal in range(1, 8)
    ]
    plan = scenario.resolve_demo_plan(ids, {"sys_id": 7, "waypoints": rows})
    text = "\n".join((
        "INFO Task assign request: sender=7 receiver=4 task_id=1 "
        "at (40.000300, 44.000300)",
        "INFO Task assign request: sender=7 receiver=9 task_id=3 "
        "at (40.000700, 44.000700)",
        "INFO Task assign response: sender=4 receiver=7 task_id=1 accepted=True",
        "INFO Task assign response: sender=9 receiver=7 task_id=3 accepted=True",
    ))

    assert assignment.derive_global_assignment_map(text, plan) == {
        7: 2,
        4: 1,
        9: 3,
    }


def test_assignment_audit_joins_detector_task_ids_to_plan_by_coordinate():
    from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

    assignment = importlib.import_module("scripts.eval_gcs_demo_assignment_audit")
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
    ids = models.ThreeUavIds.from_values((7, 4, 9))
    plan = scenario.resolve_demo_plan(
        ids,
        {
            "sys_id": 7,
            "waypoints": [
                _mission_row(ordinal, command=MAV_CMD_NAV_WAYPOINT)
                for ordinal in range(1, 8)
            ],
        },
    )
    text = "\n".join((
        "INFO Task assign request: sender=7 receiver=4 task_id=3 "
        "at (40.000300, 44.000300)",
        "INFO Task assign request: sender=7 receiver=9 task_id=1 "
        "at (40.000700, 44.000700)",
        "INFO Task assign response: sender=4 receiver=7 task_id=3 accepted=True",
        "INFO Task assign response: sender=9 receiver=7 task_id=1 accepted=True",
    ))

    assert assignment.derive_global_assignment_map(text, plan) == {
        7: 2,
        4: 1,
        9: 3,
    }


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda text: "\n".join(text.splitlines()[:2]), "response"),
        (
            lambda text: text.replace("task_id=3", "task_id=1")
            .replace("40.000700", "40.000300")
            .replace("44.000700", "44.000300"),
            "same auction|same resolved-plan",
        ),
        (lambda text: text.replace("40.000700", "41.000700"), "plan coordinate"),
        (lambda text: text.replace("40.000300", "400.000300"), "plan coordinate"),
        (lambda text: text.replace("accepted=True", "accepted=False", 1), "rejected"),
    ],
)
def test_assignment_audit_rejects_incomplete_or_inconsistent_evidence(
    mutation,
    message,
):
    from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

    assignment = importlib.import_module("scripts.eval_gcs_demo_assignment_audit")
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    scenario = importlib.import_module("scripts.eval_gcs_demo_scenario")
    ids = models.ThreeUavIds.from_values((7, 4, 9))
    plan = scenario.resolve_demo_plan(
        ids,
        {
            "sys_id": 7,
            "waypoints": [
                _mission_row(ordinal, command=MAV_CMD_NAV_WAYPOINT)
                for ordinal in range(1, 8)
            ],
        },
    )
    valid = "\n".join((
        "INFO Task assign request: sender=7 receiver=4 task_id=1 "
        "at (40.000300, 44.000300)",
        "INFO Task assign request: sender=7 receiver=9 task_id=3 "
        "at (40.000700, 44.000700)",
        "INFO Task assign response: sender=4 receiver=7 task_id=1 accepted=True",
        "INFO Task assign response: sender=9 receiver=7 task_id=3 accepted=True",
    ))

    with pytest.raises(ValueError, match=message):
        assignment.derive_global_assignment_map(mutation(valid), plan)


def test_command_bounds_artifact_round_trips_json_arrays(tmp_path):
    bounds = importlib.import_module("scripts.eval_gcs_demo_command_bounds")
    artifact = bounds.DemoCommandBounds(1, tuple(
        bounds.VehicleCommandBounds(sys_id, -25.0, 20.0, 45.0, 0.5)
        for sys_id in (1, 2, 3)
    ))
    path = tmp_path / "command-bounds.json"

    bounds.persist_command_bounds(artifact, path)

    assert bounds.load_command_bounds(path) == artifact
    assert isinstance(json.loads(path.read_text(encoding="utf-8"))["vehicles"], list)


def test_live_run_validates_manifest_before_creating_files_or_launching(
    monkeypatch,
    tmp_path,
):
    runtime = importlib.import_module("scripts.eval_gcs_demo_runtime")
    launched = False

    def launch(*_args, **_kwargs):
        nonlocal launched
        launched = True
        raise AssertionError("stack must not launch")

    monkeypatch.setattr(runtime, "with_owned_stack", launch)
    monkeypatch.setattr(
        runtime,
        "load_scenario_manifest",
        lambda: (_ for _ in ()).throw(ValueError("invalid manifest")),
    )
    run_dir = tmp_path / "not-created"

    with pytest.raises(ValueError, match="invalid manifest"):
        runtime.execute_live_run(run_dir, timeout_s=1.0)

    assert launched is False
    assert not run_dir.exists()


def test_command_bounds_are_captured_from_actual_parameter_snapshot():
    bounds = importlib.import_module("scripts.eval_gcs_demo_command_bounds")
    snapshot = {
        "sys_id": 7,
        "params": [
            {"name": "PTCH_LIM_MIN_DEG", "value": -42.0},
            {"name": "PTCH_LIM_MAX_DEG", "value": 19.0},
            {"name": "AAS_DEL_THR", "value": -1.0},
            {"name": "TRIM_THROTTLE", "value": 55.0},
        ],
    }

    captured = bounds.command_bounds_from_snapshot(
        7,
        snapshot,
        roll_limit_deg=45.0,
    )

    assert captured.pitch_min_deg == -42.0
    assert captured.pitch_max_deg == 19.0
    assert captured.roll_limit_deg == 45.0
    assert captured.throttle == 0.55
    with pytest.raises(ValueError, match="sys_id mismatch"):
        bounds.command_bounds_from_snapshot(4, snapshot, roll_limit_deg=45.0)
    for malformed_sys_id in (True, 7.0):
        with pytest.raises((TypeError, ValueError), match="sys_id"):
            bounds.command_bounds_from_snapshot(
                7,
                {**snapshot, "sys_id": malformed_sys_id},
                roll_limit_deg=45.0,
            )


@pytest.mark.parametrize("bad", [True, float("nan"), float("inf"), -float("inf")])
def test_gate_and_navigation_speedup_reject_bad_numbers_before_io(bad, tmp_path):
    models = importlib.import_module("scripts.eval_gcs_demo_models")
    config = importlib.import_module("scripts.eval_gcs_demo_config")
    with pytest.raises((TypeError, ValueError)):
        models.GateLimits(
            truth=models.TruthLimits(max_snap_distance_m=bad),
        )
    with pytest.raises((TypeError, ValueError)):
        config.build_environment(
            tmp_path / "settings.json",
            tmp_path / "logs",
            navigation_speedup=bad,
            base_environment={"GCS_SIM_NAVIGATION_SPEEDUP": "99"},
        )


def test_gate_limits_are_composed_from_focused_limit_groups():
    models = importlib.import_module("scripts.eval_gcs_demo_models")

    assert len(dataclasses.fields(models.GateLimits)) <= 12
    limits = models.GateLimits()
    assert isinstance(limits.cadence, models.CadenceLimits)
    assert isinstance(limits.roll_quality, models.RollQualityLimits)
    assert isinstance(limits.truth, models.TruthLimits)


def test_zero_and_positive_navigation_speedup_environment_contract(tmp_path):
    config = importlib.import_module("scripts.eval_gcs_demo_config")
    inherited = {"GCS_SIM_NAVIGATION_SPEEDUP": "99", "KEEP": "yes"}
    zero = config.build_environment(
        tmp_path / "settings.json",
        tmp_path / "logs",
        navigation_speedup=0.0,
        base_environment=inherited,
    )
    positive = config.build_environment(
        tmp_path / "settings.json",
        tmp_path / "logs",
        navigation_speedup=2.5,
        base_environment=inherited,
    )
    assert "GCS_SIM_NAVIGATION_SPEEDUP" not in zero
    assert positive["GCS_SIM_NAVIGATION_SPEEDUP"] == "2.5"
    assert zero["NAVPY_POSE_CADENCE_DEBUG"] == str(
        (tmp_path / "logs" / "source-time").resolve()
    )
    assert positive["NAVPY_POSE_CADENCE_DEBUG"] == zero[
        "NAVPY_POSE_CADENCE_DEBUG"
    ]
    assert inherited["GCS_SIM_NAVIGATION_SPEEDUP"] == "99"


def test_navigation_speedup_evidence_requires_ordered_apply_and_restore():
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")
    zero_log = "args nav_sim_speedup=0.0\nSNAP(VISION-NAV): 3d=0.2"
    assert evidence.navigation_speedup_errors(
        zero_log, requested=0.0, launch_speed=10.0
    )
    assert evidence.navigation_speedup_errors(
        "SNAP(VISION-NAV): 3d=0.2",
        requested=0.0,
        launch_speed=10.0,
    ) == []
    positive_log = (
        "args nav_sim_speedup=2.5\n"
        "SIM_SPEEDUP=2.5\n"
        "SIM_SPEEDUP=10.0\n"
        "SNAP(VISION-NAV): 3d=0.2"
    )
    assert evidence.navigation_speedup_errors(
        positive_log, requested=2.5, launch_speed=10.0
    ) == []
    assert evidence.navigation_speedup_errors(
        positive_log.replace("SIM_SPEEDUP=10.0\n", ""),
        requested=2.5,
        launch_speed=10.0,
    )
    assert evidence.navigation_speedup_errors(
        "args nav_sim_speedup=99\n" + positive_log,
        requested=2.5,
        launch_speed=10.0,
    )


@pytest.mark.parametrize(
    "text,requested",
    [
        ("args nav_sim_speedup=bogus\n", 0.0),
        (
            "args nav_sim_speedup=bogus\n"
            "args nav_sim_speedup=2.5\n"
            "SIM_SPEEDUP=2.5\nSIM_SPEEDUP=10\n"
            "SNAP(VISION-NAV): 3d=0.2",
            2.5,
        ),
        (
            "args nav_sim_speedup=2.5junk\n"
            "SIM_SPEEDUP=2.5\nSIM_SPEEDUP=10\n"
            "SNAP(VISION-NAV): 3d=0.2",
            2.5,
        ),
        (
            "args nav_sim_speedup=2.5\n"
            "SIM_SPEEDUP=2.5junk\nSIM_SPEEDUP=10\n"
            "SNAP(VISION-NAV): 3d=0.2",
            2.5,
        ),
    ],
)
def test_navigation_speedup_rejects_every_malformed_marker(text, requested):
    evidence = importlib.import_module("scripts.eval_gcs_demo_evidence")

    assert evidence.navigation_speedup_errors(
        text,
        requested=requested,
        launch_speed=10.0,
    )


def test_wait_until_propagates_non_transient_schema_errors():
    process = importlib.import_module("scripts.eval_gcs_demo_process")
    clock = _Clock()
    calls = 0

    def probe():
        nonlocal calls
        calls += 1
        raise TypeError("bad registry schema")

    with pytest.raises(TypeError, match="bad registry schema"):
        process.wait_until(
            "registry",
            timeout_s=1.0,
            probe=probe,
            transient_exceptions=(OSError,),
            clock=clock,
        )
    assert calls == 1


def test_owned_stack_cleanup_attempts_every_step_and_aggregates(monkeypatch):
    process = importlib.import_module("scripts.eval_gcs_demo_process")
    calls: list[str] = []

    class Api:
        def close(self):
            calls.append("close")
            raise OSError("close failed")

    class Tail:
        def read_new(self):
            calls.append("tail")
            raise OSError("tail failed")

    context = type("Context", (), {"api": Api(), "backend_tail": Tail()})()
    monkeypatch.setattr(process, "launch_stack", lambda *_a, **_k: context)

    def stop(*_args, **_kwargs):
        calls.append("stop")
        raise OSError("stop failed")

    monkeypatch.setattr(process, "stop_stack", stop)
    with pytest.raises(process.RegressionError) as caught:
        process.with_owned_stack({}, 1.0, lambda _context: (_ for _ in ()).throw(RuntimeError("action failed")))
    assert calls == ["close", "stop", "tail"]
    assert all(text in str(caught.value) for text in ("action failed", "close failed", "stop failed", "tail failed"))


def test_facade_exact_reexports_and_all_modules_meet_budgets():
    expected = {
        "scripts.eval_gcs_demo_models": (
            "GateLimits",
            "EpisodeMetrics",
            "VehicleReport",
            "RunReport",
            "MissionArtifacts",
            "RegressionError",
        ),
        "scripts.eval_gcs_demo_config": (
            "build_settings",
            "aas_params_for",
            "full_param_changes",
            "build_environment",
        ),
        "scripts.eval_gcs_demo_analysis": ("analyze_vehicle", "analyze_run"),
        "scripts.eval_gcs_demo_runtime": ("execute_live_run",),
        "scripts.eval_gcs_demo_cli": ("parse_args", "main"),
    }
    for module_name, names in expected.items():
        leaf = importlib.import_module(module_name)
        for name in names:
            assert getattr(evaluator, name) is getattr(leaf, name)

    violations: list[str] = []
    for path in sorted((ROOT / "scripts").glob("eval_gcs_*.py")):
        lines = path.read_text(encoding="utf-8").splitlines()
        if len(lines) > 300:
            violations.append(f"{path.name}: {len(lines)} lines")
        tree = ast.parse("\n".join(lines))
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                size = int(node.end_lineno) - node.lineno + 1
                if size > 150:
                    violations.append(f"{path.name}:{node.name} {size} lines")
            elif isinstance(node, ast.ClassDef):
                size = int(node.end_lineno) - node.lineno + 1
                methods = sum(
                    isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
                    for item in node.body
                )
                if size > 200 or methods > 15:
                    violations.append(
                        f"{path.name}:{node.name} {size} lines/{methods} methods"
                    )
    assert not violations, "\n".join(violations)


def test_evaluator_has_no_magic_poi_mask_regex_or_owner_order_inference():
    text = (ROOT / "scripts" / "eval_gcs_navigation_demo.py").read_text(
        encoding="utf-8"
    )
    assert "OWNER_POI_MASK" not in text
    assert "_OWNER_POI_RE" not in text
    assert "actual_ids[0]" not in text
    assert "actual_ids[:EXPECTED_UAV_COUNT]" not in text
    assert "= 76" not in text
