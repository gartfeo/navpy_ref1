"""The demo evaluator over an explicit scenario: the certified three-UAV
scenario and the owner-only lockstep slice share one code path."""

from __future__ import annotations

import ast
import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import eval_gcs_demo_analysis as analysis
from scripts import eval_gcs_demo_cli as cli
from scripts import eval_gcs_demo_process as process
from scripts.eval_gcs_demo_analysis import analyze_run, owner_only_assignment_evidence
from scripts.eval_gcs_demo_api import NavigationLogTails
from scripts.eval_gcs_demo_config import aas_params_for, aas_params_for_plan, launch_command
from scripts.eval_gcs_demo_constants import backend_log_path
from scripts.eval_gcs_demo_execution import SystemClock
from scripts.eval_gcs_demo_gates import workflow_errors
from scripts.eval_gcs_demo_inputs import INPUT_MANIFEST, require_recorded_scenario
from scripts.eval_gcs_demo_models import (
    MissionArtifacts,
    RegressionError,
    StackContext,
    UavIds,
)
from scripts.eval_gcs_demo_gate_profiles import PROFILE_SIM_SPEEDUP
from scripts.eval_gcs_demo_role_gates import peer_assignment_marker_errors
from scripts.eval_gcs_demo_scenario import (
    CERTIFIED_SCENARIO_PATH,
    load_resolved_plan,
    load_scenario,
    persist_resolved_plan,
    resolve_demo_plan,
)
from scripts.eval_gcs_demo_stack_identity import flown_sys_ids
from navpy.sim.world.navigation_poi_args import encode_wp_bitmask
from tests.scripts.test_eval_gcs_navigation_demo import _approval_records, _write_episode

ROOT = Path(__file__).resolve().parents[2]
SLICE_SCENARIO_PATH = ROOT / "scripts" / "eval_lockstep_slice_scenario.json"
CERTIFIED = load_scenario(CERTIFIED_SCENARIO_PATH)
SLICE = load_scenario(SLICE_SCENARIO_PATH)
OWNER = 4
ROLLS = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]


def _payload(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_scenario(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / "scenario.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


# --- Scenario files and their gate profiles --------------------------------


def test_both_profiles_load_with_their_shapes():
    assert CERTIFIED.gate_profile == "certified"
    assert [slot.role for slot in CERTIFIED.slots] == ["owner", "peer", "peer"]
    assert CERTIFIED.poi_nav_waypoint_ordinals == (2, 3, 6)
    assert CERTIFIED.path == CERTIFIED_SCENARIO_PATH
    assert SLICE.gate_profile == "owner_only_slice"
    assert [slot.role for slot in SLICE.slots] == ["owner"]
    assert SLICE.poi_nav_waypoint_ordinals == (2,)
    assert len(SLICE.missions) == 1
    # The slice flies the certified owner's mission from the certified home.
    assert SLICE.home == CERTIFIED.home
    assert SLICE.missions[0] == CERTIFIED.missions[CERTIFIED.owner_index]


def test_certified_profile_rejects_a_one_slot_scenario(tmp_path):
    payload = _payload(CERTIFIED_SCENARIO_PATH)
    payload["slots"] = payload["slots"][:1]
    payload["missions"] = payload["missions"][:1]
    with pytest.raises(ValueError, match="certified scenario requires exactly three slots"):
        load_scenario(_write_scenario(tmp_path, payload))


def test_slice_profile_rejects_a_second_slot(tmp_path):
    payload = _payload(SLICE_SCENARIO_PATH)
    payload["slots"].append({"name": "peer-1", "role": "peer"})
    payload["missions"].append(payload["missions"][0])
    with pytest.raises(ValueError, match="requires exactly one slot, the owner"):
        load_scenario(_write_scenario(tmp_path, payload))


def test_slice_profile_rejects_a_second_poi_ordinal(tmp_path):
    payload = _payload(SLICE_SCENARIO_PATH)
    payload["poi_nav_waypoint_ordinals"] = [2, 3]
    with pytest.raises(ValueError, match="exactly one POI NAV waypoint ordinal"):
        load_scenario(_write_scenario(tmp_path, payload))


def test_a_scenario_must_name_its_gate_profile(tmp_path):
    payload = _payload(SLICE_SCENARIO_PATH)
    del payload["gate_profile"]
    with pytest.raises(ValueError):
        load_scenario(_write_scenario(tmp_path, payload))


@pytest.mark.parametrize("profile", ["", "Certified", "slice", None, 1])
def test_an_unknown_gate_profile_is_rejected(tmp_path, profile):
    payload = _payload(SLICE_SCENARIO_PATH)
    payload["gate_profile"] = profile
    with pytest.raises(ValueError, match="gate_profile must be one of"):
        load_scenario(_write_scenario(tmp_path, payload))


# --- Sysids per scenario ---------------------------------------------------


@pytest.mark.parametrize(
    ("values", "count"),
    [((4,), 3), ((4, 5, 6), 1), ((), 0), ((4, 4), 2), ((0,), 1), ((True,), 1), (("4",), 1)],
)
def test_uav_ids_require_the_count_and_unique_positive_ints(values, count):
    with pytest.raises((TypeError, ValueError)):
        UavIds.from_values(values, count)


def test_uav_ids_keep_slot_order_for_any_count():
    assert UavIds.from_values((9,), 1).values == (9,)
    assert UavIds.from_values((7, 4, 9), 3).values == (7, 4, 9)


def test_stack_context_and_mission_artifacts_check_the_scenario_count():
    ports = dict(
        api=None, backend_tail=None, clock=SystemClock(),
        event_stream_factory=lambda _url: None, navigation_tail=lambda _id: None,
    )
    assert StackContext(sys_ids=(4,), scenario=SLICE, **ports).sys_ids == (4,)
    with pytest.raises(ValueError):
        StackContext(sys_ids=(4, 5, 6), scenario=SLICE, **ports)
    with pytest.raises(ValueError):
        StackContext(sys_ids=(4,), scenario=CERTIFIED, **ports)
    assert MissionArtifacts((4,), [], 1).sys_ids == (4,)
    with pytest.raises(ValueError):
        MissionArtifacts((4, 5, 6), [], 1)


def test_the_scenario_flies_the_leading_sysids_of_the_slot_reservation():
    entry = {"sysids": [7, 8, 9]}
    assert flown_sys_ids(entry, SLICE) == (7,)
    assert flown_sys_ids(entry, CERTIFIED) == (7, 8, 9)
    with pytest.raises(ValueError):
        flown_sys_ids({"sysids": [7]}, SLICE)


# --- Plan, parameters and launch -------------------------------------------


def _slice_owner_mission(sys_id: int = OWNER) -> dict:
    from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

    rows = [
        {
            "lat": 40.0 + ordinal / 10_000,
            "lon": 44.0 + ordinal / 10_000,
            "alt": 100.0,
            "mission_sequence": ordinal + 1,
            "command": MAV_CMD_NAV_WAYPOINT,
            "nav_waypoint_ordinal": ordinal,
        }
        for ordinal in range(1, 8)
    ]
    return {"sys_id": sys_id, "waypoints": rows}


def test_slice_plan_round_trips_with_one_vehicle_and_one_poi(tmp_path):
    plan = resolve_demo_plan(UavIds.from_values((OWNER,), 1), _slice_owner_mission(), SLICE)

    assert plan.sys_ids == (OWNER,)
    assert plan.owner.sys_id == OWNER
    assert plan.peers == ()
    assert [(poi.task_id, poi.nav_waypoint_ordinal) for poi in plan.pois] == [(1, 2)]
    path = tmp_path / "demo_mission_plan.json"
    persist_resolved_plan(plan, path, SLICE)
    assert load_resolved_plan(path, SLICE) == plan
    # A slice plan is not a certified plan, and the reverse.
    with pytest.raises(ValueError):
        load_resolved_plan(path, CERTIFIED)


def test_slice_owner_gets_its_one_poi_ordinal():
    params = aas_params_for(SLICE, OWNER, OWNER)
    assert params["targ_wps"] == encode_wp_bitmask([2])
    certified = aas_params_for(CERTIFIED, OWNER, OWNER)
    assert certified["targ_wps"] == encode_wp_bitmask([2, 3, 6]) == 38
    # Everything but the POI mask is the certified owner's profile.
    assert {k: v for k, v in params.items() if k != "targ_wps"} == {
        k: v for k, v in certified.items() if k != "targ_wps"
    }
    plan = resolve_demo_plan(UavIds.from_values((OWNER,), 1), _slice_owner_mission(), SLICE)
    assert aas_params_for_plan(SLICE, OWNER, plan) == params


@pytest.mark.parametrize(("scenario", "count"), [(SLICE, "1"), (CERTIFIED, "3")])
def test_launch_flies_one_vehicle_per_scenario_slot(monkeypatch, tmp_path, scenario, count):
    commands = []
    monkeypatch.setattr(process, "owned_entry", lambda _env: None)

    def run_process(command, _env, *, timeout_s):
        commands.append(command)
        return SimpleNamespace(returncode=1, stdout="", stderr="stopped by test")

    monkeypatch.setattr(process, "run_process", run_process)
    with pytest.raises(RegressionError, match="stopped by test"):
        process.launch_stack(
            {"GCS_LOG_DIR": str(tmp_path / "gcs"), "NAVPY_LOG_DIR": str(tmp_path)},
            1.0,
            clock=SystemClock(),
            scenario=scenario,
        )
    assert commands[0][3:5] == ["-n", count]
    assert commands[0] == launch_command(
        commands[0][commands[0].index("--launch-token") + 1]
        if "--launch-token" in commands[0] else None,
        scenario.home,
        instances=len(scenario.slots),
    )


# --- Analysis refuses a scenario other than the one the run flew -----------


def _record_scenario(run_dir: Path, scenario_path: Path) -> None:
    import hashlib

    digest = hashlib.sha256(scenario_path.read_bytes()).hexdigest()
    (run_dir / INPUT_MANIFEST).write_text(
        json.dumps({"scenario_sha256": digest}), encoding="utf-8",
    )


def test_analysis_refuses_a_scenario_whose_hash_the_run_did_not_record(tmp_path):
    _record_scenario(tmp_path, CERTIFIED_SCENARIO_PATH)

    require_recorded_scenario(tmp_path, CERTIFIED)
    with pytest.raises(RegressionError, match="not the run's recorded scenario"):
        require_recorded_scenario(tmp_path, SLICE)
    # A run directory without a manifest (a synthetic fixture) has nothing to compare.
    require_recorded_scenario(tmp_path / "no-manifest", SLICE)


def test_analyze_only_cli_refuses_a_mismatched_scenario(monkeypatch, tmp_path):
    _record_scenario(tmp_path, CERTIFIED_SCENARIO_PATH)
    monkeypatch.setattr(
        analysis, "load_resolved_plan", lambda *_a, **_k: pytest.fail("must not analyze"),
    )
    with pytest.raises(RegressionError, match="not the run's recorded scenario"):
        cli.main(["--analyze-only", str(tmp_path), "--scenario", str(SLICE_SCENARIO_PATH)])


def test_analyze_run_itself_refuses_a_mismatched_scenario(tmp_path):
    # The run comparison calls analyze_run directly, not through the CLI.
    _write_slice_run(tmp_path)
    _record_scenario(tmp_path, CERTIFIED_SCENARIO_PATH)
    with pytest.raises(RegressionError, match="not the run's recorded scenario"):
        analyze_run(tmp_path, scenario=SLICE, sys_ids=(OWNER,), approvals=_approval_records((OWNER,)))


def test_cli_refuses_an_invalid_scenario_file_before_any_run(monkeypatch, tmp_path):
    payload = _payload(SLICE_SCENARIO_PATH)
    del payload["gate_profile"]
    path = _write_scenario(tmp_path, payload)
    monkeypatch.setattr(
        cli, "execute_live_run", lambda *_a, **_k: pytest.fail("must not launch"),
    )
    with pytest.raises(RegressionError, match="invalid --scenario"):
        cli.main(["--scenario", str(path), "--run-dir", str(tmp_path / "run")])
    assert not (tmp_path / "run").exists()


# --- Gate profiles ----------------------------------------------------------


def _owner_only_text(tmp_path: Path, speedup: float = 1.0) -> str:
    """A passing owner's NavPy log with no dispatch, auction or peer line,
    flown at *speedup* (the slice's lockstep factor 1 by default)."""
    _write_episode(
        tmp_path, OWNER, ROLLS, snap_m=0.4, role="owner",
        wall_step_s=0.1 / speedup, source_step_s=0.1,
    )
    path = tmp_path / f"uav_{OWNER}_navigation.log"
    text = "\n".join(
        line.replace("sim_speed=10.0", f"sim_speed={speedup:.1f}")
        for line in path.read_text(encoding="utf-8").splitlines()
        if "Rebroadcast task" not in line
    ) + "\n"
    path.write_text(text, encoding="utf-8")
    return text


def _errors(text: str, profile: str, role: str = "owner") -> list[str]:
    return workflow_errors(
        text, role=role, approval_count=1, navigation_speedup=0.0, gate_profile=profile,
        launch_speedup=PROFILE_SIM_SPEEDUP.get(profile, 1.0),
    )


def test_slice_passes_an_owner_log_certified_fails_only_for_dispatch(tmp_path):
    text = _owner_only_text(tmp_path)
    (tmp_path / "certified").mkdir()
    certified_text = _owner_only_text(tmp_path / "certified", speedup=10.0)

    assert _errors(text, "owner_only_slice") == []
    assert _errors(certified_text, "certified") == [
        "owner did not dispatch task 2 to peers",
        "owner did not dispatch task 3 to peers",
    ]


@pytest.mark.parametrize(
    "line",
    [
        "Rebroadcast task 2, waiting for 0 peers",
        "Task 2 accepted by 5",
        "Task 2 can be handled by 5 (cost 12.0)",
        "Task 2: request acked by 5",
        "Task 2 declined by 5",
        "Task 1 assigned by owner 4",
        "Peer navigation started: task 2",
        "Holding GUIDED: flying peer task 2",
    ],
)
def test_slice_fails_any_dispatch_auction_or_peer_marker(tmp_path, line):
    text = _owner_only_text(tmp_path) + f"2026-07-10 12:00:04,000 INFO {line}\n"

    errors = _errors(text, "owner_only_slice")

    assert len(errors) == 1 and errors[0].startswith("owner-only slice logged ")


#: Every module that logs task dispatch, the auction or peer participation.
_AUCTION_LOGGERS = tuple(
    ROOT / "src/navpy/core/swarm" / name
    for name in (
        "task_auction_coordinator.py",
        "task_rebroadcast.py",
        "task_auction_lifecycle.py",
        "task_capability.py",
        "task_assign_reply.py",
        "peer_task_priority.py",
    )
)


def _rendered(node: ast.expr) -> str | None:
    """A log message with every interpolation rendered as a number."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return re.sub(r"\{[^}]*\}", "7", node.value)
    if isinstance(node, ast.JoinedStr):
        return "".join(
            part.value if isinstance(part, ast.Constant) else "7" for part in node.values
        )
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left, right = _rendered(node.left), _rendered(node.right)
        return None if left is None or right is None else left + right
    return None


def _auction_log_messages() -> list[str]:
    messages = []
    for path in _AUCTION_LOGGERS:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr in {"debug", "info", "warning", "error"}
                and "logger" in ast.unparse(node.func.value) and node.args
            ):
                messages.append(_rendered(node.args[0]))
            elif isinstance(node, ast.Tuple) and len(node.elts) == 2:
                level, text = node.elts
                if isinstance(level, ast.Constant) and level.value in {"debug", "info", "warning", "error"}:
                    messages.append(_rendered(text))  # the verdict log table
    return messages


def test_every_auction_log_line_is_a_forbidden_slice_marker():
    messages = _auction_log_messages()

    assert len(messages) >= 15
    unrendered = [message for message in messages if message is None]
    assert not unrendered, "a log call whose message the guard cannot render"
    uncovered = [message for message in messages if not peer_assignment_marker_errors(message)]
    assert not uncovered, f"auction log lines no slice marker forbids: {uncovered}"


def test_slice_keeps_the_shared_workflow_gates(tmp_path):
    text = _owner_only_text(tmp_path)

    untracked = text.replace("start_tracking obj_id=", "started obj_id=")
    assert "missing gimbal detection lock marker ('start_tracking obj_id=')" in _errors(
        untracked, "owner_only_slice"
    )
    ideal = text + "2026-07-10 12:00:04,000 INFO static ideal 360 enabled\n"
    assert any("ideal_360" in error for error in _errors(ideal, "owner_only_slice"))
    ideal_args = text.replace("vision_profile='siyi_zr10'", "vision_profile='ideal_360'")
    assert any("ideal_360" in error for error in _errors(ideal_args, "owner_only_slice"))


def test_slice_has_no_peer_role(tmp_path):
    text = _owner_only_text(tmp_path)
    assert "the owner-only slice has no peer role" in _errors(text, "owner_only_slice", "peer")


def test_unknown_gate_profile_is_refused_by_the_gates(tmp_path):
    with pytest.raises(ValueError, match="invalid gate profile"):
        _errors(_owner_only_text(tmp_path), "owner_only")


# --- Slice run analysis end to end -----------------------------------------


def _write_slice_run(
    log_dir: Path, backend_lines: tuple[str, ...] = (), speedup: float = 1.0,
) -> None:
    _owner_only_text(log_dir, speedup)
    (log_dir / "demo_mission_plan.json").write_text(json.dumps({
        "schema_version": 1,
        "manifest_schema_version": 2,
        "owner_slot": "owner",
        "vehicles": [{"slot": "owner", "role": "owner", "sys_id": OWNER}],
        "pois": [{"task_id": 1, "nav_waypoint_ordinal": 2, "lat": 40.1, "lon": 44.1}],
    }), encoding="utf-8")
    (log_dir / "demo_command_bounds.json").write_text(json.dumps({
        "schema_version": 1,
        "vehicles": [{
            "sys_id": OWNER, "pitch_min_deg": -40.0, "pitch_max_deg": 20.0,
            "roll_limit_deg": 45.0, "throttle": 0.55,
        }],
    }), encoding="utf-8")
    backend = backend_log_path(log_dir)
    backend.parent.mkdir(parents=True, exist_ok=True)
    backend.write_text("".join(f"{line}\n" for line in backend_lines), encoding="utf-8")


def test_slice_run_passes_with_the_owner_assigned_its_one_poi(tmp_path):
    _write_slice_run(tmp_path)

    report = analyze_run(
        tmp_path, scenario=SLICE, sys_ids=(OWNER,), approvals=_approval_records((OWNER,)),
    )

    assert report.passed is True, report
    assert [(vehicle.sys_id, vehicle.role) for vehicle in report.vehicles] == [(OWNER, "owner")]
    plan = load_resolved_plan(tmp_path / "demo_mission_plan.json", SLICE)
    assert owner_only_assignment_evidence(tmp_path, plan).assignments == {OWNER: 1}


def test_slice_run_fails_when_the_backend_logged_a_task_assignment(tmp_path):
    _write_slice_run(tmp_path, (
        "INFO Task assign request: sender=4 receiver=5 task_id=1 at (40.100000, 44.100000)",
    ))
    plan = load_resolved_plan(tmp_path / "demo_mission_plan.json", SLICE)

    evidence = owner_only_assignment_evidence(tmp_path, plan)
    report = analyze_run(
        tmp_path, scenario=SLICE, sys_ids=(OWNER,), approvals=_approval_records((OWNER,)),
    )

    assert evidence.errors == ["owner-only slice backend log holds 1 task-assignment lines"]
    assert report.passed is False


def test_navigation_tails_read_a_log_that_appears_later(tmp_path):
    tails = NavigationLogTails(tmp_path)
    tail = tails(OWNER)
    tail.read_new()
    assert tail.text == ""
    (tmp_path / f"uav_{OWNER}_navigation.log").write_text("SNAP(VISION-NAV\n", encoding="utf-8")
    assert tails(OWNER) is tail
    tail.read_new()
    assert "SNAP(VISION-NAV" in tail.text


def _slice_report(log_dir: Path):
    return analyze_run(
        log_dir, scenario=SLICE, sys_ids=(OWNER,), approvals=_approval_records((OWNER,)),
    )


def _report_errors(report) -> str:
    return "\n".join([*report.errors, *(e for v in report.vehicles for e in v.errors)])


def test_slice_cadence_expects_the_lockstep_factor(tmp_path):
    _write_slice_run(tmp_path, speedup=10.0)

    report = _slice_report(tmp_path)

    assert report.sim_speedup == 1.0
    assert report.passed is False
    assert "speedup 10.000x outside" in _report_errors(report)


def _sub(file_name: str, pattern: str, replacement: str):
    """A mutation of one run file: every match of *pattern* replaced."""
    def mutate(log_dir: Path) -> None:
        path = log_dir / file_name
        text, count = re.subn(pattern, replacement, path.read_text(encoding="utf-8"), flags=re.M)
        assert count, (file_name, pattern)
        path.write_text(text, encoding="utf-8")
    return mutate


def _all(*mutations):
    def mutate(log_dir: Path) -> None:
        for mutation in mutations:
            mutation(log_dir)
    return mutate


_LOG = f"uav_{OWNER}_navigation.log"
_COMPACT = f"uav_{OWNER}_navigation_compact.csv"
_DEBUG = f"uav_{OWNER}_navigation_debug.csv"
_META = f"uav_{OWNER}_confirmation_t1_120001_meta.json"

#: One mutation per certified owner gate the slice keeps, with the reason it
#: must fail for.
_RETAINED_GATE_MUTATIONS = {
    "requested vision profile": (
        _sub(_LOG, r"vision_profile='siyi_zr10'", "vision_profile='ideal_360'"),
        "simulator-only sensor upper bound",
    ),
    "resolved vision profile": (
        _sub(_LOG, r"Using vision profile 'siyi_zr10'", "Using vision profile 'ideal_360'"),
        "resolved vision profile 'ideal_360'",
    ),
    "mount configuration": (
        _sub(_LOG, r"pitch=-14\.0, fixed=False", "pitch=-14.0, fixed=True"),
        "resolved mount fixed=True != False",
    ),
    "missing self-detect orbit": (
        _sub(_LOG, r"^Self-detect orbit:.*\n", ""),
        "owner did not keep a POI-centred self-detect orbit",
    ),
    "workflow order": (
        _all(
            _sub(_LOG, r"^.*INFO INIT: NAV MODE\n", ""),
            _sub(_LOG, r"(INFO CONFIRMING: P1\n)", r"\g<1>2026-07-10 12:00:01,005 INFO INIT: NAV MODE\n"),
        ),
        "required order is ground-station-confirmed -> INIT NAV",
    ),
    "captured command bounds": (
        _sub("demo_command_bounds.json", r'"roll_limit_deg": 45\.0', '"roll_limit_deg": 8.0'),
        "roll command exceeds configured ROLL_LIMIT_DEG",
    ),
    "forbidden confirmation path": (
        _sub(_LOG, r"INFO POI 1 confirmed by ground station\.", "INFO POI 1 auto-confirmed"),
        "forbidden auto confirmation marker",
    ),
    "tracking lost in review": (
        _sub(
            _LOG, r"(INFO Sending confirm request for P1\.\n)",
            r"\g<1>2026-07-10 12:00:01,015 INFO gimbal recentred\n",
        ),
        "gimbal tracking was not retained during operator review",
    ),
    "confirmation image": (
        _sub(_META, r'"bbox_cxcywh": \[320\.0, 240\.0, 50\.0', '"bbox_cxcywh": [320.0, 240.0, 0.0'),
        "invalid confirmation bbox size",
    ),
    "torn command identity": (
        _sub(_DEBUG, r"task=1;obj=11;obs_ts=1000\.000", "task=2;obj=11;obs_ts=1000.000"),
        "changed atomic source identity",
    ),
    "consistent wrong command identity": (
        _sub(_DEBUG, r"task=1;", "task=2;"),
        "command identity [(2, 11)] does not match selected P1/obj11",
    ),
    "wrong POI coordinate": (
        _sub("demo_mission_plan.json", r'"lat": 40\.1,', '"lat": 40.2,'),
        "differs from assigned coordinate",
    ),
    "missing SNAP component row": (
        _sub(_DEBUG, r"EVENT:SNAP_COMPONENTS", "EVENT:SNAP_COMPONENTS_X"),
        "did not terminate at SNAP_COMPONENTS",
    ),
    "nonfinite SNAP": (
        _sub(_LOG, r"3d=0\.4$", "3d=nan"),
        "truth SNAP distance is non-finite",
    ),
    "cross-file SNAP disagreement": (
        _sub(_COMPACT, r"3d=0\.4\(", "3d=0.9("),
        "compact/debug SNAP disagreement",
    ),
    "insufficient observations": (
        _sub(_DEBUG, r"^.*obs_ts=1000\.[1-9]00.*\n", ""),
        "observation count 3 < 10",
    ),
    "observation gap": (
        _sub(_DEBUG, r"^.*obs_ts=1000\.[3-8]00.*\n", ""),
        "max observation gap 0.700s > 0.500s",
    ),
    "roll quality": (
        _all(*(
            _sub(_DEBUG, rf"cmd_roll={old}\.00;", f"cmd_roll={new}.00;")
            for old, new in (("11", "-15"), ("9", "15"), ("7", "-15"))
        )),
        "significant roll reversals 3 > 1",
    ),
}


@pytest.mark.parametrize("gate", sorted(_RETAINED_GATE_MUTATIONS))
def test_slice_fails_on_each_retained_certified_gate(tmp_path, gate):
    mutate, reason = _RETAINED_GATE_MUTATIONS[gate]
    _write_slice_run(tmp_path)
    assert _slice_report(tmp_path).passed is True

    mutate(tmp_path)

    report = _slice_report(tmp_path)
    assert report.passed is False
    assert reason in _report_errors(report)


@pytest.mark.parametrize(
    ("approved_ids", "reason"),
    [
        ((), "expected exactly one audited approval, found 0"),
        ((5,), "approval artifact contains unknown sysids [5]"),
    ],
)
def test_slice_fails_without_the_owners_one_approval(tmp_path, approved_ids, reason):
    _write_slice_run(tmp_path)

    report = analyze_run(
        tmp_path, scenario=SLICE, sys_ids=(OWNER,), approvals=_approval_records(approved_ids),
    )

    assert report.passed is False
    assert reason in _report_errors(report)


def test_slice_refuses_a_navigation_speedup_override(tmp_path):
    _write_slice_run(tmp_path, speedup=10.0)

    with pytest.raises(ValueError, match="no navigation_speedup override"):
        analyze_run(
            tmp_path, scenario=SLICE, sys_ids=(OWNER,),
            approvals=_approval_records((OWNER,)), navigation_speedup=10.0,
        )


@pytest.mark.parametrize(
    "line",
    [
        "Task 1 offered by owner 5; waiting for its confirmation.",
        "Task 1 from owner 5 rejected: busy.",
        "Busy (holding a task or flying a final approach); not bidding.",
    ],
)
def test_slice_fails_on_participation_in_another_owners_auction(tmp_path, line):
    _write_slice_run(tmp_path)
    with (tmp_path / _LOG).open("a", encoding="utf-8") as log:
        log.write(f"2026-07-10 12:00:03,100 INFO {line}\n")

    report = _slice_report(tmp_path)

    assert report.passed is False
