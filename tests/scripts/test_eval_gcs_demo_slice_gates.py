"""The owner-only slice's journal gates, each on passing and failing evidence (step 9g)."""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.coordinator_mavlink import DIRECTIONS, ROUTER_LINK, parse_key
from navpy.sim.lockstep.coordinator_types import serial0_device, serial1_device

from scripts.eval_gcs_demo_scenario import load_resolved_plan, load_scenario
from scripts.eval_gcs_demo_slice_gates import (
    SLICE_FORBIDDEN_MESSAGES,
    approval_errors,
    assignment_errors,
    forbidden_message_ids,
    is_self_assignment,
    load_slice_evidence,
    pose_cadence_errors,
    run_errors,
    sensor_errors,
    slice_gate_errors,
    snap_errors,
    workflow_tick_errors,
)
from tests.scripts.slice_journal_evidence import slice_approval, write_slice_journal, written_approvals
from tests.scripts.test_eval_gcs_demo_slice_scenario import OWNER
from tests.scripts.test_eval_gcs_demo_slice_scenario import _write_slice_run as _write_wall_run


def _write_slice_run(log_dir: Path) -> None:
    """A wall owner-only run plus the lockstep evidence matching its owner's log."""
    _write_wall_run(log_dir)
    write_slice_journal(log_dir, OWNER)

ROOT = Path(__file__).resolve().parents[2]
SLICE = load_scenario(ROOT / "scripts" / "eval_lockstep_slice_scenario.json")


@pytest.fixture
def evidence(tmp_path):
    _write_slice_run(tmp_path)
    loaded = load_slice_evidence(tmp_path)
    lines = [line for _k, line in loaded.lines]
    return tmp_path, loaded.records, loaded.lines, [slice_approval(OWNER, lines)]


def _counts(records):
    return next(r for r in records if "mavlink_counts" in r)["mavlink_counts"]


def test_passing_evidence_passes_every_gate(evidence):
    log_dir, records, lines, approvals = evidence
    plan = load_resolved_plan(log_dir / "demo_mission_plan.json", SLICE)
    assert slice_gate_errors(log_dir, plan, approvals) == []


def test_a_run_without_lockstep_evidence_fails(tmp_path):
    _write_slice_run(tmp_path)
    (tmp_path / "lockstep_journal.jsonl").unlink()
    plan = load_resolved_plan(tmp_path / "demo_mission_plan.json", SLICE)
    errors = slice_gate_errors(tmp_path, plan, [])
    assert errors == ["slice evidence: the owner-only slice needs the lockstep journal and NavPy "
                      "lines (run it with --lockstep)"]


def test_run_gate_fails_on_a_fault_or_an_outcome_other_than_ok(evidence):
    _log_dir, records, _lines, _approvals = evidence
    assert run_errors(records) == []
    faulted = records[:-1] + [{"fault": "HostFault", "kind": "exit_budget", "tick": 9, "ordinal": 1}]
    assert run_errors(faulted) == ["lockstep fault HostFault exit_budget at tick 9",
                                   "the journal records no run outcome"]
    timeout = records[:-1] + [{**records[-1], "outcome": "timeout"}]
    assert run_errors(timeout) == ["lockstep outcome 'timeout', not 'ok'"]


def test_sensor_gate_needs_the_owners_poi_selection(evidence):
    _log_dir, _records, lines, _approvals = evidence
    assert sensor_errors(lines, 1) == []
    assert sensor_errors(lines, 2) == ["owner never logged 'POI: P2 (tracking'"]


def test_workflow_tick_gate_holds_the_retained_order_in_tick_and_stream_position(evidence):
    _log_dir, _records, lines, _approvals = evidence
    assert workflow_tick_errors(lines) == []
    nav = next(i for i, (_k, line) in enumerate(lines) if "INIT: NAV MODE" in line)
    reset = next(i for i, (_k, line) in enumerate(lines) if "RESET: PASSED POI" in line)
    swapped = list(lines)
    swapped[nav], swapped[reset] = (lines[nav][0], lines[reset][1]), (lines[reset][0], lines[nav][1])
    assert any("out of order" in e for e in workflow_tick_errors(swapped))
    assert workflow_tick_errors([(2, "a"), (1, "b")]) == ["NavPy lines are not in tick order"]
    missing = [(k, line) for k, line in lines if "RESET: PASSED POI" not in line]
    assert workflow_tick_errors(missing) == ["workflow ticks: no reset marker"]


SERIAL0 = serial0_device(OWNER)
LINKS = [(device, direction) for device in (SERIAL0, serial1_device(OWNER), ROUTER_LINK)
         for direction in DIRECTIONS]


def _with(records, device, direction, key):
    changed = copy.deepcopy(records)
    _counts(changed)[device][direction]["messages"][key] = 1
    return changed


@pytest.mark.parametrize("kind", sorted(SLICE_FORBIDDEN_MESSAGES))
def test_assignment_gate_fails_on_every_named_message_on_either_link(evidence, kind):
    """Each forbidden id from the owner's companion to a peer, on every link and direction."""
    _log_dir, records, lines, _approvals = evidence
    assert assignment_errors(records, lines, OWNER) == []
    for message, label in forbidden_message_ids().items():
        if not label.startswith(kind):
            continue
        for device, direction in LINKS:
            changed = _with(records, device, direction, f"{message}:{OWNER}.191>2")
            assert assignment_errors(changed, lines, OWNER) == [
                f"owner-only slice carried 1 {label} from {OWNER}.191 to 2 on {device} {direction}"]


@pytest.mark.parametrize("name", ["SWARM_ACK", "TASK_ASSIGN_RESPONSE", "AVAILABLE_TASK_RESPONSE",
                                  "AVAILABLE_TASK_REQUEST"])
def test_a_self_addressed_forbidden_message_other_than_the_self_assignment_fails(evidence, name):
    """Codex 9g P2-2: only the owner's own TASK_ASSIGN_REQUEST to itself is exempt."""
    _log_dir, records, lines, _approvals = evidence
    message = getattr(mavlink, f"MAVLINK_MSG_ID_{name}")
    changed = _with(records, SERIAL0, "to_sitl", f"{message}:{OWNER}.191>{OWNER}")
    assert assignment_errors(changed, lines, OWNER) == [
        f"owner-only slice carried 1 {forbidden_message_ids()[message]} from {OWNER}.191 to {OWNER} "
        f"on {SERIAL0} to_sitl"]


def test_the_self_assignment_exemption_is_the_owners_companion_only(evidence):
    _log_dir, records, lines, _approvals = evidence
    request = mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST
    assert is_self_assignment(parse_key(f"{request}:{OWNER}.191>{OWNER}"), OWNER)
    # Another vehicle's self-assignment is relayed peer traffic.
    other = _with(records, SERIAL0, "from_sitl", f"{request}:2.191>2")
    assert assignment_errors(other, lines, OWNER) == [
        "owner-only slice carried 1 peer relay task dispatch TASK_ASSIGN_REQUEST from system 2 "
        f"on {SERIAL0} from_sitl"]
    # The owner's autopilot (not its companion) or the ground station: no publication.
    for source in (f"{OWNER}.1", "255.0"):
        changed = _with(records, SERIAL0, "to_sitl", f"{request}:{source}>{OWNER}")
        assert assignment_errors(changed, lines, OWNER) == [
            f"owner-only slice carried 1 task dispatch TASK_ASSIGN_REQUEST from {source} to {OWNER} "
            f"on {SERIAL0} to_sitl"]


@pytest.mark.parametrize("device,direction", LINKS)
def test_any_message_from_a_peer_system_is_peer_relay(evidence, device, direction):
    """Codex 9g P2-2: peer relay is a source that is neither the owner nor the
    ground station, whatever the message (MAVLink routing keeps the source)."""
    _log_dir, records, lines, _approvals = evidence
    changed = _with(records, device, direction, "0:2.1>-")  # a peer autopilot's heartbeat
    assert assignment_errors(changed, lines, OWNER) == [
        f"owner-only slice carried 1 peer relay message 0 from system 2 on {device} {direction}"]


def test_assignment_gate_allows_the_owners_self_assignment_but_not_a_marker(evidence):
    _log_dir, records, lines, _approvals = evidence
    assert _counts(records)[SERIAL0]["to_sitl"]["messages"][f"25106:{OWNER}.191>{OWNER}"] == 1
    marked = lines + [(lines[-1][0], "INFO Rebroadcast task 1")]
    assert any("task rebroadcast" in e for e in assignment_errors(records, marked, OWNER))
    assert assignment_errors([r for r in records if "mavlink_counts" not in r], lines, OWNER) == [
        "the journal records no MAVLink message counts"]


def test_approval_gate_needs_exactly_one_for_p_with_its_timing(evidence):
    _log_dir, records, lines, approvals = evidence
    assert approval_errors(records, lines, approvals, OWNER, 1) == []
    assert approval_errors(records, lines, [], OWNER, 1) == ["expected exactly one approval, found 0"]
    assert approval_errors(records, lines, approvals * 2, OWNER, 1) == [
        "expected exactly one approval, found 2"]
    other = [{**approvals[0], "task_id": 2}]
    assert approval_errors(records, lines, other, OWNER, 1) == [
        f"approval for ({OWNER}, P2), not ({OWNER}, P1)"]
    hasty = [{**approvals[0], "actual_delay_s": approvals[0]["actual_delay_s"] - 0.5}]
    assert any("actual_delay_s" in e for e in approval_errors(records, lines, hasty, OWNER, 1))


def test_approval_gate_replays_the_operators_polling_schedule(evidence):
    """Step 10g-2: a late wake, or a journal without the operator's evidence, fails."""
    _log_dir, records, lines, approvals = evidence
    late = copy.deepcopy(records)
    record = next(r for r in late if r.get("operator", {}).get("sleep"))
    record["operator"]["sleep"][0][1] += 1
    (error,) = approval_errors(late, lines, approvals, OWNER, 1)
    assert error.startswith("approval replay: pass ") and "sleep" in error
    plain = copy.deepcopy(records)
    for r in plain:
        r.get("operator", {}).pop("due_check", None)
    assert approval_errors(plain, lines, approvals, OWNER, 1) == [
        "approval replay: the journal holds no due checks (operator evidence off)"]


def test_snap_gate_needs_the_first_snap_at_the_terminal_tick(evidence):
    _log_dir, records, lines, _approvals = evidence
    assert snap_errors(records, lines) == []
    outcome = records[-1]
    moved = records[:-1] + [{**outcome, "terminal_tick": outcome["terminal_tick"] + 1}]
    assert snap_errors(moved, lines) == [
        f"first SNAP at tick {outcome['terminal_tick']}, mission terminal tick "
        f"{outcome['terminal_tick'] + 1}"]
    no_snap = [(k, line) for k, line in lines if "SNAP(VISION-NAV" not in line]
    assert snap_errors(records, no_snap) == ["no SNAP at a mission terminal tick"]



def test_pose_cadence_gate_needs_producer_named_flushed_csvs(evidence):
    """Codex 9g P2-5: the pose-cadence CSVs are slice artifacts, flushed whole."""
    log_dir, _records, _lines, _approvals = evidence
    assert pose_cadence_errors(log_dir, (OWNER,)) == []
    directory = log_dir / "source-time"
    attitude = directory / f"pose_cadence_attitude_arrival_uav_{OWNER}_navpy.csv"
    text = attitude.read_text(encoding="utf-8")
    attitude.write_text(text[:-4], encoding="utf-8")  # a tail cut mid-row
    assert pose_cadence_errors(log_dir, (OWNER,)) == [
        f"pose cadence: {attitude.name} has no flushed rows"]
    attitude.write_text(text + "1000.04" + chr(10), encoding="utf-8")  # a torn last row
    assert pose_cadence_errors(log_dir, (OWNER,)) == [f"pose cadence: {attitude.name} row 4 is torn"]
    attitude.write_text(text, encoding="utf-8")
    (directory / "pose_cadence_worker_uav_4_pid_77.csv").write_text(text, encoding="utf-8")
    assert pose_cadence_errors(log_dir, (OWNER,)) == [
        "pose cadence: pose_cadence_worker_uav_4_pid_77.csv is not named by producer"]
    (directory / "pose_cadence_worker_uav_4_pid_77.csv").unlink()
    position = directory / f"pose_cadence_position_arrival_uav_{OWNER}_navpy.csv"
    position.unlink()
    assert pose_cadence_errors(log_dir, (OWNER,)) == [f"pose cadence: no {position.name}"]


def _analyze(log_dir: Path, approvals):
    from scripts.eval_gcs_demo_analysis import analyze_run

    return analyze_run(log_dir, scenario=SLICE, sys_ids=(OWNER,), approvals=approvals)


def test_analysis_selects_the_journal_gates_by_recorded_execution_mode(tmp_path):
    """Codex 9g P2-7: a wall owner-only run keeps its pre-9g analysis (no journal
    gates); a journal or a lockstep input manifest selects them."""
    import json

    from scripts.eval_gcs_demo_analysis import LOCKSTEP, WALL, execution_mode
    from scripts.eval_gcs_demo_sitl_root import sha256_file
    from tests.scripts.test_eval_gcs_navigation_demo import _approval_records

    wall = tmp_path / "wall"
    wall.mkdir()
    _write_wall_run(wall)
    assert execution_mode(wall) == WALL
    assert _analyze(wall, _approval_records((OWNER,))).passed is True

    lockstep = tmp_path / "lockstep"
    lockstep.mkdir()
    _write_slice_run(lockstep)
    assert execution_mode(lockstep) == LOCKSTEP
    assert _analyze(lockstep, written_approvals(lockstep, OWNER)).passed is True
    records = [json.loads(row) for row in
               (lockstep / "lockstep_journal.jsonl").read_text(encoding="utf-8").splitlines()]
    records[-1]["outcome"] = "timeout"
    (lockstep / "lockstep_journal.jsonl").write_text(
        "".join(json.dumps(r) + chr(10) for r in records), encoding="utf-8")
    assert "lockstep outcome 'timeout', not 'ok'" in _analyze(
        lockstep, written_approvals(lockstep, OWNER)).errors

    recorded = tmp_path / "recorded"
    recorded.mkdir()
    _write_wall_run(recorded)
    (recorded / "input_manifest.json").write_text(json.dumps({
        "scenario_sha256": sha256_file(SLICE.path), "run_options": {"lockstep": True}}),
        encoding="utf-8")
    assert execution_mode(recorded) == LOCKSTEP
    report = _analyze(recorded, _approval_records((OWNER,)))
    assert report.passed is False
    assert ("slice evidence: the owner-only slice needs the lockstep journal and NavPy lines "
            "(run it with --lockstep)") in report.errors
