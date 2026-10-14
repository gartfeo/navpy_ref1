"""The 9d coordinator end to end: toy participants as real processes over NVPART01.

A hub (backend-like, spawned by the coordinator, admitted at the boot
barrier) and a rover (NavPy-like, spawned from the hub's spawn request in a
``DONE``, admitted the tick after) run against a scripted SITL that parks
like the relay. The operator echoes, spawns, cancels a pending request and
stops the rover at SNAP; drain then ends the hub.
"""

from __future__ import annotations

import dataclasses
import json

import pytest

from gcs.backend.instance_registry_runtime import pid_alive
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault, WallBudgets
from tests.modules.lockstep.coordinator_doubles import HUB, ROVER, ScriptedSitl, build, manifest


def _run(tmp_path, name, **kwargs):
    coordinator, sitl, operator, roster = build(tmp_path / name, **kwargs)
    result = coordinator.run()
    return coordinator, sitl, operator, roster, result


def _lifecycle(coordinator):
    return [(r["k"], e["kind"], e["ordinal"], e["result"], e["payload"])
            for r in coordinator.journal.records if "lifecycle" in r for e in r["lifecycle"]]


def test_two_runs_and_a_delayed_run_give_the_same_journal(tmp_path):
    plain = _run(tmp_path, "a")
    again = _run(tmp_path, "b")
    delayed = _run(tmp_path, "c", delay_seed=7, plan=manifest(
        hub={"exit_delay_s": 0.3}, rover={"exit_delay_s": 0.3}))
    assert plain[0].journal.digest == again[0].journal.digest == delayed[0].journal.digest
    assert plain[0].journal.records == delayed[0].journal.records
    assert plain[4] == delayed[4]
    assert plain[1].snapshot_rows == delayed[1].snapshot_rows  # the SITL artifact, byte for byte
    assert plain[4].outcome == "ok"
    # Two seeds' capabilities differ, but nothing else in a toy run depends on it.
    coordinator, sitl, _operator, roster, result = plain
    assert result.parked_tick == result.final_tick + 1
    assert sitl.snapshot_rows[-1][0] == result.parked_tick and sitl.stopped
    assert [m.ordinal for m in roster.members] == [HUB, ROVER]
    assert all(m.flushed and m.exited for m in roster.members)
    coordinator.journal.write(tmp_path / "journal.jsonl")
    delayed[0].journal.write(tmp_path / "delayed.jsonl")
    assert (tmp_path / "journal.jsonl").read_bytes() == (tmp_path / "delayed.jsonl").read_bytes()
    assert (tmp_path / "journal_digest.txt").read_text() == coordinator.journal.digest + "\n"
    host = (tmp_path / "a" / "lockstep_participants_host.json").read_text()
    assert '"exit_code": 0' in host and "pid" not in json.dumps(coordinator.journal.records)


def test_the_late_joiner_is_admitted_the_tick_after_its_spawn(tmp_path):
    coordinator, _sitl, operator, _roster, _result = _run(tmp_path, "run")
    spawn = next(k for k, kind, ordinal, result, _ in _lifecycle(coordinator) if kind == "spawn")
    ticks = [r for r in coordinator.journal.records if "members" in r]
    first = next(r["k"] for r in ticks if any(m["ordinal"] == ROVER for m in r["members"]))
    assert first == spawn + 1
    before = [r for r in ticks if r["phase"] != "boot" and r["k"] < first]
    assert before and all(r["discarded"].get("lockstep:serial0") for r in before)
    assert all("lockstep:serial1" not in r["discarded"] for r in before)
    boot = [r for r in ticks if r["phase"] == "boot"]
    assert boot and all(r["members"] == [] for r in boot)
    assert operator.answers[operator.slow_id] == {"cancelled": True}  # pending over ticks, then cancelled


def test_readiness_comes_ticks_after_registration(tmp_path):
    coordinator, _sitl, operator, _roster, _result = _run(tmp_path, "run")
    ticks = [r for r in coordinator.journal.records if r.get("phase") == "mission"]
    admitted = ticks[0]["k"]
    assert [m["ordinal"] for m in ticks[0]["members"]] == [HUB]
    ready = next(k for k, kind, ordinal, *_ in _lifecycle(coordinator) if kind == "ready" and ordinal == HUB)
    assert ready - admitted >= 3  # registration, then bootstrap ticks, then readiness
    first_request = next(entry[0] for entry in operator.trace if entry[1] == "request")
    assert first_request == ready  # not ready (no request) until the record arrives


def test_a_duplicate_stop_gets_the_same_result_and_a_stale_one_an_error(tmp_path):
    _coordinator, _sitl, operator, _roster, _result = _run(tmp_path, "run", extra_stops=(1, 7))
    assert [operator.answers[k] for k in operator.stop_ids] == [
        {"result": "acked"}, {"result": "acked"}, {"result": "error"}]


def test_a_cleanup_failure_is_a_missed_stop_and_a_host_fault(tmp_path):
    coordinator, *_ = build(tmp_path, plan=manifest(rover={"fail_flush": True}))
    with pytest.raises(HostFault, match="without a flushed end"):
        coordinator.run()
    assert ("stop_result", "missed") in [(kind, result) for _k, kind, _o, result, _p in _lifecycle(coordinator)]


def test_a_nonzero_exit_after_a_flushed_end_is_a_host_fault(tmp_path):
    coordinator, *_rest, roster = build(tmp_path, plan=manifest(rover={"exit_code": 2}))
    with pytest.raises(HostFault, match="exited nonzero .* after a flushed end"):
        coordinator.run()
    assert not any(pid_alive(m.child.pid) for m in roster.members)


def test_a_missed_exit_budget_is_a_host_fault(tmp_path):
    coordinator, *_rest, roster = build(
        tmp_path, plan=manifest(rover={"exit_delay_s": 5.0}),
        budgets=WallBudgets(register_s=60, done_s=60, exit_s=0.5))
    with pytest.raises(HostFault, match="did not exit"):
        coordinator.run()
    assert not any(pid_alive(m.child.pid) for m in roster.members)


def test_an_exit_without_an_ending_done_is_a_coordinator_fault(tmp_path):
    coordinator, *_ = build(tmp_path, plan=manifest(rover={"die_after_s": 1.3}))
    with pytest.raises(CoordinatorFault, match="EOF at tick .* without an ending DONE"):
        coordinator.run()
    assert coordinator.journal.records[-1]["fault"] == "CoordinatorFault"


def test_a_startup_failure_commits_exited_failed(tmp_path):
    coordinator, *_ = build(tmp_path, plan=manifest(rover={"crash_before_ready": True}))
    with pytest.raises(Exception, match="RuntimeError:tick"):
        coordinator.run()
    exited = [p for _k, kind, ordinal, _r, p in _lifecycle(coordinator) if kind == "exited"]
    assert exited == [{"failed": True}]


def test_a_stop_requested_during_drain_is_missed_and_a_fault(tmp_path):
    coordinator, *_ = build(tmp_path, plan=manifest(hub={"stop_on_end": True}))
    with pytest.raises(HostFault, match="during drain"):
        coordinator.run()
    drained = [r for r in coordinator.journal.records if r.get("phase") == "drain"]
    assert [(e["kind"], e["result"]) for e in drained[0]["lifecycle"]] == [("stop_result", "missed")]


def test_the_drain_budget_is_a_fault_and_stops_by_ownership(tmp_path):
    coordinator, sitl, _operator, roster = build(
        tmp_path, plan=manifest(hub={"ignore_end": True}, drain_period_us=200_000))
    with pytest.raises(CoordinatorFault, match="drain passed its budget"):
        coordinator.run()
    assert sitl.stopped and sitl.snapshot_rows is None
    assert not any(pid_alive(m.child.pid) for m in roster.members)


def test_a_stop_budget_miss_is_missed_and_a_fault(tmp_path):
    coordinator, *_ = build(tmp_path, plan=manifest(rover={"ignore_stop": True}))
    with pytest.raises(HostFault, match="missed its stop budget"):
        coordinator.run()
    assert ("stop_result", "missed") in [(kind, result) for _k, kind, _o, result, _p in _lifecycle(coordinator)]


def test_the_boot_barrier_frame_is_still_boot_and_admission_follows_it(tmp_path):
    coordinator, sitl, *_ = _run(tmp_path, "run")
    ticks = [r for r in coordinator.journal.records if "members" in r]
    boot = [r["k"] for r in ticks if r["phase"] == "boot"]
    assert boot == [1, 2, 3]  # frame 3 is the first with helpers_quiet (ScriptedSitl quiet_from=3)
    mission = next(r for r in ticks if r["phase"] == "mission")
    assert mission["k"] == 4 and [m["ordinal"] for m in mission["members"]] == [HUB]


def test_an_operator_returning_before_snap_is_operator_returned_not_ok(tmp_path):
    coordinator, *_ = build(tmp_path, return_after_spawn=True)
    result = coordinator.run()
    assert result.outcome == "operator_returned"
    assert {"operator_returned": result.terminal_tick} in coordinator.journal.records
    assert coordinator.journal.records[-1]["outcome"] == "operator_returned"


def test_an_operator_returning_before_snap_ends_the_mission_as_operator_returned(tmp_path):
    # Step 9g: an operator whose own sim-time workflow ended without SNAP (it
    # stops its companion, then returns) ends the mission at that tick; the
    # run drains and snapshots cleanly, and the outcome is never ok.
    plan = manifest(rover={"snap_at_s": 1e9}, mission_budget_us=60_000_000)
    coordinator, sitl, operator, roster = build(tmp_path, plan=plan, give_up_at_us=1_680_000)
    result = coordinator.run()
    assert result.outcome == "operator_returned" and operator.snap_tick is None
    assert {"operator_returned": result.terminal_tick} in coordinator.journal.records
    assert result.final_tick == result.terminal_tick
    assert sitl.snapshot_rows[-1][0] == result.parked_tick
    assert all(m.flushed and m.exited for m in roster.members)


def test_a_companion_still_running_at_drain_is_a_fault(tmp_path):
    plan = dataclasses.replace(manifest(), companion_roles=("rover", "hub"))
    coordinator, *_ = build(tmp_path, plan=plan)
    with pytest.raises(HostFault, match="not stopped through the participant stop"):
        coordinator.run()
    assert coordinator.journal.records[-1]["ordinal"] == HUB


def test_a_mission_timeout_completes_finalization_and_drain(tmp_path):
    # Mission starts at frame 4 (t = 1.08 s); the 0.6 s budget times out at 1.68 s.
    plan = manifest(rover={"snap_at_s": 1e9}, mission_budget_us=600_000)
    coordinator, sitl, operator, roster = build(tmp_path, plan=plan, give_up_at_us=1_680_000)
    result = coordinator.run()
    assert result.outcome == "timeout" and operator.snap_tick is None
    assert {"timeout": "mission sim-time budget", "k": result.terminal_tick} in coordinator.journal.records
    assert sitl.snapshot_rows[-1][0] == result.parked_tick
    assert all(m.flushed and m.exited for m in roster.members)


def test_ambient_settings_never_reach_a_participant(tmp_path, monkeypatch):
    monkeypatch.setenv("NAVPY_POSE_CADENCE_DEBUG", str(tmp_path / "pose"))
    monkeypatch.setenv("PYTHONHASHSEED", "random")
    names = ["NAVPY_POSE_CADENCE_DEBUG", "PYTHONHASHSEED"]
    coordinator, _sitl, operator, _roster = build(tmp_path, plan=manifest(hub={"report_env": names}))
    coordinator.run()
    assert "env NAVPY_POSE_CADENCE_DEBUG=None" in operator.log_lines
    assert "env PYTHONHASHSEED=0" in operator.log_lines


def _parked_tick(tmp_path) -> int:
    return _run(tmp_path, "plain")[4].parked_tick


def test_the_parked_frame_gets_the_out_of_loop_check(tmp_path):
    parked = _parked_tick(tmp_path)
    sitl = ScriptedSitl(advances=lambda k: 1 if k == parked else 0)
    coordinator, *_ = build(tmp_path / "advance", sitl=sitl)
    with pytest.raises(HostFault, match="out-of-loop"):
        coordinator.run()
    assert coordinator.journal.records[-1] == {
        "fault": "HostFault", "kind": "out_of_loop_advance", "tick": parked, "ordinal": None}


@pytest.mark.parametrize("step_us", [0, 30_000])
def test_the_parked_frame_gets_the_grid_check(tmp_path, step_us):
    parked = _parked_tick(tmp_path)
    source = (lambda k: 1_000_000 + k * 20_000 if k < parked
              else 1_000_000 + (parked - 1) * 20_000 + step_us)
    coordinator, *_ = build(tmp_path / "grid", sitl=ScriptedSitl(source_us=source))
    with pytest.raises(CoordinatorFault, match=f"grid check at tick {parked}"):
        coordinator.run()


def test_a_failing_fault_sidecar_neither_skips_teardown_nor_replaces_the_fault(tmp_path, monkeypatch):
    import pathlib

    original = pathlib.Path.write_text

    def failing_write(self, *args, **kwargs):
        if self.name == "lockstep_fault_host.json":
            raise OSError("host disk full")
        return original(self, *args, **kwargs)

    monkeypatch.setattr(pathlib.Path, "write_text", failing_write)
    coordinator, sitl, _operator, roster = build(tmp_path, plan=manifest(rover={"exit_code": 2}))
    with pytest.raises(HostFault, match="exited nonzero"):
        coordinator.run()
    assert sitl.stopped and not any(pid_alive(m.child.pid) for m in roster.members)
    assert coordinator.journal.records[-1]["kind"] == "exit_nonzero"


def test_participant_fault_text_never_reaches_the_journal(tmp_path):
    digests = []
    for name in ("one", "two"):
        message = f"cannot open {tmp_path / name / 'model.bin'}"
        plan = manifest(rover={"crash_before_ready": True, "crash_message": message})
        coordinator, *_ = build(tmp_path / name, plan=plan)
        with pytest.raises(Exception, match="RuntimeError:tick"):
            coordinator.run()
        digests.append(coordinator.journal.digest)
        logs = "".join(p.read_text(errors="replace") for p in (tmp_path / name / "host").glob("participant_1_*"))
        assert message in logs  # the text is in the host-side log only
    assert digests[0] == digests[1]


def test_a_backend_snap_line_does_not_end_the_mission(tmp_path):
    plan = manifest(hub={"snap_at_s": 1.2}, rover={"snap_at_s": 1e9}, mission_budget_us=600_000)
    coordinator, _sitl, _operator, _roster = build(tmp_path, plan=plan, give_up_at_us=1_680_000)
    result = coordinator.run()
    assert result.outcome == "timeout"  # the hub's marker was not the mission's end


def _peer_response() -> bytes:
    """A forbidden message from a peer (system 2): a TASK_ASSIGN_RESPONSE to the owner."""
    from navpy.sim.lockstep.mavlink_wire import encoder

    link, sink = encoder(2, 191)
    link.task_assign_response_send(0, 0, 0, 0, 1, 1, 1)
    return sink.drain()


def _forbidden_counts(coordinator) -> dict:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    from navpy.sim.lockstep.coordinator_mavlink import message_id

    (counts,) = [r["mavlink_counts"] for r in coordinator.journal.records if "mavlink_counts" in r]
    return {(device, direction): n
            for device, directions in counts.items() for direction, tally in directions.items()
            for key, n in tally["messages"].items()
            if message_id(key) == mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE}


def test_a_message_completed_in_the_parked_frame_is_counted(tmp_path):
    """Codex 9g P2-3: the parked frame k_F+1 is never replied to, but its egress is tallied."""
    from scripts.eval_gcs_demo_slice_gates import assignment_errors

    parked = _parked_tick(tmp_path)
    packet = _peer_response()
    sitl = ScriptedSitl(extra_egress=lambda k: packet if k == parked else b"")
    coordinator, *_ = build(tmp_path / "parked", sitl=sitl)
    coordinator.run()
    assert _forbidden_counts(coordinator) == {("lockstep:serial0", "from_sitl"): 1}
    (parked_record,) = [r["parked"] for r in coordinator.journal.records if "parked" in r]
    assert parked_record["tick"] == parked
    assert any("TASK_ASSIGN_RESPONSE" in e for e in assignment_errors(coordinator.journal.records, [], 1))


def test_a_message_emitted_during_drain_is_counted(tmp_path):
    """Codex 9g P2-3: drain egress is never delivered, but it is tallied (``drain_to_sitl``)."""
    from scripts.eval_gcs_demo_slice_gates import assignment_errors

    plan = manifest(hub={"end_egress_hex": _peer_response().hex()})
    coordinator, *_ = _run(tmp_path, "drain", plan=plan)
    assert _forbidden_counts(coordinator) == {("lockstep:serial1", "drain_to_sitl"): 1}
    drain = [r for r in coordinator.journal.records if r.get("phase") == "drain" and r["drain_egress"]]
    assert drain and drain[0]["navlink"]["lockstep:serial1"]["drain_to_sitl"]
    assert any("TASK_ASSIGN_RESPONSE" in e for e in assignment_errors(coordinator.journal.records, [], 1))
