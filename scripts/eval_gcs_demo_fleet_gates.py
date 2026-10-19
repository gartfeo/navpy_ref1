"""The certified lockstep run's journal gates (``lockstep_fleet.md``, Journal gates).

They read the lockstep run's evidence in the run directory, after the
snapshot like every gate: the hash-chained journal
(``lockstep_journal.jsonl``) and each NavPy's log lines with their ticks
(``lockstep_navpy<i>_lines.jsonl``, ``[tick, line]``, in stream order).
Sysid i is SITL i and NavPy ordinal i (``FleetLayout``).

| Gate | Pass |
|---|---|
| Run | outcome ``ok``, no fault record (``fleet_clock``, ``boot_barrier``, ``exchange_budget`` and an out-of-loop advance are faults); the packet dispositions (``eval_gcs_demo_fleet_packets``) |
| Assignment and forwarding | ``eval_gcs_demo_fleet_packets.forwarding_errors``, without the advert check when the input manifest's profile sets ``SIM_UART_LOSS`` above 0; R2 joined to the journal: every APPLIED line of the backend log is an APPLIED the owner's NavPy emitted of a response its helper emitted for that task (UIDs equal), and every held task has one |
| Approval timing | the polling-schedule replay over every approval (``eval_gcs_demo_approval_replay``); per approval, its k1 <= k2 <= k3, its response sends and its audit (``approval_timing_errors``) |
| Workflow ticks | per vehicle, the retained order, then its role's: the owner's orbit before its request state; a peer's assignment, then its approach, then its request state |
| Pose cadence | the ``navpy`` and ``gcs`` arrival streams of every sysid, each file flushed whole |
| SNAP | each vehicle's first SNAP line is its latched first SNAP, by the generation that tick's members name; k_T is their maximum |

The slice's owner-only checks (forbidden messages, no peer source, exactly
one approval, owner-only pose files and assignment) do not apply.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path

from scripts.eval_gcs_demo_approval_replay import approval_replay_errors
from scripts.eval_gcs_demo_confirmation_timing import approval_timing_errors
from scripts.eval_gcs_demo_assignment_lines import parse_assignment_lines
from scripts.eval_gcs_demo_constants import backend_log_path
from scripts.eval_gcs_demo_fleet_packets import (
    applied_chains,
    applied_emissions,
    forwarding_errors,
    packet_run_errors,
)
from scripts.eval_gcs_demo_inputs import recorded_profile_parameters
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoMissionPlan, DemoScenario
from scripts.eval_gcs_demo_slice_gates import (
    JOURNAL_FILE,
    SNAP_MARKER,
    first_markers,
    pose_cadence_errors,
    run_errors,
    workflow_tick_errors,
)

#: The arrival-stream producers every sysid records.
POSE_PRODUCERS = ("navpy", "gcs")
#: Each role's markers, in the order they must first appear, all before the
#: request state (``Sending confirm request`` / ``CONFIRMING``).
ROLE_MARKERS: dict[str, tuple[tuple[str, str], ...]] = {
    "owner": (("orbit", "Self-detect orbit:"),),
    # task_capability.py, then peer_approach_start.py
    "peer": (("assigned", " assigned by owner "), ("approach", "Peer navigation started")),
}
_REQUEST_STATE = {"request": "Sending confirm request for P", "confirming": "CONFIRMING: P"}

Lines = list[tuple[int, str]]


def navpy_lines_file(sys_id: int) -> str:
    """NavPy ``sys_id``'s log lines with their ticks."""
    return f"lockstep_navpy{sys_id}_lines.jsonl"


def load_fleet_evidence(log_dir: Path, sys_ids: Sequence[int]) -> tuple[list[dict], dict[int, Lines]]:
    paths = [log_dir / JOURNAL_FILE, *(log_dir / navpy_lines_file(i) for i in sys_ids)]
    missing = [path.name for path in paths if not path.is_file()]
    if missing:
        raise ValueError(f"the certified lockstep run needs {', '.join(missing)}")
    records = [json.loads(row) for row in paths[0].read_text(encoding="utf-8").splitlines()]
    lines = {}
    for sys_id, path in zip(sys_ids, paths[1:]):
        pairs = [json.loads(row) for row in path.read_text(encoding="utf-8").splitlines()]
        lines[sys_id] = [(int(k), str(line)) for k, line in pairs]
    return records, lines


def role_order_errors(lines: Lines, role: str) -> list[str]:
    markers = ROLE_MARKERS[role]
    at = first_markers(lines, {**dict(markers), **_REQUEST_STATE})
    missing = [name for name, _marker in markers if name not in at]
    if missing:
        return [f"workflow ticks: no {', '.join(missing)} marker"]
    chain = [at[name] for name, _marker in markers]
    state = min((at[name] for name in _REQUEST_STATE if name in at), default=None)
    if state is not None:
        chain.append(state)
    if chain != sorted(chain) or len(set(chain)) != len(chain):
        names = [name for name, _marker in markers] + ["request state"]
        return [f"workflow ticks: {' -> '.join(names)} out of order at {chain}"]
    return []


def vehicle_workflow_errors(lines: Mapping[int, Lines], plan: DemoMissionPlan) -> list[str]:
    return [f"vehicle {vehicle.sys_id}: {error}" for vehicle in plan.vehicles
            for error in workflow_tick_errors(lines[vehicle.sys_id])
            + role_order_errors(lines[vehicle.sys_id], vehicle.role)]


def backend_assignment_errors(log_dir: Path, records: Sequence[Mapping],
                              plan: DemoMissionPlan) -> list[str]:
    """The backend log's APPLIED lines (``task_assign_listener``, one per APPLIED of a
    round's copy) against the journal. A line need not be a complete chain: a REPEAT's
    APPLIED may reach the backend while its copy to a retiring helper is discarded at stop."""
    owner = plan.owner.sys_id
    try:
        text = backend_log_path(log_dir).read_text(encoding="utf-8", errors="strict")
        logged = {(ack.owner, ack.helper, ack.task, ack.ref, ack.uid)
                  for ack in parse_assignment_lines(text).acks}
    except (OSError, UnicodeError, TypeError, ValueError) as error:
        return [f"assignment cross-check: {error}"]
    errors = [f"assignment cross-check: backend APPLIED {list(line)} is no journal APPLIED"
              for line in sorted(logged - applied_emissions(records))]
    held = {(helper, task) for helper, task, _ref, _uid in applied_chains(records, owner, plan.sys_ids)}
    errors += [f"assignment cross-check: helper {helper} holds task {task} with no backend APPLIED"
               for helper, task in sorted(held - {line[1:3] for line in logged})]
    return errors


def fleet_approval_errors(records: Sequence[Mapping], lines: Mapping[int, Lines],
                          approvals: Sequence[Mapping[str, JsonValue]]) -> list[str]:
    """Every approval's own timing on its vehicle's lines, then the replay of them all
    (unknown sysids are the approval count's error)."""
    errors = [error for approval in approvals if approval["sys_id"] in lines
              for error in approval_timing_errors(records, lines[approval["sys_id"]], approval)]
    return errors + approval_replay_errors(records, approvals)


def fleet_snap_errors(records: Sequence[Mapping], lines: Mapping[int, Lines]) -> list[str]:
    outcome = next((r for r in records if "outcome" in r), None)
    firsts = {sys_id: next((k for k, line in rows if SNAP_MARKER in line), None)
              for sys_id, rows in lines.items()}
    missing = [sys_id for sys_id, k in firsts.items() if k is None]
    if outcome is None or missing:
        return [f"SNAP: no terminal tick or no SNAP from vehicles {missing}"]
    errors = []
    latched = {ordinal: (generation, k) for ordinal, generation, k in outcome.get("first_snaps", ())}
    members = {r["k"]: {m["ordinal"]: m["generation"] for m in r.get("members", ())}
               for r in records if "k" in r}
    for sys_id, k in firsts.items():
        generation = members.get(k, {}).get(sys_id)
        if latched.get(sys_id) != (generation, k):
            errors.append(f"SNAP: vehicle {sys_id}'s first SNAP line at tick {k} (generation "
                          f"{generation}), latched {list(latched.get(sys_id, ()))}")
        if k > outcome["terminal_tick"]:
            errors.append(f"SNAP: vehicle {sys_id}'s first SNAP at tick {k}, after k_T "
                          f"{outcome['terminal_tick']}")
    if outcome["terminal_tick"] != max(firsts.values()):
        errors.append(f"SNAP: k_T {outcome['terminal_tick']} is not the last first SNAP "
                      f"{max(firsts.values())}")
    return errors


def certified_lockstep_gate_errors(log_dir: Path, plan: DemoMissionPlan,
                                   approvals: Sequence[Mapping[str, JsonValue]],
                                   scenario: DemoScenario | None = None) -> list[str]:
    del scenario  # these gates read the run's own logs only
    sys_ids = plan.sys_ids
    if sys_ids != tuple(range(1, len(sys_ids) + 1)):
        return [f"certified lockstep: sysids {list(sys_ids)} are not the fleet's 1..{len(sys_ids)}"]
    try:
        records, lines = load_fleet_evidence(log_dir, sys_ids)
        uart_loss = recorded_profile_parameters(log_dir).get("SIM_UART_LOSS", 0.0) > 0.0
    except (OSError, ValueError, RegressionError) as error:
        return [f"certified lockstep evidence: {error}"]
    peers = tuple(vehicle.sys_id for vehicle in plan.peers)
    return (run_errors(records) + packet_run_errors(records)
            + forwarding_errors(records, plan.owner.sys_id, peers, sys_ids, uart_loss=uart_loss)
            + backend_assignment_errors(log_dir, records, plan)
            + fleet_approval_errors(records, lines, approvals)
            + vehicle_workflow_errors(lines, plan) + fleet_snap_errors(records, lines)
            + pose_cadence_errors(log_dir, sys_ids, POSE_PRODUCERS))


__all__ = [
    "POSE_PRODUCERS", "ROLE_MARKERS", "backend_assignment_errors", "certified_lockstep_gate_errors", "fleet_approval_errors",
    "fleet_snap_errors", "load_fleet_evidence", "navpy_lines_file", "role_order_errors",
    "vehicle_workflow_errors",
]
