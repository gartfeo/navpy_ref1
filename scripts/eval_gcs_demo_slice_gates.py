"""The owner-only slice's journal gates (``lockstep_slice.md``, Slice checks).

They read the lockstep run's evidence in the run directory, after the
snapshot like every gate: the hash-chained journal
(``lockstep_journal.jsonl``) and NavPy's log lines with their ticks
(``lockstep_navpy_lines.jsonl``, ``[tick, line]``, in stream order).

| Gate | Evidence | Pass |
|---|---|---|
| Run | the journal | outcome ``ok``, no fault record |
| Sensor | NavPy lines | ``POI: P<task> (tracking`` from the owner |
| Workflow ticks | NavPy lines as (tick, stream position) | the retained order |
| Assignment | the journal's decoded message counts (with their source and target), NavPy lines | no peer relay, none of :data:`SLICE_FORBIDDEN_MESSAGES` but the owner's self-assignment, no peer marker |
| Approval | the approvals and the journal | exactly one, for P, with the confirmation timing and the polling-schedule replay |
| SNAP | NavPy lines and the outcome | the first SNAP line is at the mission terminal tick |
| Pose cadence | ``source-time/*.csv`` | producer-named files, the owner's NavPy arrival streams, every file's tail flushed whole |

The command-bound and cadence gates are the retained vehicle gates
(``analyze_vehicle``) on the named-read bounds and the profile's speedup 1.
A run without this evidence (a wall run) fails: the slice runs only under
``--lockstep``.
"""

from __future__ import annotations

import json
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.coordinator_mavlink import MessageKey, parse_key
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_approval_replay import approval_replay_errors
from scripts.eval_gcs_demo_confirmation_timing import confirmation_timing_errors
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_role_gates import peer_assignment_marker_errors
from scripts.eval_gcs_demo_scenario import DemoMissionPlan

JOURNAL_FILE = "lockstep_journal.jsonl"
NAVPY_LINES_FILE = "lockstep_navpy_lines.jsonl"
SNAP_MARKER = "SNAP(VISION-NAV"
#: Messages that must never cross either link or direction in an owner-only
#: run, by kind. The one exception is :func:`is_self_assignment`.
SLICE_FORBIDDEN_MESSAGES: dict[str, tuple[str, ...]] = {
    # The auction's request, the peer's reply, and the ack only the auction
    # and an assigned peer send (task_auction_coordinator.py, task_capability.py).
    "task dispatch": ("TASK_ASSIGN_REQUEST", "TASK_ASSIGN_RESPONSE", "SWARM_ACK"),
    # A free peer's availability advert and the owner's offer.
    "advert": ("AVAILABLE_TASK_REQUEST", "AVAILABLE_TASK_RESPONSE"),
}
#: Peer relay. Swarm traffic reaches a vehicle only by MAVLink routing, which
#: forwards a packet with its original source (mavlink-router between the
#: SITLs, the autopilot between its serial ports; ``router_model.py``); no
#: product code re-originates a peer's message. So a peer's traffic on either
#: link is any message whose source system is neither the owner (its autopilot
#: and companion share its system id) nor the ground station.
GCS_SYSTEM = 255  # the backend's MAVLink source system (vehicle_manager.py)
#: The owner's self-assignment publication comes from its companion.
COMPANION_COMPONENT = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
#: The retained workflow markers (``eval_gcs_demo_gates._ordered_workflow_errors``).
WORKFLOW_MARKERS = {
    "request": "Sending confirm request for P",
    "confirming": "CONFIRMING: P",
    "confirmed": "confirmed by ground station.",
    "tracking": "start_tracking obj_id=",
    "nav": "INIT: NAV MODE",
    "reset": "RESET: PASSED POI",
    "snap": SNAP_MARKER,
}


#: The owner NavPy's pose-cadence streams every slice run feeds (its telemetry).
POSE_CADENCE_STREAMS = ("attitude_arrival", "position_arrival")
_PRODUCER_FILE = re.compile(r"^pose_cadence_[a-z_]+_uav_\d+_(navpy|gcs)\.csv$")


@dataclass(frozen=True)
class SliceEvidence:
    records: list[dict]
    lines: list[tuple[int, str]]


def load_slice_evidence(log_dir: Path) -> SliceEvidence:
    journal, lines = log_dir / JOURNAL_FILE, log_dir / NAVPY_LINES_FILE
    if not journal.is_file() or not lines.is_file():
        raise ValueError("the owner-only slice needs the lockstep journal and NavPy lines "
                         "(run it with --lockstep)")
    records = [json.loads(row) for row in journal.read_text(encoding="utf-8").splitlines()]
    pairs = [tuple(json.loads(row)) for row in lines.read_text(encoding="utf-8").splitlines()]
    return SliceEvidence(records, [(int(k), str(line)) for k, line in pairs])


def run_errors(records: Sequence[Mapping]) -> list[str]:
    errors = [f"lockstep fault {r['fault']} {r['kind']} at tick {r['tick']}" for r in records
              if "fault" in r]
    outcome = next((r for r in records if "outcome" in r), None)
    if outcome is None:
        errors.append("the journal records no run outcome")
    elif outcome["outcome"] != "ok":
        errors.append(f"lockstep outcome {outcome['outcome']!r}, not 'ok'")
    return errors


def sensor_errors(lines: Sequence[tuple[int, str]], poi: int) -> list[str]:
    marker = f"POI: P{poi} (tracking"
    return [] if any(marker in line for _k, line in lines) else [f"owner never logged {marker!r}"]


def workflow_tick_errors(lines: Sequence[tuple[int, str]]) -> list[str]:
    ticks = [k for k, _line in lines]
    if ticks != sorted(ticks):
        return ["NavPy lines are not in tick order"]
    at: dict[str, tuple[int, int]] = {}
    for position, (k, line) in enumerate(lines):
        for name, marker in WORKFLOW_MARKERS.items():
            if name not in at and marker in line:
                at[name] = (k, position)
    missing = [name for name in WORKFLOW_MARKERS if name not in at]
    if missing:
        return [f"workflow ticks: no {', '.join(missing)} marker"]
    errors = []
    if max(at["request"], at["confirming"]) > at["confirmed"]:
        errors.append(f"workflow ticks: confirmed at {at['confirmed']} before the request state "
                      f"({at['request']}, {at['confirming']})")
    chain = [at[name] for name in ("confirmed", "nav", "reset", "snap")]
    if chain != sorted(chain) or len(set(chain)) != len(chain):
        errors.append(f"workflow ticks: confirmed -> NAV -> reset -> SNAP out of order at {chain}")
    return errors


def forbidden_message_ids() -> dict[int, str]:
    """Message id -> its kind and name."""
    return {getattr(mavlink, f"MAVLINK_MSG_ID_{name}"): f"{kind} {name}"
            for kind, names in SLICE_FORBIDDEN_MESSAGES.items() for name in names}


def is_self_assignment(key: MessageKey, owner: int) -> bool:
    """The owner companion's ``TASK_ASSIGN_REQUEST`` to its own system
    (``self_assignment_publisher.py``): its task's publication, no dispatch."""
    return (key.message_id == mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST
            and (key.source_system, key.source_component) == (owner, COMPANION_COMPONENT)
            and key.target_system == owner)


def _message_error(key: MessageKey, owner: int, forbidden: Mapping[int, str]) -> str | None:
    if key.source_system not in (owner, GCS_SYSTEM):
        name = forbidden.get(key.message_id, f"message {key.message_id}")
        return f"peer relay {name} from system {key.source_system}"
    if key.message_id in forbidden and not is_self_assignment(key, owner):
        return (f"{forbidden[key.message_id]} from {key.source_system}.{key.source_component} "
                f"to {key.target_system}")
    return None


def assignment_errors(records: Sequence[Mapping], lines: Sequence[tuple[int, str]],
                      owner: int) -> list[str]:
    counts = next((r["mavlink_counts"] for r in records if "mavlink_counts" in r), None)
    if counts is None:
        return ["the journal records no MAVLink message counts"]
    forbidden = forbidden_message_ids()
    errors = []
    for device, directions in sorted(counts.items()):
        for direction, tally in sorted(directions.items()):
            for raw, n in sorted(tally["messages"].items()):
                error = _message_error(parse_key(raw), owner, forbidden) if n else None
                if error is not None:
                    errors.append(f"owner-only slice carried {n} {error} on {device} {direction}")
    return errors + peer_assignment_marker_errors("\n".join(line for _k, line in lines))


def approval_errors(records: Sequence[Mapping], lines: Sequence[tuple[int, str]],
                    approvals: Sequence[Mapping[str, JsonValue]], sys_id: int, poi: int) -> list[str]:
    if len(approvals) != 1:
        return [f"expected exactly one approval, found {len(approvals)}"]
    (approval,) = approvals
    if (approval["sys_id"], approval["task_id"]) != (sys_id, poi):
        return [f"approval for ({approval['sys_id']}, P{approval['task_id']}), not ({sys_id}, P{poi})"]
    return (confirmation_timing_errors(records, lines, approval, poi, sys_id)
            + approval_replay_errors(records, approvals))


def snap_errors(records: Sequence[Mapping], lines: Sequence[tuple[int, str]]) -> list[str]:
    outcome = next((r for r in records if "outcome" in r), None)
    snap = next((k for k, line in lines if SNAP_MARKER in line), None)
    if outcome is None or snap is None:
        return ["no SNAP at a mission terminal tick"]
    if snap != outcome["terminal_tick"]:
        return [f"first SNAP at tick {snap}, mission terminal tick {outcome['terminal_tick']}"]
    return []


def pose_cadence_errors(log_dir: Path, sys_id: int) -> list[str]:
    """The pose-cadence CSVs: stable producer names, the owner's arrival
    streams, and each file flushed whole (a header, then complete rows ending
    in a newline: the strict flush at the participant stop)."""
    directory = log_dir / SOURCE_TIME_SUBDIR
    files = sorted(directory.glob("*.csv")) if directory.is_dir() else []
    names = {path.name for path in files}
    errors = [f"pose cadence: no {name}" for name in
              (f"pose_cadence_{stream}_uav_{sys_id}_navpy.csv" for stream in POSE_CADENCE_STREAMS)
              if name not in names]
    for path in files:
        if not _PRODUCER_FILE.match(path.name):
            errors.append(f"pose cadence: {path.name} is not named by producer")
            continue
        text = path.read_text(encoding="utf-8")
        rows = text.split("\n")
        if not text.endswith("\n") or len(rows) < 3:
            errors.append(f"pose cadence: {path.name} has no flushed rows")
            continue
        width = rows[0].count(",")
        torn = [i for i, row in enumerate(rows[1:-1], 2) if row.count(",") != width]
        if torn:
            errors.append(f"pose cadence: {path.name} row {torn[0]} is torn")
    return errors


def slice_gate_errors(log_dir: Path, plan: DemoMissionPlan,
                      approvals: Sequence[Mapping[str, JsonValue]]) -> list[str]:
    try:
        evidence = load_slice_evidence(log_dir)
    except (OSError, ValueError) as error:
        return [f"slice evidence: {error}"]
    records, lines = evidence.records, evidence.lines
    poi, sys_id = plan.pois[0].task_id, plan.owner.sys_id
    return (run_errors(records) + sensor_errors(lines, poi) + workflow_tick_errors(lines)
            + assignment_errors(records, lines, sys_id)
            + approval_errors(records, lines, approvals, sys_id, poi) + snap_errors(records, lines)
            + pose_cadence_errors(log_dir, sys_id))


__all__ = [
    "COMPANION_COMPONENT", "GCS_SYSTEM", "JOURNAL_FILE", "NAVPY_LINES_FILE", "SLICE_FORBIDDEN_MESSAGES",
    "SliceEvidence", "is_self_assignment",
    "WORKFLOW_MARKERS", "approval_errors", "assignment_errors", "forbidden_message_ids",
    "POSE_CADENCE_STREAMS", "load_slice_evidence", "pose_cadence_errors", "run_errors", "sensor_errors", "slice_gate_errors", "snap_errors",
    "workflow_tick_errors",
]
