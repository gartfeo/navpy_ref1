"""Passing lockstep slice evidence for an owner's NavPy log: a journal and its lines.

Built as the coordinator writes them (``scripts/eval_gcs_demo_lockstep.py``):
NavPy's lines with their ticks on a uniform 20 ms grid, the operator's
observations of the confirmation (e2-e6), the response sends' NAVLINK
counts, the decoded message totals (the owner's self-assignment only) and
an ``ok`` outcome whose terminal tick is the first SNAP line's.
"""

from __future__ import annotations

import json
from pathlib import Path

from navpy.sim.lockstep.coordinator_mavlink import DIRECTIONS, ROUTER_LINK
from navpy.sim.lockstep.coordinator_types import serial0_device, serial1_device
from scripts.eval_gcs_demo_confirmation_timing import RESPONSE_ID
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE, NAVPY_LINES_FILE, POSE_CADENCE_STREAMS

GRID_US = 20_000
MAX_GAP_US = 20_825
APPROVAL_ID = [0, 9]
#: The confirmation's ticks after k3 (e3): the approval 15.02 s later, then
#: the three response sends 13 ticks (0.26 s) apart.
K4_AFTER_K3, SEND_GAP_TICKS = 751, 13
GCS_HEARTBEAT = "0:255.0>-"


def owner_keys(sys_id: int) -> dict[str, str]:
    """Journal keys ``<id>:<src_sys>.<src_comp>><target>`` for the owner: its
    companion's self-assignment, the backend's response, its heartbeats."""
    return {"self_assignment": f"25106:{sys_id}.191>{sys_id}",
            "response": f"{RESPONSE_ID}:255.0>{sys_id}",
            "autopilot": f"0:{sys_id}.1>-", "companion": f"0:{sys_id}.191>-"}


def _ticks(lines: list[str]) -> tuple[list[int], dict[str, int]]:
    confirming = next(i for i, line in enumerate(lines) if "CONFIRMING: P1" in line)
    k3 = confirming + 1
    k4 = k3 + K4_AFTER_K3
    sends = [k4 + 1 + n * SEND_GAP_TICKS for n in range(3)]
    k6 = sends[-1]
    ticks = [i + 1 if i <= confirming else k6 + i for i in range(len(lines))]
    return ticks, {"k3": k3, "k4": k4, "k6": k6, "sends": sends}


def slice_approval(sys_id: int, lines: list[str]) -> dict:
    _ticks_, k = _ticks(lines)
    return {
        "sys_id": sys_id, "task_id": 1, "is_confirmed": True, "action": "approve",
        "round_uid": "10:1", "request_observed": True, "response": "approved",
        "request_observed_at_unix_s": 1000.0 + k["k3"] * GRID_US / 1e6,
        "approved_at_unix_s": 1000.0 + k["k6"] * GRID_US / 1e6,
        "requested_delay_s": 15.0, "actual_delay_s": (k["k6"] - k["k3"]) * GRID_US / 1e6,
        "nav_before_approval": False,
    }


def navigation_lines(log_dir: Path, sys_id: int) -> list[str]:
    text = (log_dir / f"uav_{sys_id}_navigation.log").read_text(encoding="utf-8")
    return [line for line in text.splitlines() if line]


def slice_records(lines: list[str], sys_id: int) -> list[dict]:
    ticks, k = _ticks(lines)
    keys = owner_keys(sys_id)
    terminal = next(t for t, line in zip(ticks, lines) if "SNAP(VISION-NAV" in line)
    serial0, serial1 = serial0_device(sys_id), serial1_device(sys_id)
    records: list[dict] = [{"grid": {"max_frame_gap_us": MAX_GAP_US, "drain_period_us": GRID_US,
                                     "mission_budget_us": 3_600_000_000,
                                     "sitl_ports": [[serial0, serial1]], "boot_budget_us": None,
                                     "expected_fleet": [sys_id]}}]
    for tick in range(1, terminal + 3):
        record: dict = {"k": tick, "phase": "mission", "t_us": tick * GRID_US}
        if tick == k["k3"]:
            record["operator"] = {"receipts": [[0, 1]], "received": [[tick, "task_confirm_request"]]}
        elif tick == k["k4"]:
            record["operator"] = {"sent": [[*APPROVAL_ID, "POST", "/api/control/task_confirm"]]}
        elif tick == k["k6"]:
            record["operator"] = {"answered": [[*APPROVAL_ID, 200]]}
        if tick in k["sends"]:
            record["navlink"] = {serial1: {"to_sitl": {keys["response"]: 1}}}
        records.append(record)
    def tally(messages: dict) -> dict:
        return {"messages": messages, "skipped_bytes": 0}

    quiet = {keys["autopilot"]: 10, GCS_HEARTBEAT: 10}
    to_backend = {keys["autopilot"]: 10, GCS_HEARTBEAT: 10}
    gcs = {GCS_HEARTBEAT: 10, keys["response"]: 3}
    counts = {link: {direction: tally({}) for direction in DIRECTIONS}
              for link in (serial0, serial1, ROUTER_LINK)}
    counts[serial0].update(from_sitl=tally(quiet),
                           to_sitl=tally({keys["companion"]: 10, keys["self_assignment"]: 1}))
    counts[serial1].update(from_sitl=tally(quiet), to_sitl=tally(gcs), routed_out=tally(gcs))
    counts[ROUTER_LINK].update(routed_out=tally(to_backend))
    records.append({"mavlink_counts": counts})
    records.append({"outcome": "ok", "terminal_tick": terminal, "final_tick": terminal + 2,
                    "parked_tick": terminal + 3,
                    "first_snaps": [[sys_id, 1, terminal]]})
    return records


def write_pose_cadence(log_dir: Path, sys_id: int) -> None:
    """Flushed producer-named pose-cadence CSVs: the owner NavPy's arrivals."""
    directory = log_dir / SOURCE_TIME_SUBDIR
    directory.mkdir(exist_ok=True)
    for stream in POSE_CADENCE_STREAMS:
        (directory / f"pose_cadence_{stream}_uav_{sys_id}_navpy.csv").write_text(
            "wall_s,boot_ms\n1000.0,20\n1000.02,40\n", encoding="utf-8")


def write_slice_journal(log_dir: Path, sys_id: int, records: list[dict] | None = None,
                        lines: list[str] | None = None) -> list[dict]:
    """Write passing evidence for the owner's log (or the given records and
    lines); return the matching approvals."""
    lines = navigation_lines(log_dir, sys_id) if lines is None else lines
    records = slice_records(lines, sys_id) if records is None else records
    ticks, _k = _ticks(lines)
    write_pose_cadence(log_dir, sys_id)
    (log_dir / JOURNAL_FILE).write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")
    (log_dir / NAVPY_LINES_FILE).write_text(
        "".join(json.dumps([t, line]) + "\n" for t, line in zip(ticks, lines)), encoding="utf-8")
    return [slice_approval(sys_id, lines)]


def written_approvals(log_dir: Path, sys_id: int) -> list[dict]:
    """The approvals matching the lines a run directory's evidence holds."""
    rows = (log_dir / NAVPY_LINES_FILE).read_text(encoding="utf-8").splitlines()
    return [slice_approval(sys_id, [json.loads(row)[1] for row in rows])]


__all__ = [
    "navigation_lines", "owner_keys", "slice_approval", "slice_records", "write_pose_cadence", "write_slice_journal", "written_approvals",
]
