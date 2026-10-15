"""Passing lockstep slice evidence for an owner's NavPy log: a journal and its lines.

Built as the coordinator writes them (``scripts/eval_gcs_demo_lockstep.py``):
NavPy's lines with their ticks on a uniform 20 ms grid, the operator's
observations of the confirmation (e2-e6) with its polling-schedule evidence
(the real workflow's, ``approval_runs``), the response sends' NAVLINK
counts, the decoded message totals (the owner's self-assignment only) and
an ``ok`` outcome whose terminal tick is the first SNAP line's.
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

from navpy.sim.lockstep.coordinator_mavlink import DIRECTIONS, ROUTER_LINK
from navpy.sim.lockstep.coordinator_types import serial0_device, serial1_device
from scripts.eval_gcs_demo_confirmation_timing import RESPONSE_ID
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE, NAVPY_LINES_FILE, POSE_CADENCE_STREAMS
from scripts.lockstep_wsl_sitl import BOOT_BUDGET_US
from tests.modules.lockstep.approval_runs import request, run_workflow, uniform

GRID_US = 20_000
MAX_GAP_US = 20_825
#: Ticks the confirmation workflow may take after the request's delivery.
WORKFLOW_TICKS = 1_000
GCS_HEARTBEAT = "0:255.0>-"


def owner_keys(sys_id: int) -> dict[str, str]:
    """Journal keys ``<id>:<src_sys>.<src_comp>><target>`` for the owner: its
    companion's self-assignment, the backend's response, its heartbeats."""
    return {"self_assignment": f"25106:{sys_id}.191>{sys_id}",
            "response": f"{RESPONSE_ID}:255.0>{sys_id}",
            "autopilot": f"0:{sys_id}.1>-", "companion": f"0:{sys_id}.191>-"}


@lru_cache
def _confirmation(sys_id: int, delivered: int) -> tuple[dict[int, dict], dict[int, int], dict]:
    """The real workflow's operator observations by tick, its response sends
    by tick and its approval, for the request delivered at ``delivered``."""
    records, (approval,) = run_workflow(uniform(GRID_US, delivered + WORKFLOW_TICKS),
                                        {delivered: [request(sys_id)]}, {sys_id}, max_gap_us=MAX_GAP_US)
    operator = {r["k"]: r["operator"] for r in records if "operator" in r}
    sends = {r["k"]: n for r in records for n in r.get("navlink", {}).get(
        serial1_device(sys_id), {}).get("to_sitl", {}).values()}
    return operator, sends, approval


def _ticks(lines: list[str], sys_id: int) -> tuple[list[int], dict]:
    """Line ticks: up to ``CONFIRMING`` (e1) one per tick, the request
    delivered at e1's tick, the rest after the approval's answer (k6)."""
    confirming = next(i for i, line in enumerate(lines) if "CONFIRMING: P1" in line)
    operator, sends, _approval = _confirmation(sys_id, confirming + 1)
    k6 = max(sends)
    ticks = [i + 1 if i <= confirming else k6 + i for i in range(len(lines))]
    return ticks, {"operator": operator, "sends": sends}


def slice_approval(sys_id: int, lines: list[str]) -> dict:
    confirming = next(i for i, line in enumerate(lines) if "CONFIRMING: P1" in line)
    return dict(_confirmation(sys_id, confirming + 1)[2])


def navigation_lines(log_dir: Path, sys_id: int) -> list[str]:
    text = (log_dir / f"uav_{sys_id}_navigation.log").read_text(encoding="utf-8")
    return [line for line in text.splitlines() if line]


def slice_records(lines: list[str], sys_id: int) -> list[dict]:
    ticks, k = _ticks(lines, sys_id)
    keys = owner_keys(sys_id)
    terminal = next(t for t, line in zip(ticks, lines) if "SNAP(VISION-NAV" in line)
    serial0, serial1 = serial0_device(sys_id), serial1_device(sys_id)
    records: list[dict] = [{"grid": {"max_frame_gap_us": MAX_GAP_US, "drain_period_us": GRID_US,
                                     "mission_budget_us": 3_600_000_000,
                                     "sitl_ports": [[serial0, serial1]], "boot_budget_us": BOOT_BUDGET_US,
                                     "expected_fleet": [sys_id]}}]
    for tick in range(1, terminal + 3):
        record: dict = {"k": tick, "phase": "mission", "t_us": tick * GRID_US}
        if tick in k["operator"]:
            record["operator"] = k["operator"][tick]
        if tick in k["sends"]:
            record["navlink"] = {serial1: {"to_sitl": {keys["response"]: k["sends"][tick]}}}
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
    ticks, _k = _ticks(lines, sys_id)
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
