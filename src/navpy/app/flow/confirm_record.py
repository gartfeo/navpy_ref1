"""The confirm record: the mission-state event that releases the HITL run's recovery floor (HITL step 23b).

Docks stand on the ground (owner, 2026-11-08, option A): the runner's height floor (tools/tuning/flight_floor.py)
guards the lane, the search and the orbit, and is released at the operator's confirm, step 25's confirm that
releases NAV. navpy prints one flushed line, ``NAVPY_CONFIRM_V1 {"task": N}``, the first time a task's confirm
releases NAV (a confirmation the gates revoked before NAV releases nothing); the runner reads it from navpy's output.
It is a mission-state event: nothing on the NAV command or completion path reads it, and no altitude is read for it.
"""
from __future__ import annotations

import json

NAVPY_CONFIRM_MARKER = "NAVPY_CONFIRM_V1"


def print_confirm_record(task_id: int) -> None:
    encoded = json.dumps({"task": int(task_id)}, sort_keys=True, separators=(",", ":"))
    print(f"{NAVPY_CONFIRM_MARKER} {encoded}", flush=True)


def parse_confirm_record(line: str) -> dict[str, object] | None:
    prefix = f"{NAVPY_CONFIRM_MARKER} "
    if not isinstance(line, str) or not line.startswith(prefix):
        return None
    try:
        record = json.loads(line[len(prefix):])
    except ValueError:
        return None
    if not isinstance(record, dict) or isinstance(record.get("task"), bool) or not isinstance(record.get("task"), int):
        return None
    return record


__all__ = ["NAVPY_CONFIRM_MARKER", "parse_confirm_record", "print_confirm_record"]
