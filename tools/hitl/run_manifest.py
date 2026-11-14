"""One approach run's manifest (hitl_guidance.md section 9, stage 1 step 13): its files and their producers.

The runner (tools/tuning/approach.py) writes it once every owned process has stopped:

- ``producers``: each role it started once (pid, argv, exit code, whether it ended after the runner's stop);
- ``files``: each output file by role, with its producer, sha256, size and line count, and whether it carries
  its clean-end marker (``END_MARKERS``).

``verify`` is what check_integration_timing.py runs with ``--manifest``: every input file must be the manifest's
file of that role (by content, not by path, since WSL and Windows name the same file differently), the manifest
and every log must be the run record's run, every producer must have ended after the stop, and every log must end
cleanly *now*: a log cut off at a line boundary still parses, so only its end marker shows it was cut.
"""
from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, Optional

from navpy.core.perception.pipeline.integration_log import EVENT_LOG_END

SCHEMA = "navpy.hitl.run_manifest.v1"
UNREAL_END = "Log file closed"           # Unreal's last line when its log is closed (FOutputDeviceFile)

# File roles the timing check reads, and the producer each must come from.
FILE_PRODUCERS = {
    "run_record": "runner",
    "navpy_integration_log": "navpy",
    "publisher_status": "publisher",
    "publisher_frames": "publisher",
    "unreal_log": "unreal",
    "emulator_log": "emulator",
    "media_fault_log": "navpy",          # media_fault_injector.py runs inside navpy's process
}
# Roles only some runs have: a commissioning run's fault log (step 15), which the timing check never reads, since it
# refuses a commissioning run outright.
OPTIONAL_ROLES = frozenset({"media_fault_log"})


def _lines(data: bytes) -> list[str]:
    return data.decode("utf-8", errors="replace").splitlines()


def _last_json(data: bytes) -> Optional[dict]:
    for line in reversed(_lines(data)):
        if line.strip():
            try:
                record = json.loads(line)
            except ValueError:
                return None
            return record if isinstance(record, dict) else None
    return None


def _navpy_ended(data: bytes) -> bool:
    last = _last_json(data)
    return bool(last and last.get("event") == EVENT_LOG_END)


def _emulator_ended(data: bytes) -> bool:
    last = _last_json(data)
    return bool(last and last.get("event") == "summary")


def _fault_log_ended(data: bytes) -> bool:
    last = _last_json(data)
    return bool(last and last.get("event") == "close")


def _publisher_ended(data: bytes) -> bool:
    try:
        status = json.loads(data)
    except ValueError:
        return False
    return isinstance(status, dict) and status.get("exit_code") == 0 and "ended_monotonic_ns" in status


def _unreal_ended(data: bytes) -> bool:
    lines = [line for line in _lines(data) if line.strip()]
    return bool(lines) and UNREAL_END in lines[-1]


def _frames_whole(data: bytes) -> bool:
    """A CSV the publisher closed ends with a newline; a torn last row does not."""
    return data.endswith(b"\n")


END_MARKERS = {
    "run_record": lambda data: True,
    "navpy_integration_log": _navpy_ended,
    "publisher_status": _publisher_ended,
    "publisher_frames": _frames_whole,
    "unreal_log": _unreal_ended,
    "emulator_log": _emulator_ended,
    "media_fault_log": _fault_log_ended,
}


def digest(path: Path) -> dict[str, Any]:
    data = Path(path).read_bytes()
    return {"sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)}


def file_entry(path: Path, role: str) -> dict[str, Any]:
    data = Path(path).read_bytes()
    return {"role": role, "name": Path(path).name, "producer": FILE_PRODUCERS[role],
            "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data), "lines": len(_lines(data)),
            "ended": END_MARKERS[role](data)}


def build(run_id: str, producers: Mapping[str, Mapping[str, Any]],
          files: Sequence[tuple[str, Path]]) -> dict[str, Any]:
    """The manifest of a stopped run; files are (role, path) pairs."""
    return {"schema": SCHEMA, "run_id": run_id,
            "producers": {name: dict(item) for name, item in producers.items()},
            "files": [file_entry(path, role) for role, path in files]}


def verify(manifest: Mapping[str, Any], run_id: str, inputs: Sequence[tuple[str, Path]],
           log_run_ids: set[str]) -> list[str]:
    """Why these inputs are not one whole run of this manifest (empty: they are)."""
    if manifest.get("schema") != SCHEMA:
        return [f"manifest schema {manifest.get('schema')!r} is not {SCHEMA}"]
    failures = []
    if manifest.get("run_id") != run_id:
        failures.append(f"manifest run {manifest.get('run_id')!r} is not the record's run {run_id!r}")
    if log_run_ids != {run_id}:
        failures.append(f"navpy log runs {sorted(log_run_ids)} are not only {run_id!r}")
    producers = manifest.get("producers", {})
    for name in sorted(set(FILE_PRODUCERS.values())):
        producer = producers.get(name)
        if not isinstance(producer, dict):
            failures.append(f"manifest has no producer {name}")
        elif producer.get("ended_after_stop") is not True:
            failures.append(f"producer {name} did not end after the runner's stop")
    entries = [entry for entry in manifest.get("files", []) if isinstance(entry, dict)]
    for role, path in inputs:
        data = Path(path).read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        match = [entry for entry in entries if entry.get("role") == role and entry.get("sha256") == digest
                 and entry.get("bytes") == len(data)]
        if not match:
            failures.append(f"{role} {path} is not a file of this run's manifest")
            continue
        if match[0].get("producer") != FILE_PRODUCERS[role]:
            failures.append(f"{role} {path} comes from {match[0].get('producer')}, not {FILE_PRODUCERS[role]}")
        if not END_MARKERS[role](data) or match[0].get("ended") is not True:
            failures.append(f"{role} {path} does not end cleanly (cut off)")
    for role in FILE_PRODUCERS.keys() - OPTIONAL_ROLES:
        if not any(given == role for given, _ in inputs):
            failures.append(f"no {role} input was checked")
    return failures


__all__ = ["END_MARKERS", "FILE_PRODUCERS", "OPTIONAL_ROLES", "SCHEMA", "build", "file_entry", "verify"]
