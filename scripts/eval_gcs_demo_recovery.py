"""Evals honest (docs/design/navpy_recovery.md, step 8): a run's recoveries and its verdict.

NavPy counts every recovery report in ``uav_<id>_recovery.jsonl`` (``core/kernel/logging/recovery_journal.py``),
apart from the evidence writer. The evaluator, not the run, sets the verdict:

- ``invalid``: the evidence is missing or incomplete (a missing file, the evidence or recovery-journal failure
  marker, unreadable evidence); not scored, never a pass.
- ``failed``: a product gate failed, or a run without scheduled faults recovered (the clean-run gate).
- ``recovered``: NavPy recovered, or the run's scheduled faults bit and recovered (their gates,
  ``eval_gcs_demo_fleet_faults``), and every gate passed. A pass only where recovery was expected.
- ``clean``: every gate passed and nothing recovered.
"""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navpy.core.kernel.logging.recovery_journal import RECOVERY_JOURNAL_FAILURE_MARKER

CLEAN, RECOVERED, INVALID, FAILED = "clean", "recovered", "invalid", "failed"
#: The run-level gates' evidence errors, each producer's prefix (Codex review of step 8, finding 4): the backend log,
#: the lockstep journal and NavPy lines, or the delivery evidence could not be read, so the run is not scored.
ASSIGNMENT_EVIDENCE_ERROR = "invalid global assignment evidence"
#: Readable assignment evidence that breaks an assignment rule (a non-owner request, one POI to several peers): a
#: product failure, so not in RUN_EVIDENCE_ERRORS.
ASSIGNMENT_RULE_ERROR = "global assignment rule broken"
LOCKSTEP_EVIDENCE_ERROR = "certified lockstep evidence"
SLICE_EVIDENCE_ERROR = "slice evidence"
DELIVERY_EVIDENCE_ERROR = "delivery: invalid evidence"
RUN_EVIDENCE_ERRORS = (ASSIGNMENT_EVIDENCE_ERROR, LOCKSTEP_EVIDENCE_ERROR, SLICE_EVIDENCE_ERROR,
                       DELIVERY_EVIDENCE_ERROR)
#: Recoveries quoted in a clean-run gate error; the report lists them all.
JOURNAL_SHOWN = 3


def recovery_journal(log_dir: Path, sys_id: int) -> Path:
    return log_dir / f"uav_{sys_id}_recovery.jsonl"


@dataclass(frozen=True)
class RecoveryEvidence:
    """One NavPy's recoveries (every process of it), or why its journal cannot be read."""

    events: tuple[dict[str, Any], ...] = ()
    errors: tuple[str, ...] = ()


@dataclass
class _Process:
    """One NavPy process's part of the journal: its open line, its entries, its close line if it closed."""

    entries: list[dict[str, Any]]
    close: dict[str, Any] | None = None


def _processes(name: str, text: str, errors: list[str]) -> list[_Process]:
    processes: list[_Process] = []
    for number, line in enumerate(text.splitlines(), 1):
        try:
            entry = json.loads(line)
        except ValueError:
            errors.append(f"recovery journal {name}: line {number} is not JSON")
            continue
        event = entry.get("event") if isinstance(entry, dict) else None
        if event == "open":
            processes.append(_Process([]))
        elif not processes:
            errors.append(f"recovery journal {name}: line {number} comes before an open line")
        elif processes[-1].close is not None:
            errors.append(f"recovery journal {name}: line {number} follows its process's close line")
        elif event == "close":
            processes[-1].close = entry
        elif isinstance(entry, dict) and isinstance(entry.get("source"), str) and isinstance(entry.get("message"), str):
            processes[-1].entries.append({"source": entry["source"], "message": entry["message"], "t": entry.get("t")})
        else:
            errors.append(f"recovery journal {name}: line {number} has no source and message")
    return processes


def _process_errors(name: str, processes: Sequence[_Process]) -> list[str]:
    """Each process that closed counted what it recorded; the last one must have closed (Codex review of step 8,
    finding 2): a journal cut short, or one whose writer dropped entries, is not the whole story."""
    if not processes:
        return [f"recovery journal {name}: no open line"]
    errors = []
    if processes[-1].close is None:
        errors.append(f"recovery journal {name}: NavPy's last process has no close line (cut short, or it did "
                      "not stop cleanly)")
    for index, process in enumerate(processes, 1):
        close = process.close
        if close is None:
            continue
        counts = (close.get("recorded"), close.get("dropped"))
        if not all(type(count) is int and count >= 0 for count in counts):   # bool is not a count
            errors.append(f"recovery journal {name}: process {index} close line has malformed counts {counts}")
            continue
        if close.get("dropped") != 0:
            errors.append(f"recovery journal {name}: process {index} dropped {close.get('dropped')} entries")
        if close.get("recorded") != len(process.entries) + (close.get("dropped") or 0):
            errors.append(f"recovery journal {name}: process {index} recorded {close.get('recorded')} entries, "
                          f"the journal holds {len(process.entries)}")
    return errors


def recovery_evidence(log_dir: Path, sys_id: int, navigation_text: str) -> RecoveryEvidence:
    """Each process's entries; each NavPy process after the first is a restart, a recovery itself (Codex review of
    step 8, finding 2), whether its predecessor crashed or stopped."""
    path = recovery_journal(log_dir, sys_id)
    if not path.is_file():
        return RecoveryEvidence(errors=(f"missing {path.name}",))
    errors = []
    if RECOVERY_JOURNAL_FAILURE_MARKER in navigation_text:
        errors.append(f"recovery journal incomplete, not scored ({RECOVERY_JOURNAL_FAILURE_MARKER} logged)")
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        return RecoveryEvidence(errors=(*errors, f"recovery journal {path.name}: unreadable: {error}"))
    processes = _processes(path.name, text, errors)
    errors += _process_errors(path.name, processes)
    events = []
    for index, process in enumerate(processes):
        if index:
            ended = "stopped" if processes[index - 1].close is not None else "ended without closing"
            events.append({"source": "process", "message": f"NavPy restarted (process {index + 1}; the previous "
                           f"one {ended})", "t": None})
        events += process.entries
    return RecoveryEvidence(tuple(events), tuple(errors))


def vehicle_verdict(evidence_errors: Sequence[str], errors: Sequence[str],
                    recoveries: Sequence[Mapping[str, Any]]) -> str:
    """``evidence_errors``: the gates that found the evidence missing or broken (a subset of ``errors``)."""
    if evidence_errors:
        return INVALID
    if errors:
        return FAILED
    return RECOVERED if recoveries else CLEAN


def unexpected_recovery_errors(vehicles: Sequence[Any], faults_planned: bool) -> list[str]:
    """The clean-run gate: without scheduled faults, any recovery fails the run."""
    if faults_planned:
        return []
    errors = []
    for vehicle in vehicles:
        if not vehicle.recoveries:
            continue
        shown = "; ".join(f"{r['source']}: {r['message']}" for r in vehicle.recoveries[:JOURNAL_SHOWN])
        more = len(vehicle.recoveries) - JOURNAL_SHOWN
        errors.append(f"uav {vehicle.sys_id}: unexpected recovery in a clean run: {shown}"
                      + (f" (+{more} more)" if more > 0 else ""))
    return errors


def run_verdict(vehicles: Sequence[Any], errors: Sequence[str], passed: bool, faults_planned: bool) -> str:
    if (any(vehicle.verdict == INVALID for vehicle in vehicles)
            or any(error.startswith(RUN_EVIDENCE_ERRORS) for error in errors)):
        return INVALID
    if not passed:
        return FAILED
    return RECOVERED if faults_planned or any(vehicle.recoveries for vehicle in vehicles) else CLEAN


def scored_with(verdict: str, errors: Sequence[str]) -> str:
    """The verdict after product errors found beside the analysis (a lockstep run's outcome): invalid stays."""
    return verdict if verdict in ("", INVALID) or not errors else FAILED


def faults_planned(log_dir: Path, journal_file: str) -> bool:
    """A lockstep run's grid plans faults (``coordinator_faults``); a wall run has no journal and no faults."""
    path = log_dir / journal_file
    if not path.is_file():
        return False
    for line in path.read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        if "grid" in record:
            return bool(record["grid"].get("faults"))
    return False


__all__ = [
    "ASSIGNMENT_EVIDENCE_ERROR", "ASSIGNMENT_RULE_ERROR", "CLEAN", "DELIVERY_EVIDENCE_ERROR", "FAILED", "INVALID", "LOCKSTEP_EVIDENCE_ERROR",
    "RECOVERED", "RUN_EVIDENCE_ERRORS", "RecoveryEvidence", "SLICE_EVIDENCE_ERROR", "faults_planned",
    "recovery_evidence", "recovery_journal", "run_verdict", "scored_with", "unexpected_recovery_errors",
    "vehicle_verdict",
]
