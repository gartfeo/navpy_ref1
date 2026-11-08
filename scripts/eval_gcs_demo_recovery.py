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
#: Recoveries quoted in a clean-run gate error; the report lists them all.
JOURNAL_SHOWN = 3


def recovery_journal(log_dir: Path, sys_id: int) -> Path:
    return log_dir / f"uav_{sys_id}_recovery.jsonl"


@dataclass(frozen=True)
class RecoveryEvidence:
    """One NavPy's recoveries (every process of it), or why its journal cannot be read."""

    events: tuple[dict[str, Any], ...] = ()
    errors: tuple[str, ...] = ()


def recovery_evidence(log_dir: Path, sys_id: int, navigation_text: str) -> RecoveryEvidence:
    path = recovery_journal(log_dir, sys_id)
    if not path.is_file():
        return RecoveryEvidence(errors=(f"missing {path.name}",))
    errors = []
    if RECOVERY_JOURNAL_FAILURE_MARKER in navigation_text:
        errors.append(f"recovery journal incomplete, not scored ({RECOVERY_JOURNAL_FAILURE_MARKER} logged)")
    events = []
    opened = False
    for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        try:
            entry = json.loads(line)
        except ValueError:
            errors.append(f"recovery journal {path.name}: line {number} is not JSON")
            continue
        if not isinstance(entry, dict):
            errors.append(f"recovery journal {path.name}: line {number} is not an object")
        elif entry.get("event") == "open":
            opened = True
        elif isinstance(entry.get("source"), str) and isinstance(entry.get("message"), str):
            events.append({"source": entry["source"], "message": entry["message"], "t": entry.get("t")})
        else:
            errors.append(f"recovery journal {path.name}: line {number} has no source and message")
    if not opened:
        errors.append(f"recovery journal {path.name}: no open line")
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


def run_verdict(vehicles: Sequence[Any], passed: bool, faults_planned: bool) -> str:
    if any(vehicle.verdict == INVALID for vehicle in vehicles):
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
    "CLEAN", "FAILED", "INVALID", "RECOVERED", "RecoveryEvidence", "faults_planned",
    "recovery_evidence", "recovery_journal", "run_verdict", "scored_with", "unexpected_recovery_errors",
    "vehicle_verdict",
]
