"""Partial evidence for failed three-UAV operator confirmation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

from scripts.eval_gcs_demo_models import StackContext
from scripts.eval_gcs_demo_ports import JsonValue


def persist_confirmation_failure(
    context: StackContext,
    log_dir: Path,
    observed_sys_ids: set[int],
    attempted_sys_ids: set[int],
    approvals: list[dict[str, JsonValue]],
    *,
    status: str,
    reason: str,
    has_snap: Callable[[Path, int], bool],
) -> tuple[list[int], list[str]]:
    approved_sys_ids = {int(row["sys_id"]) for row in approvals}
    evidence_errors: list[str] = []
    vehicles = []
    for sys_id in context.sys_ids:
        try:
            snap_observed: bool | None = has_snap(log_dir, sys_id)
        except OSError as read_error:
            snap_observed = None
            evidence_errors.append(f"vehicle {sys_id} SNAP log: {read_error}")
        vehicles.append({
            "sys_id": sys_id,
            "request_observed": sys_id in observed_sys_ids,
            "approval_attempted": sys_id in attempted_sys_ids or sys_id in approved_sys_ids,
            "approval_accepted": sys_id in approved_sys_ids,
            "snap_observed": snap_observed,
        })
    evidence = {
        "status": status,
        "reason": reason,
        "expected_sys_ids": list(context.sys_ids),
        "vehicles": vehicles,
        "approvals": approvals,
    }
    try:
        context.backend_tail.read_new()
    except Exception as tail_error:
        evidence["backend_tail_error"] = f"{type(tail_error).__name__}: {tail_error}"
    for path, content in (
        (log_dir / "demo_confirmation_failure.json", json.dumps(evidence, indent=2)),
        (log_dir / "gcs_backend.partial.log", context.backend_tail.text),
    ):
        try:
            path.write_text(content, encoding="utf-8")
        except OSError as write_error:
            evidence_errors.append(f"{path.name}: {write_error}")
    missing_snap = [row["sys_id"] for row in vehicles if row["snap_observed"] is False]
    return missing_snap, evidence_errors


__all__ = ["persist_confirmation_failure"]
