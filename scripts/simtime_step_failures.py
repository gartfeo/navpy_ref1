"""Prove each negative trial failed for its intended reason, without a relaunch."""

import json
from pathlib import Path
import re


EXPECTED = {
    "boot": ("identity_or_version", 5),
    "step": ("identity_or_version", 5),
    "version": ("identity_or_version", 5),
    "truncated": ("frame_length", 5),
    "oversized": ("frame_length", 5),
    "duplicate": ("identity_or_version", 6),
    "disconnect": ("disconnected", 5),
    "timeout": ("deadline", 5),
}


def validate_failure(case: Path, fault: str) -> dict:
    log = (case / "supervisor.log").read_text(encoding="utf-8", errors="replace")
    failures = re.findall(r"NAVPY_STEP_INVALID (\w+) step=(\d+)", log)
    reason, step = EXPECTED[fault]
    if failures != [(reason, str(step))] or "NAVPY_STEP_COMPLETE" in log:
        raise ValueError(f"wrong failure evidence: {failures}")
    if log.count("Starting sketch 'ArduPlane'") != 1:
        raise ValueError("negative trial must use exactly one simulator boot")
    if (case / "navpy-step.csv").exists():
        raise ValueError("negative trial produced a complete AP window")
    peer = json.loads((case / "peer.json").read_text())
    if peer["fault"] != fault or len(peer["records"]) != 5:
        raise ValueError("peer did not inject the intended fifth-step fault")
    if [row["step"] for row in peer["records"]] != [1, 2, 3, 4, 5]:
        raise ValueError("wrong peer progression before injection")
    return {"reason": reason, "failed_step": step}
