"""Evaluator log discovery, readiness, and SNAP parsing."""

from __future__ import annotations

import re
import time
from collections.abc import Sequence
from pathlib import Path


NAVPY_LOG_SUBDIR = "navpy-logs"


def wait_for_text(path: Path, needles: Sequence[str], timeout_s: float) -> bool:
    """Wait until a text file contains every requested marker."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if path.exists():
            try:
                contents = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                contents = ""
            if all(needle in contents for needle in needles):
                return True
        time.sleep(0.25)
    return False


def wait_for_ready_text(path: Path, timeout_s: float) -> bool:
    """Wait for both navigation and detector readiness markers."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if path.exists():
            try:
                contents = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                contents = ""
            if "N: 40.0s" in contents and (
                "Detector: Start Detection" in contents
                or "DetectionCoordinator: Started" in contents
            ):
                return True
        time.sleep(0.25)
    return False


def navpy_log_paths(run_case_dir: Path, sysid: int) -> tuple[Path, Path]:
    """Return process-bound compact and navigation log paths."""
    log_dir = run_case_dir / NAVPY_LOG_SUBDIR
    return (
        log_dir / f"uav_{sysid}_navigation_compact.csv",
        log_dir / f"uav_{sysid}_navigation.log",
    )


def snap_from_compact(path: Path | None) -> str | None:
    """Return the final compact SNAP row, when present."""
    if path is None:
        return None
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    snaps = [line for line in lines if "SNAP(" in line]
    return snaps[-1] if snaps else None


def parse_snap_summary(snap_line: str | None) -> dict[str, float]:
    """Parse compact SNAP distance components."""
    if not snap_line:
        return {}
    match = re.search(
        r"3d=(?P<dist>[-+]?\d+(?:\.\d+)?)\(h=(?P<h>[-+]?\d+(?:\.\d+)?);\s*"
        r"v=(?P<v>[-+]?\d+(?:\.\d+)?)\)",
        snap_line,
    )
    if not match:
        return {}
    return {
        "dist_3d_m": float(match.group("dist")),
        "h_m": float(match.group("h")),
        "v_m": float(match.group("v")),
    }


def parse_snap_components(debug_path: Path | None) -> dict[str, float | str]:
    """Parse the detailed SNAP_COMPONENTS sidecar event."""
    if debug_path is None or not debug_path.exists():
        return {}
    snap_line = None
    with debug_path.open(
        "r", encoding="utf-8", errors="replace", newline=""
    ) as handle:
        for line in handle:
            if "EVENT:SNAP_COMPONENTS" in line:
                snap_line = line.strip()
    if not snap_line:
        return {}
    payload = re.search(r"EVENT:SNAP_COMPONENTS,(.+?)(?:,{2,}|$)", snap_line)
    if not payload:
        return {}
    values: dict[str, float | str] = {}
    for part in payload.group(1).split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        try:
            values[key.strip()] = float(value.strip())
        except ValueError:
            values[key.strip()] = value.strip()
    return values


def sidecar_paths(compact_path: Path | None) -> tuple[Path | None, Path | None]:
    """Derive debug and verbose navigation sidecars from a compact path."""
    if compact_path is None:
        return None, None
    compact_text = str(compact_path)
    return (
        Path(compact_text.replace("_compact.csv", "_debug.csv")),
        Path(compact_text.replace("_compact.csv", ".log")),
    )


def post_snap_poi_seen(log_path: Path | None) -> bool | None:
    """Return whether another POI episode appeared after SNAP."""
    if log_path is None or not log_path.exists():
        return None
    seen_snap = False
    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "SNAP(" in line:
                seen_snap = True
            elif seen_snap and "POI:" in line:
                return True
    return False
