"""Attitude arrival lag per reader: the seed sweep's check on todos 199 and 200.

No gate sees a reader that falls behind: a lagging reader can still dock
(``todo65_root_cause.md``, "Stress seed 212"). This reads the pose-cadence
files every run writes (``source-time/pose_cadence_attitude_arrival_uav_<n>_<side>.csv``,
``wall_s`` the sim clock at arrival and ``boot_ms`` the aircraft's time of the
message) and gives each reader's lag: arrival minus aircraft time, less that
file's smallest value, so 0 is the reader's best delivery. ``navpy`` is UAV
n's NavPy reader, ``gcs`` the backend's for UAV n.

Rows inside a scheduled fault that stops a NavPy's work (``late_heartbeat``,
``navpy_restart``; ``fault_schedule``) are left out for that NavPy, from the
fault's start to its recovery deadline: the freeze is the fault, not lag. A
link drop or partition loses packets and delays none.
"""

from __future__ import annotations

import csv
import json
import re
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from scripts.eval_gcs_demo_lockstep import EPOCH_US, JOURNAL_FILE

#: A reader this far behind its best delivery is falling behind, not jittering. Measured
#: maxima of healthy runs: NavPy 361 ms, backend 260 ms at B 6 and above; the failure mode
#: reached 3-27 s (``todo65_root_cause.md``, "Why low B fails"). 0.5 s is the link jitter
#: budget the product is built for (``guidance_delivery_fix.md``).
READER_LAG_LIMIT_MS = 500.0
ARRIVAL_DIR = "source-time"
_FILE = re.compile(r"pose_cadence_attitude_arrival_uav_(\d+)_(navpy|gcs)\.csv")
#: Faults that hold a NavPy's threads, so its reader, by design.
_HOLDING_FAULTS = ("late_heartbeat", "navpy_restart")


def holding_windows(records: Iterable[Mapping[str, Any]]) -> dict[int, list[tuple[float, float]]]:
    """UAV -> its (start, recovery deadline) windows in sim seconds, from the journal's fault records."""
    kinds: dict[str, tuple[str, int]] = {}
    starts: dict[str, float] = {}
    windows: dict[int, list[tuple[float, float]]] = {}
    for record in records:
        for plan in record.get("grid", {}).get("faults", ()):
            match = re.fullmatch(r"uav(\d+)", str(plan.get("target")))
            if plan.get("kind") in _HOLDING_FAULTS and match:
                kinds[plan["id"]] = (plan["kind"], int(match.group(1)))
        for event in record.get("faults", ()) if isinstance(record.get("faults"), list) else ():
            if event[1] not in kinds:
                continue
            if event[0] == "start":
                starts[event[1]] = record["t_us"] / 1e6
            elif event[0] == "heal" and event[1] in starts:
                windows.setdefault(kinds[event[1]][1], []).append((starts[event[1]], event[2] / 1e6))
    return windows


def _lags(path: Path, windows: list[tuple[float, float]]) -> list[float]:
    with path.open(encoding="utf-8", newline="") as rows:
        pairs = [(float(row["wall_s"]) - EPOCH_US / 1e6, float(row["boot_ms"])) for row in csv.DictReader(rows)]
    lags = [t * 1000 - boot for t, boot in pairs if not any(a <= t <= b for a, b in windows)]
    floor = min(lags, default=0.0)
    return sorted(lag - floor for lag in lags)


def reader_lag(run_dir: Path) -> dict[str, dict[str, float]]:
    """``"<side> <uav>"`` -> its lag p50, p99 and max in ms; empty when the run wrote no arrival files."""
    windows: dict[int, list[tuple[float, float]]] = {}
    journal = run_dir / JOURNAL_FILE
    if journal.is_file():
        with journal.open(encoding="utf-8") as lines:
            windows = holding_windows(json.loads(line) for line in lines)
    result = {}
    for path in sorted((run_dir / ARRIVAL_DIR).glob("pose_cadence_attitude_arrival_uav_*.csv")):
        match = _FILE.fullmatch(path.name)
        if match is None:
            continue
        uav, side = int(match.group(1)), match.group(2)
        lags = _lags(path, windows.get(uav, []) if side == "navpy" else [])
        if lags:
            result[f"{side} {uav}"] = {"p50": round(lags[len(lags) // 2], 1),
                                       "p99": round(lags[int(0.99 * (len(lags) - 1))], 1),
                                       "max": round(lags[-1], 1)}
    return result


def lagging_readers(lag: Mapping[str, Mapping[str, float]]) -> list[str]:
    """The readers whose lag passed :data:`READER_LAG_LIMIT_MS`."""
    return [name for name, stats in lag.items() if stats["max"] > READER_LAG_LIMIT_MS]


__all__ = ["READER_LAG_LIMIT_MS", "holding_windows", "lagging_readers", "reader_lag"]
