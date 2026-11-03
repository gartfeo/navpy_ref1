"""What a wall run's logs say about NavPy stalls and failures (dev sim step 9).

``docs/design/deterministic_dev_sim.md``, section 3 (B). A wall run writes,
per UAV, the wall time each ATTITUDE reached the NavPy and the GCS backend
(``source-time/pose_cadence_attitude_arrival_uav_<n>_<navpy|gcs>.csv``, or one
file per process, ``..._pid_<pid>.csv``, in older runs), with the packet's
``time_boot_ms``. The GCS's latest packet at a wall time is the aircraft's time
then, so a NavPy packet's lag in aircraft time is that time less its own boot
time, whatever the wall speedup and whether the SITL itself kept pace. The
median lag is the path's own offset; a **stall** is a stretch where the NavPy's
lag rises :data:`STALL_MIN_MS` or more above it, and its length is the peak.
Its **rise** is the aircraft time from its first late packet to the peak and
its **drain** from the peak to its last late packet. A NavPy that read nothing
(a freeze, a blocked reader) shows a lag that jumps at once and drains: the
lockstep ``process_freeze`` model. One whose lag grew slower than aircraft time
passed (:attr:`Stall.slow`) kept receiving while it fell behind: the
``reader_lag`` model (todo 65).

Failures are what failed on the wall: each UAV's ``navigation.log``
(``NAV_FAIL``/``DOCK_FAILED`` lines) and the run's ``report.json`` gate errors,
as :func:`failure_kinds`, which reads a lockstep run's errors the same way. A
long source-time gap in the cadence episodes is no failure: the wall gate
measures wall time, and at 10x a 740 ms source gap is 74 ms of it.
"""

from __future__ import annotations

import bisect
import csv
import json
import re
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from datetime import datetime
from functools import cached_property
from pathlib import Path
from statistics import median

from scripts.eval_gcs_demo_limits import CadenceLimits

ARRIVALS = "pose_cadence_attitude_arrival_uav_{uav}_"
#: A stall: NavPy lag this far above its median. At the wall's 10x a 10 ms wall hiccup (Windows' 15.6 ms
#: timer) is 100 ms of aircraft time, and the quiet wall runs (``.logs/e10/r2``, ``r3``) reach 200 ms once
#: per UAV in about 860 s; the loaded ones (``live``, ``r4``) 70-160 times.
STALL_MIN_MS = 200
#: A GCS packet older than this at a NavPy arrival does not give the aircraft's time (a GCS gap).
GROUND_STALE_S = 0.2
#: The observation-gap gate (``max observation gap``), in aircraft time as lockstep scores it.
GAP_GATE_MS = round(1000 * CadenceLimits().max_wall_gap_s)
_LOG_TIME = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3}) \w+ (.*)$")
_NAV_ENTRY = "INIT: NAV MODE"
#: Failure kinds, as the product logs them and the eval's gates word them.
KINDS: dict[str, re.Pattern[str]] = {
    "no_detection": re.compile(r"(NAV_FAIL|DOCK_FAILED): no_detection"),
    "source_command_stale": re.compile(r"NAV_FAIL: source_command_stale"),
    "roll_step": re.compile(r"max roll-command step [\d.]+deg > [\d.]+deg"),
    "observation_gap": re.compile(r"(median|max) observation gap [\d.]+s > [\d.]+s"),
}
_NAV_FAILURE = re.compile(r"(NAV_FAIL|DOCK_FAILED): \w+")
#: A NAV window's end: a failure, a dock, or a passed-POI reset.
_NAV_END = re.compile(r"NAV_FAIL|DOCK_FAILED|DOCKED|RESET: PASSED POI")


@dataclass(frozen=True)
class Stall:
    uav: int
    #: Aircraft time (``time_boot_ms``) of the first late packet and the stall's peak lag above the median.
    boot_ms: int
    length_ms: int
    #: Aircraft time from the first late packet to the peak, and from the peak to the last late packet.
    rise_ms: int = 0
    drain_ms: int = 0

    @property
    def slow(self) -> bool:
        """The lag grew slower than aircraft time passed (a silent reader's grows as fast), so the
        reader kept receiving while it fell behind."""
        return self.rise_ms > self.length_ms


@dataclass(frozen=True)
class WallFailure:
    uav: int
    kind: str
    text: str
    #: Aircraft seconds from the UAV's NAV entry to the failure line, when both map to a packet.
    after_nav_s: float | None = None


@dataclass(frozen=True)
class UavArrivals:
    uav: int
    #: (wall s, boot ms) per packet, in arrival order.
    navpy: Sequence[tuple[float, int]]
    ground: Sequence[tuple[float, int]]

    @cached_property
    def _ground_walls(self) -> list[float]:
        return [wall for wall, _ in self.ground]

    def boot_at(self, wall_s: float) -> int | None:
        """The aircraft time at a wall time: the GCS's latest packet, if fresh."""
        i = bisect.bisect_right(self._ground_walls, wall_s) - 1
        if i < 0 or wall_s - self.ground[i][0] > GROUND_STALE_S:
            return None
        return self.ground[i][1]


def failure_kinds(texts: Iterable[str]) -> set[str]:
    return {kind for text in texts for kind, pattern in KINDS.items() if pattern.search(text)}


def _arrivals(path: Path) -> list[tuple[float, int]]:
    with path.open(newline="", encoding="utf-8") as rows:
        return [(float(row["wall_s"]), int(row["boot_ms"])) for row in csv.DictReader(rows)]


def _overlap(a: Sequence[tuple[float, int]], b: Sequence[tuple[float, int]]) -> float:
    if not a or not b:
        return 0.0
    return max(0.0, min(a[-1][0], b[-1][0]) - max(a[0][0], b[0][0]))


def uav_arrivals(run_dir: Path, uav: int) -> list[UavArrivals]:
    """Each NavPy process's arrivals with the GCS's. Per-process files: a process that wrote arrivals of
    every UAV is a ground process, the others NavPys; each NavPy is paired with the ground file it
    overlaps most in wall time."""
    times = run_dir / "source-time"
    prefix = ARRIVALS.format(uav=uav)
    navpy, ground = times / f"{prefix}navpy.csv", times / f"{prefix}gcs.csv"
    if navpy.is_file() and ground.is_file():
        return [UavArrivals(uav, _arrivals(navpy), _arrivals(ground))]
    uavs = {int(p.name.split("_uav_")[1].split("_")[0]) for p in times.glob("pose_cadence_attitude_arrival_uav_*_pid_*.csv")}
    pids = {u: {p.stem.rsplit("_pid_", 1)[1] for p in times.glob(f"{ARRIVALS.format(uav=u)}pid_*.csv")} for u in uavs}
    grounds = set.intersection(*pids.values()) if len(uavs) > 1 else set()
    files = {pid: _arrivals(times / f"{prefix}pid_{pid}.csv") for pid in pids.get(uav, ())}
    pairs = []
    for pid in sorted(set(files) - grounds):
        best = max(grounds, key=lambda g: _overlap(files[pid], files[g]), default=None)
        if best is not None and _overlap(files[pid], files[best]) > 0:
            pairs.append(UavArrivals(uav, files[pid], files[best]))
    return pairs


def stalls(arrivals: UavArrivals) -> list[Stall]:
    """The NavPy's stalls (module docstring), in aircraft-time order."""
    lags = [(boot, now - boot) for wall, boot in arrivals.navpy if (now := arrivals.boot_at(wall)) is not None]
    if not lags:
        return []
    base = median(lag for _, lag in lags)
    found: list[Stall] = []
    start = peak_at = last = None
    peak = 0
    for boot, lag in [*lags, (None, base)]:
        excess = lag - base
        if excess >= STALL_MIN_MS:
            start = boot if start is None else start
            if excess > peak:
                peak, peak_at = excess, boot
            last = boot
        elif start is not None:
            found.append(Stall(arrivals.uav, start, round(peak), peak_at - start, last - peak_at))
            start = peak_at = last = None
            peak = 0
    return found


def span_s(arrivals: UavArrivals) -> float:
    """Aircraft seconds the NavPy's arrivals cover."""
    boots = [boot for _, boot in arrivals.navpy]
    return (max(boots) - min(boots)) / 1000.0 if boots else 0.0


def _log_lines(path: Path) -> list[tuple[float, str]]:
    lines = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = _LOG_TIME.match(line)
        if match:
            when = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S,%f").timestamp()
            lines.append((when, match.group(2)))
    return lines


def nav_windows(run_dir: Path, uav: int, pairs: Sequence[UavArrivals]) -> list[tuple[float, float]]:
    """The UAV's NAV windows in aircraft seconds: each NAV entry to its end line (a NAV failure, a
    dock or a pass reset), or to its last mapped log line."""
    log = run_dir / f"uav_{uav}_navigation.log"
    windows: list[tuple[float, float]] = []
    entry: float | None = None
    now: float | None = None
    for wall, text in _log_lines(log) if log.is_file() else []:
        now = _boot_s(pairs, wall) or now
        if _NAV_ENTRY in text and now is not None:
            entry = now
        elif entry is not None and now is not None and _NAV_END.search(text):
            windows.append((entry, now))
            entry = None
    if entry is not None and now is not None:
        windows.append((entry, now))
    return windows


def _boot_s(pairs: Sequence[UavArrivals], wall_s: float) -> float | None:
    for pair in pairs:
        boot = pair.boot_at(wall_s)
        if boot is not None:
            return boot / 1000.0
    return None


def wall_failures(run_dir: Path, uav: int, pairs: Sequence[UavArrivals] = ()) -> list[WallFailure]:
    """The UAV's logged NAV failures, with their aircraft time after the NAV entry before them, then its
    report's gate errors of a known kind."""
    failures = []
    log = run_dir / f"uav_{uav}_navigation.log"
    entry: float | None = None
    for wall, text in _log_lines(log) if log.is_file() else []:
        if _NAV_ENTRY in text:
            entry = _boot_s(pairs, wall)
        match = _NAV_FAILURE.search(text)
        if match and failure_kinds([text]):
            now = _boot_s(pairs, wall)
            after = None if entry is None or now is None else round(now - entry, 3)
            failures.append(WallFailure(uav, failure_kinds([text]).pop(), match.group(0), after))
    report = _json(run_dir / "report.json")
    for vehicle in report.get("vehicles", []) if isinstance(report, dict) else []:
        if vehicle.get("sys_id") == uav:
            failures += [WallFailure(uav, kind, error) for error in vehicle.get("errors", [])
                         for kind in sorted(failure_kinds([error]))]
    return failures


def _json(path: Path) -> object:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


__all__ = ["GAP_GATE_MS", "KINDS", "STALL_MIN_MS", "Stall", "UavArrivals", "WallFailure", "failure_kinds", "span_s",
           "nav_windows", "stalls", "uav_arrivals", "wall_failures"]
