"""Observation cadence inside each final-approach window (annotation, no gate).

Whole-run poll-gap medians hide the approach window, so cadence is measured
per approach episode, all in companion source time (ATTITUDE time_boot, the
obs_ts domain):

- entry: DOCK_ARMED source_ts (NAV entry);
- exit: from the episode's rows up to the next DOCK_ARMED: its DOCKED
  aircraft_ts, else its DOCK_DISARMED source_ts (NAV exit after SNAP or
  abort), else the log end (the last ATTITUDE boot time NavPy received).
  Log order does not decide: the contact thread can log DOCKED after the
  NAV exit's DOCK_DISARMED;
- observations: fresh rows keyed (epoch, obs_ts) with entry <= obs_ts <= exit,
  in arrival order. The epoch is (observation file, vision_nav generation).
  Arrival wall time associates rows with the episode, never measures a gap:
  only rows that arrived between the episode's own DOCK_ARMED and terminal
  rows belong to it. DOCK_ARMED carries the epoch wall time; the episode's
  other rows (local time of day) are placed by their offset from it, so the
  evaluator's timezone and midnight do not matter. An episode with no
  terminal row before the next DOCK_ARMED, or a DOCK_ARMED without a wall
  time (older logs), is reported without statistics. A repeated key counts once; a key not after the previous
  one is out_of_order and not used. In-window rows of two epochs are reported
  as ambiguous, not merged. A restart inside the window (another epoch
  arriving after the episode's, or a terminal source time before the entry)
  ends the segment at its epoch's last observation, with no gap across it.

Gaps include entry->first and last->exit. Nothing here touches a verdict or a
denominator: errors become the annotation's status, never report errors.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from scripts.eval_certificate_source_streams import read_rows
from scripts.eval_certificate_source_time import SOURCE_TIME_FILE_RE, SOURCE_TIME_SUBDIR
from scripts.eval_certificate_statistics import percentile
from scripts.eval_gcs_demo_docked import parse_dock_armed, parse_docked
from scripts.eval_gcs_demo_tokens import finite_number, payload_fields, seconds_of_day

# Gaps longer than this count against coverage. Triage only, not a gate: 100 ms
# is three frames of the 30 Hz detector period.
COVERAGE_GAP_BOUND_MS = 100.0
# ATTITUDE time_boot is whole milliseconds, so gaps are rounded to 1 us: a
# nominal 100 ms gap must not exceed the bound by float noise.
_GAP_DECIMALS = 3
_DISARMED_FIELDS = ("vehicle", "task", "dock", "episode", "source_ts")
_TERMINAL_HINTS = {"EVENT:SNAP_COMPONENTS": "snap", "EVENT:ABORT": "abort"}
_DAY_S = 86400.0
UNTERMINATED = "no terminal row before the next DOCK_ARMED"


@dataclass(frozen=True)
class Window:
    """One armed episode's source-time bounds; ``exit_s`` None = unknown."""

    episode: int
    entry_s: float | None
    exit_s: float | None
    ended_by: str
    # Epoch wall times of the DOCK_ARMED and terminal rows; exit None = open.
    entry_wall_s: float | None = None
    exit_wall_s: float | None = None

    def holds_arrival(self, wall_s: float) -> bool:
        if self.entry_wall_s is None or wall_s < self.entry_wall_s:
            return False
        return self.exit_wall_s is None or wall_s <= self.exit_wall_s


#: One fresh observation: (epoch, obs_ts, arrival epoch wall seconds).
Observation = tuple[object, float, float]


def cadence_annotation(log_dir: Path, sys_id: int) -> dict[str, Any]:
    """The per-UAV annotation for report.json; never raises for bad evidence."""
    try:
        windows = approach_windows(
            log_dir / f"uav_{sys_id}_navigation_debug.csv",
            _log_end_s(log_dir / SOURCE_TIME_SUBDIR, sys_id),
        )
        observations = _observations(log_dir / SOURCE_TIME_SUBDIR, sys_id)
    except (OSError, ValueError) as exc:
        return _annotation(f"unavailable: {exc}", [])
    if not windows:
        return _annotation("approach not entered", [])
    return _annotation("ok", [episode_cadence(window, observations) for window in windows])


def approach_windows(debug_csv: Path, log_end_s: float | None) -> list[Window]:
    """Armed episodes in log order, each closed from its own terminal rows."""
    blocks: list[dict[str, Any]] = []
    with debug_csv.open("r", encoding="utf-8", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            event = row[1] if len(row) > 1 else ""
            if event == "EVENT:DOCK_ARMED":
                blocks.append({"armed": parse_dock_armed(row, line_number),
                               "entry_day_s": seconds_of_day(row[0])})
            elif not blocks:
                continue
            elif event == "EVENT:DOCKED":
                docked = parse_docked(row, line_number)
                if docked.identity == blocks[-1]["armed"].identity:
                    blocks[-1].setdefault("docked", (docked.aircraft_ts, docked.wall_s))
            elif event in _TERMINAL_HINTS:
                blocks[-1].setdefault("hint", _TERMINAL_HINTS[event])
            elif event == "EVENT:DOCK_DISARMED":
                if len(row) < 3:
                    raise ValueError(f"DOCK_DISARMED row {line_number} has fewer than 3 columns")
                payload = payload_fields(row[2], _DISARMED_FIELDS, "DOCK_DISARMED")
                identity = tuple(int(payload[name]) for name in _DISARMED_FIELDS[:4])
                if identity == blocks[-1]["armed"].identity:
                    blocks[-1].setdefault("disarmed", (finite_number(
                        "source_ts", payload["source_ts"], optional=True
                    ), seconds_of_day(row[0])))
    return [
        _window(block, log_end_s, index == len(blocks) - 1)
        for index, block in enumerate(blocks)
    ]


def _window(block: dict[str, Any], log_end_s: float | None, last: bool) -> Window:
    armed = block["armed"]
    if "docked" in block:
        (exit_s, exit_day_s), ended_by = block["docked"], "docked"
    elif "disarmed" in block:
        (exit_s, exit_day_s), ended_by = block["disarmed"], block.get("hint", "nav_exit")
    elif last:
        exit_s, exit_day_s, ended_by = log_end_s, None, "log_end"
    else:
        return Window(armed.episode, armed.source_ts, None, UNTERMINATED, armed.wall_s)
    exit_wall = None
    if armed.wall_s is not None and exit_day_s is not None:
        # The row clock is local time of day: offset from the armed row, mod a day.
        exit_wall = armed.wall_s + (exit_day_s - block["entry_day_s"]) % _DAY_S
    return Window(armed.episode, armed.source_ts, exit_s, ended_by, armed.wall_s, exit_wall)


def episode_cadence(
    window: Window,
    observations: Sequence[Observation],
    bound_ms: float = COVERAGE_GAP_BOUND_MS,
) -> dict[str, Any]:
    """Gap statistics of one window; ``observations`` in arrival order."""
    result: dict[str, Any] = {
        "episode": window.episode, "entry_s": window.entry_s,
        "exit_s": window.exit_s, "ended_by": window.ended_by,
    }
    if window.ended_by == UNTERMINATED:
        return {**result, "status": UNTERMINATED}
    if window.entry_s is None or window.exit_s is None:
        return {**result, "status": "source time unavailable"}
    if window.entry_wall_s is None:
        return {**result, "status": "arrival time unavailable"}
    restarted = window.exit_s < window.entry_s
    upper_s = float("inf") if restarted else window.exit_s
    arrived = [row for row in observations if window.holds_arrival(row[2])]
    in_window = [row for row in arrived if window.entry_s <= row[1] <= upper_s]
    epochs = list(dict.fromkeys(row[0] for row in in_window))
    if len(epochs) > 1:
        return {**result, "status": "ambiguous source epoch", "epochs": len(epochs)}
    if epochs:
        first = arrived.index(in_window[0])
        restarted = restarted or any(row[0] != epochs[0] for row in arrived[first:])
    times, counts = _fresh_times([(row[0], row[1]) for row in in_window])
    result.update(counts, observations=len(times))
    if restarted:
        result["ended_by"] = "source_restart"
    if len(times) < 2:
        return {**result, "status": f"entered with {len(times)} observation(s)"}
    # A restart ends the segment at its last observation: no gap across it.
    end_s = times[-1] if restarted else window.exit_s
    points = [window.entry_s, *times] + ([] if restarted else [end_s])
    gaps = [
        round((later - earlier) * 1000.0, _GAP_DECIMALS)
        for earlier, later in zip(points, points[1:])
    ]
    span_ms = (end_s - window.entry_s) * 1000.0
    longest = max(range(len(gaps)), key=gaps.__getitem__)
    over = sum(gap for gap in gaps if gap > bound_ms)
    return {
        **result,
        "status": "ok",
        "gap_count": len(gaps),
        "gap_median_ms": percentile(gaps, 50.0),
        "gap_p90_ms": percentile(gaps, 90.0),
        "longest_gap_ms": gaps[longest],
        "longest_gap_from_s": points[longest],
        "longest_gap_to_s": points[longest + 1],
        "coverage": None if span_ms <= 0.0 else 1.0 - over / span_ms,
    }


def _fresh_times(
    keys: Sequence[tuple[object, float]],
) -> tuple[list[float], dict[str, int]]:
    """One epoch's keys in arrival order: duplicates once, out of order dropped."""
    times: list[float] = []
    seen: set[tuple[object, float]] = set()
    duplicates = out_of_order = 0
    for key in keys:
        if key in seen:
            duplicates += 1
            continue
        seen.add(key)
        if times and key[1] <= times[-1]:
            out_of_order += 1
            continue
        times.append(key[1])
    return times, {"duplicates": duplicates, "out_of_order": out_of_order}


def _observations(source_dir: Path, sys_id: int) -> list[Observation]:
    """Fresh observations of every file, in arrival (wall) order."""
    rows: list[tuple[float, Observation]] = []
    for index, path in enumerate(_stream_files(source_dir, "observation", sys_id)):
        for row in read_rows(path, "observation"):
            if row["outcome"] != "fresh":
                continue
            wall_s = finite_number("wall_s", row["wall_s"])
            obs_ts = finite_number("obs_ts", row["obs_ts"])
            rows.append((wall_s, (  # type: ignore[arg-type]
                (index, row.get("epoch", "")), obs_ts, wall_s,
            )))
    rows.sort(key=lambda item: item[0])
    return [observation for _wall, observation in rows]


def _log_end_s(source_dir: Path, sys_id: int) -> float | None:
    """The boot time of the last ATTITUDE NavPy received (latest wall_s), in s.

    The last received, not the largest: after a restart they differ."""
    arrivals = [
        (finite_number("wall_s", row["wall_s"]), finite_number("boot_ms", row["boot_ms"]))
        for path in _stream_files(source_dir, "attitude_arrival", sys_id, navpy_only=True)
        for row in read_rows(path, "attitude_arrival")
        if row["boot_ms"].strip()
    ]
    if not arrivals:
        return None
    return max(arrivals, key=lambda arrival: arrival[0])[1] / 1000.0  # type: ignore[operator]


def _stream_files(
    source_dir: Path, stream: str, sys_id: int, *, navpy_only: bool = False
) -> list[Path]:
    """The UAV's files of one stream; ``navpy_only`` drops other producers (gcs)."""
    if not source_dir.is_dir():
        return []
    files = []
    for path in sorted(source_dir.iterdir()):
        match = SOURCE_TIME_FILE_RE.fullmatch(path.name)
        if match is None or match.group("stream") != stream or int(match.group("sys")) != sys_id:
            continue
        if navpy_only and match.group("producer") not in (None, "navpy"):
            continue
        files.append(path)
    return files


def _annotation(status: str, episodes: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "status": status,
        "clock": "companion source time (ATTITUDE time_boot)",
        "coverage_gap_bound_ms": COVERAGE_GAP_BOUND_MS,
        "gate": False,
        "episodes": episodes,
    }


__all__ = [
    "COVERAGE_GAP_BOUND_MS",
    "Observation",
    "UNTERMINATED",
    "Window",
    "approach_windows",
    "cadence_annotation",
    "episode_cadence",
]
