"""One UAV's part of ``delivery_demo_check``: its log, approaches and source time."""

from __future__ import annotations

import re
from collections import Counter
from collections.abc import Callable
from pathlib import Path
from typing import Any, NamedTuple

from scripts.eval_certificate_source_time import (
    SOURCE_TIME_FILE_RE,
    SOURCE_TIME_SUBDIR,
    source_time_summary,
)
from scripts.eval_gcs_demo_cadence import cadence_annotation
from scripts.eval_gcs_demo_evidence import mission_navigation_segment, parse_final_approach_outcomes
from scripts.eval_gcs_demo_gates import sensor_config_errors

NO_END = "no DOCKED or DOCK_FAILED row"
#: The lines that tell the story of a flight; WARNING and ERROR lines always count.
_LOG_EVENT_RE = re.compile(
    r"NAV_FAIL:|DOCKED \(sim\)|STATE |SIM_CLOSEST_APPROACH:|Asking confirmation"
    r"|POI \d+ .*(?:confirmed|rejected)|REJECTED:|RESET:|INIT: NAV MODE|Default delivery hub navigation"
)
_LOG_LINE_RE = re.compile(r"^\S+ (?P<time>\d\d:\d\d:\d\d),\d+ (?P<level>[A-Z]+) (?P<message>.*)$")
_PROCESS_START = "Heartbeat from system"
_UAV_FILE_RE = re.compile(r"^uav_(\d+)_")
_ARRIVAL_PRODUCERS = ("navpy", "gcs")


class Outcome(NamedTuple):
    """One approach as the debug log records it."""

    name: str
    result: str
    detail: str


def navigation_log(run_dir: Path, uav: int) -> Path:
    return run_dir / f"uav_{uav}_navigation.log"


def debug_csv(run_dir: Path, uav: int) -> Path:
    return run_dir / f"uav_{uav}_navigation_debug.csv"


def guarded(title: str, section: Callable[..., list[str]], *args: object) -> list[str]:
    """One section's lines; a section that cannot read its evidence only says so."""
    try:
        return section(*args)
    except Exception as error:  # noqa: BLE001 - partial evidence must not stop the report
        return [f"{title}: could not read: {error}"]


def uav_ids(run_dir: Path) -> list[int]:
    ids = {int(match.group(1)) for match in map(_UAV_FILE_RE.match, _names(run_dir)) if match}
    source_files = map(SOURCE_TIME_FILE_RE.fullmatch, _names(run_dir / SOURCE_TIME_SUBDIR))
    ids.update(int(match.group("sys")) for match in source_files if match)
    return sorted(ids)


def _names(directory: Path) -> list[str]:
    return sorted(path.name for path in directory.iterdir()) if directory.is_dir() else []


def _episode_name(identity: tuple[int, int, int, int] | None) -> str:
    if identity is None:
        return "episode with no DOCK_ARMED row"
    _vehicle, task, dock, episode = identity
    return f"episode {episode} task {task} dock {dock}"


def approach_outcomes(path: Path, uav: int) -> list[Outcome]:
    episodes, failures = parse_final_approach_outcomes(path, vehicle_id=uav)
    reasons = {failure.identity: failure.reason for failure in failures}
    outcomes = []
    for episode in episodes:
        identity = episode.armed.identity if episode.armed is not None else None
        name = _episode_name(identity)
        commands = f"{len(episode.commands)} command rows"
        row = episode.docked_row
        if row is not None:
            detail = (
                f"closest {row.closest_3d_m:.2f} m (h {row.closest_h_m:.2f}, "
                f"v {row.closest_v_m:.2f}), {commands}"
            )
            outcomes.append(Outcome(name, "DOCKED", detail))
        elif identity in reasons:
            outcomes.append(Outcome(name, f"DOCK_FAILED ({reasons.pop(identity)})", commands))
        else:
            outcomes.append(Outcome(name, NO_END, commands))
    # A failure with no command row has no episode of its own.
    outcomes += [
        Outcome(_episode_name(identity), f"DOCK_FAILED ({reason})", "0 command rows")
        for identity, reason in reasons.items()
    ]
    return outcomes


def _approach_lines(run_dir: Path, uav: int) -> list[str]:
    path = debug_csv(run_dir, uav)
    if not path.is_file():
        return [f"approaches: {path.name} is missing"]
    outcomes = approach_outcomes(path, uav)
    if not outcomes:
        return ["approaches: none ended in the debug log"]
    return [f"approach: {item.name}: {item.result}, {item.detail}" for item in outcomes]


def _log_lines(run_dir: Path, uav: int) -> list[str]:
    path = navigation_log(run_dir, uav)
    if not path.is_file():
        return [f"log: {path.name} is missing"]
    rows = path.read_text(encoding="utf-8", errors="replace").splitlines()
    parsed = [match for match in map(_LOG_LINE_RE.match, rows) if match]
    levels = Counter(match["level"] for match in parsed)
    starts = sum(_PROCESS_START in match["message"] for match in parsed)
    lines = [
        f"log: {len(rows)} lines, {starts} NavPy starts, "
        f"{levels['WARNING']} WARNING, {levels['ERROR'] + levels['CRITICAL']} ERROR"
    ]
    lines += [f"log: older text not read: {name}" for name in _names(run_dir) if name.startswith(f"{path.name}.")]
    lines += [
        f"  {match['time']} {match['message']}" for match in parsed
        if match["level"] not in ("INFO", "DEBUG") or _LOG_EVENT_RE.match(match["message"])
    ]
    return lines


def _cadence_lines(run_dir: Path, uav: int) -> list[str]:
    annotation = cadence_annotation(run_dir, uav)
    if not annotation["episodes"]:
        return [f"cadence: {annotation['status']}"]
    return [_cadence_line(episode) for episode in annotation["episodes"]]


def _cadence_line(episode: dict[str, Any]) -> str:
    text = f"cadence: episode {episode['episode']}, ended by {episode['ended_by']}, {episode['status']}"
    if episode["status"] != "ok":
        return text
    coverage = episode["coverage"]
    return (
        f"{text}, {episode['observations']} observations, gap median {episode['gap_median_ms']:.1f} "
        f"p90 {episode['gap_p90_ms']:.1f} longest {episode['longest_gap_ms']:.1f} ms, "
        f"coverage {'unknown' if coverage is None else format(coverage, '.3f')}"
    )


def _ms(stats: dict[str, Any] | None, *fields: str) -> str:
    if stats is None:
        return "no data"
    return " ".join(f"{field} {stats[field]:.1f}" for field in fields) + " ms"


def _arrival_text(stream: dict[str, Any]) -> str:
    return (
        f"{stream['rows']} rows, wall gap {_ms(stream['wall_gap_ms'], 'p50', 'p99', 'max')}, "
        f"source gap {_ms(stream['source_gap_ms'], 'p50', 'max')} "
        "(whole file, so a gap across a NavPy restart counts)"
    )


def _observation_text(stream: dict[str, Any]) -> str:
    return f"{stream['rows']} rows {stream['outcomes']}, age {_ms(stream['age_ms'], 'p50', 'p95', 'max')}"


def _worker_text(stream: dict[str, Any]) -> str:
    return (
        f"{stream['rows']} rows {stream['outcomes']}, source age {_ms(stream['age_ms'], 'p50', 'p95', 'max')}, "
        f"wall gap {_ms(stream['wall_gap_ms'], 'p50', 'p99', 'max')}"
    )


def _reject_text(stream: dict[str, Any]) -> str:
    return f"{stream['rows']} rows {stream['counts']}"


#: The streams this check reads. The detector ones are the largest and left out.
_STREAM_TEXT: dict[str, Callable[[dict[str, Any]], str]] = {
    "attitude_arrival": _arrival_text,
    "telemetry_reject": _reject_text,
    "observation": _observation_text,
    "worker": _worker_text,
}


def _source_file_line(directory: Path, uav: int, name: str, stream: str, label: str) -> list[str]:
    summary = source_time_summary(directory, sys_id=uav, include_files=[name])
    return [f"{label}: {_STREAM_TEXT[stream](summary['streams'][stream])}"]


def _approach_entered(run_dir: Path, uav: int) -> bool:
    path = debug_csv(run_dir, uav)
    return path.is_file() and "EVENT:DOCK_ARMED" in path.read_text(encoding="utf-8", errors="replace")


def _source_time_lines(run_dir: Path, uav: int) -> list[str]:
    directory = run_dir / SOURCE_TIME_SUBDIR
    if not directory.is_dir():
        return [f"source time: {SOURCE_TIME_SUBDIR}/ is missing"]
    lines: list[str] = []
    found: set[str] = set()
    for name in _names(directory):
        match = SOURCE_TIME_FILE_RE.fullmatch(name)
        if match is None or int(match["sys"]) != uav or match["stream"] not in _STREAM_TEXT:
            continue
        label = f"{match['stream']} {match['producer'] or 'pid ' + str(match['pid'])}"
        found.add(label)
        lines += guarded(label, _source_file_line, directory, uav, name, match["stream"], label)
    expected = [f"attitude_arrival {producer}" for producer in _ARRIVAL_PRODUCERS]
    if _approach_entered(run_dir, uav):
        expected += ["observation navpy", "worker navpy"]
    lines += [f"{label}: file is missing" for label in expected if label not in found]
    return lines


def _sensor_lines(run_dir: Path, uav: int) -> list[str]:
    path = navigation_log(run_dir, uav)
    if not path.is_file():
        return [f"sensor config: {path.name} is missing"]
    text = path.read_text(encoding="utf-8", errors="replace")
    errors = sensor_config_errors(mission_navigation_segment(text))
    return [f"sensor config differs from the eval's: {error}" for error in errors] or ["sensor config: as the eval's"]


#: Printed in order for each UAV, each isolated from the others' failures.
UAV_SECTIONS: tuple[tuple[str, Callable[[Path, int], list[str]]], ...] = (
    ("log", _log_lines),
    ("approach", _approach_lines),
    ("cadence", _cadence_lines),
    ("source time", _source_time_lines),
    ("sensor config", _sensor_lines),
)


def uav_lines(run_dir: Path) -> list[str]:
    lines: list[str] = []
    for uav in uav_ids(run_dir):
        lines.append(f"UAV {uav}")
        for title, section in UAV_SECTIONS:
            lines.extend(f"  {line}" for line in guarded(title, section, run_dir, uav))
    return lines


__all__ = [
    "NO_END",
    "Outcome",
    "UAV_SECTIONS",
    "approach_outcomes",
    "debug_csv",
    "guarded",
    "navigation_log",
    "uav_ids",
    "uav_lines",
]
