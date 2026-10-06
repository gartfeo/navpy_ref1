"""Source-time evidence orchestration and report flattening."""

from __future__ import annotations

import re
from collections.abc import Iterable
from pathlib import Path
from typing import Any, Mapping

from scripts.eval_certificate_source_streams import (
    EMITTED_OUTCOMES,
    SOURCE_STREAMS,
    read_rows,
    summarize_arrivals,
    summarize_detector,
    summarize_observations,
    summarize_rejections,
    summarize_worker,
)


SOURCE_TIME_SUBDIR = "source-time"
SOURCE_TIME_ENV = "NAVPY_POSE_CADENCE_DEBUG"
SOURCE_TIME_FLUSH_GRACE_S = 0.6

_SOURCE_TIME_FILE_RE = re.compile(
    r"^pose_cadence_(?P<stream>[a-z_]+)_uav_(?P<sys>\d+)_pid_(?P<pid>\d+)\.csv$"
)


def source_time_summary(
    source_dir: Path,
    *,
    sys_id: int | None = None,
    include_files: Iterable[str] | None = None,
) -> dict[str, Any]:
    """Summarize raw source-time evidence without simulator-time scaling."""
    if sys_id is not None and (type(sys_id) is not int or sys_id <= 0):
        raise ValueError("source-time sys_id must be a positive integer")
    selected_files = None
    if include_files is not None:
        selected_files = frozenset(include_files)
        if not all(type(name) is str and name for name in selected_files):
            raise ValueError("source-time include_files must contain file names")
    files_by_stream: dict[str, list[list[dict[str, str]]]] = {}
    file_count = 0
    matched_names: list[str] = []
    if source_dir.exists() and not source_dir.is_dir():
        raise NotADirectoryError(f"source-time path is not a directory: {source_dir}")
    if source_dir.is_dir():
        for path in sorted(source_dir.iterdir()):
            if selected_files is not None and path.name not in selected_files:
                continue
            match = _SOURCE_TIME_FILE_RE.fullmatch(path.name)
            if match is None:
                continue
            if sys_id is not None and int(match.group("sys")) != sys_id:
                continue
            stream = match.group("stream")
            if stream not in SOURCE_STREAMS:
                raise ValueError(
                    f"{path}: unsupported source-time stream {stream!r}"
                )
            file_count += 1
            matched_names.append(path.name)
            files_by_stream.setdefault(stream, []).append(read_rows(path, stream))

    streams: dict[str, Any] = {}
    for stream in ("attitude_arrival", "position_arrival"):
        entries = files_by_stream.get(stream)
        if entries is not None:
            streams[stream] = summarize_arrivals(entries)
    handlers = {
        "telemetry_reject": summarize_rejections,
        "detector_pose": summarize_detector,
        "observation": summarize_observations,
        "worker": summarize_worker,
    }
    for stream, handler in handlers.items():
        entries = files_by_stream.get(stream)
        if entries is not None:
            streams[stream] = handler(entries)
    return {
        "source_dir": str(source_dir),
        "files": file_count,
        "file_names": sorted(matched_names),
        "streams": streams,
    }


def source_time_row_metrics(summary: Mapping[str, Any] | None) -> dict[str, Any]:
    """Flatten informational source-time metrics into one attempt row."""
    streams = (summary or {}).get("streams") or {}

    def dist_value(stream: str, dist: str, field: str) -> Any:
        stats = (streams.get(stream) or {}).get(dist) or {}
        value = stats.get(field)
        return value if value is not None else ""

    def stream_value(stream: str, field: str) -> Any:
        entry = streams.get(stream)
        if entry is None:
            return ""
        value = entry.get(field)
        return value if value is not None else ""

    def named_count(stream: str, counter: str, name: str) -> Any:
        entry = streams.get(stream)
        if entry is None:
            return ""
        return (entry.get(counter) or {}).get(name, 0)

    detector = streams.get("detector_pose")
    detector_outcomes = (detector or {}).get("outcomes") or {}
    frame_emitted = (
        sum(detector_outcomes.get(outcome, 0) for outcome in EMITTED_OUTCOMES)
        if detector is not None
        else ""
    )
    frame_dropped = (
        detector_outcomes.get("dup_attitude", 0)
        + detector_outcomes.get("reordered_attitude", 0)
        + sum(
            count
            for outcome, count in detector_outcomes.items()
            if outcome.endswith("_dropped")
        )
        + detector_outcomes.get("behind_emitted", 0)
        if detector is not None
        else ""
    )
    return {
        "attitude_count": stream_value("attitude_arrival", "rows"),
        "attitude_source_gap_p99_ms": dist_value(
            "attitude_arrival", "source_gap_ms", "p99"
        ),
        "frame_source_gap_p99_ms": dist_value(
            "detector_pose", "frame_source_gap_ms", "p99"
        ),
        "frame_emitted_count": frame_emitted,
        "frame_dropped_count": frame_dropped,
        "frame_reanchor_count": named_count(
            "detector_pose", "outcomes", "emitted_reanchor"
        ),
        "frame_reordered_count": named_count(
            "detector_pose", "outcomes", "reordered_attitude"
        ),
        "obs_source_gap_p99_ms": dist_value(
            "observation", "source_gap_ms", "p99"
        ),
        "obs_count": stream_value("observation", "rows"),
        "obs_age_p95_ms": dist_value("observation", "age_ms", "p95"),
        "obs_duplicate_count": named_count(
            "observation", "outcomes", "duplicate"
        ),
        "obs_future_count": stream_value("observation", "future_count"),
        "obs_nonmonotonic_count": stream_value(
            "observation", "nonmonotonic_count"
        ),
        "cmd_source_gap_p99_ms": dist_value("worker", "source_gap_ms", "p99"),
        "cmd_source_clock_gap_p50_ms": dist_value(
            "worker", "source_clock_gap_ms", "p50"
        ),
        "cmd_count": stream_value("worker", "rows"),
        "cmd_wall_gap_p50_ms": dist_value("worker", "wall_gap_ms", "p50"),
        "cmd_wall_gap_p99_ms": dist_value("worker", "wall_gap_ms", "p99"),
        "cmd_fresh_count": named_count("worker", "outcomes", "fresh"),
        "cmd_held_count": named_count("worker", "outcomes", "held"),
        "cmd_age_p95_ms": dist_value("worker", "age_ms", "p95"),
        "cmd_age_max_ms": dist_value("worker", "age_ms", "max"),
    }


__all__ = [
    "SOURCE_TIME_ENV",
    "SOURCE_TIME_FLUSH_GRACE_S",
    "SOURCE_TIME_SUBDIR",
    "source_time_row_metrics",
    "source_time_summary",
]
