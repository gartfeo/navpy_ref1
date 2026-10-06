"""Strict parsing and focused summaries for source-time CSV streams."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from scripts.eval_certificate_statistics import distribution


_REQUIRED_COLUMNS: dict[str, frozenset[str]] = {
    "attitude_arrival": frozenset({"wall_s", "boot_ms"}),
    "position_arrival": frozenset({"wall_s", "boot_ms"}),
    "telemetry_reject": frozenset({"mtype", "reason"}),
    "detector_pose": frozenset({"outcome", "frame_ts"}),
    "observation": frozenset({"outcome", "obs_ts", "src_now_s"}),
    "worker": frozenset({"wall_start_s", "obs_ts", "src_now_s"}),
}
SOURCE_STREAMS = frozenset(_REQUIRED_COLUMNS)
EMITTED_OUTCOMES = frozenset({"emitted", "emitted_reanchor"})


def read_rows(path: Path, stream: str) -> list[dict[str, str]]:
    """Read one present CSV or surface its I/O/schema failure."""
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames
        if fields is None:
            raise ValueError(f"{path}: source-time CSV has no header")
        required = _REQUIRED_COLUMNS.get(stream, frozenset())
        missing = sorted(required - set(fields))
        if missing:
            raise ValueError(f"{path}: missing source-time columns {missing}")
        rows = list(reader)
    for line_number, row in enumerate(rows, start=2):
        if None in row:
            raise ValueError(f"{path}:{line_number}: unexpected extra CSV cells")
        if any(value is None for value in row.values()):
            raise ValueError(f"{path}:{line_number}: incomplete CSV row")
    return rows


def summarize_arrivals(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    source_gaps: list[float] = []
    wall_gaps: list[float] = []
    total = 0
    for rows in entries:
        total += len(rows)
        source_gaps.extend(_deltas(_numbers(rows, "boot_ms", required=True)))
        wall_gaps.extend(
            delta * 1000.0
            for delta in _deltas(_numbers(rows, "wall_s", required=True))
        )
    return {
        "rows": total,
        "source_gap_ms": _dist_dict(source_gaps),
        "wall_gap_ms": _dist_dict(wall_gaps),
    }


def summarize_rejections(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    counts: dict[str, int] = {}
    total = 0
    for rows in entries:
        for row in rows:
            total += 1
            key = f"{row.get('mtype', '')}/{row.get('reason', '')}"
            counts[key] = counts.get(key, 0) + 1
    return {"rows": total, "counts": counts}


def summarize_detector(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    outcomes: dict[str, int] = {}
    frame_gaps: list[float] = []
    total = 0
    for rows in entries:
        emitted_ts: list[float] = []
        for row in rows:
            total += 1
            outcome = row.get("outcome", "")
            outcomes[outcome] = outcomes.get(outcome, 0) + 1
            emitted = outcome in EMITTED_OUTCOMES
            frame_ts = _number(
                row,
                "frame_ts",
                required=emitted,
            )
            if not emitted and frame_ts is not None:
                raise ValueError(
                    f"non-emitted detector outcome {outcome!r} has frame_ts"
                )
            if emitted:
                emitted_ts.append(frame_ts)
        frame_gaps.extend(delta * 1000.0 for delta in _deltas(emitted_ts))
    return {
        "rows": total,
        "outcomes": outcomes,
        "frame_source_gap_ms": _dist_dict(frame_gaps),
    }


def summarize_observations(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    outcomes: dict[str, int] = {}
    gaps: list[float] = []
    ages: list[float] = []
    future_count = 0
    nonmonotonic_count = 0
    total = 0
    for rows in entries:
        fresh_ts: list[float] = []
        for row in rows:
            total += 1
            outcome = row.get("outcome", "")
            outcomes[outcome] = outcomes.get(outcome, 0) + 1
            if outcome != "fresh":
                continue
            obs_ts = _number(row, "obs_ts", required=True)
            src_now = _number(row, "src_now_s", required=True)
            age_ms = (src_now - obs_ts) * 1000.0
            ages.append(age_ms)
            if age_ms < 0.0:
                future_count += 1
            if fresh_ts and obs_ts <= fresh_ts[-1]:
                nonmonotonic_count += 1
            fresh_ts.append(obs_ts)
        gaps.extend(delta * 1000.0 for delta in _deltas(fresh_ts))
    return {
        "rows": total,
        "outcomes": outcomes,
        "source_gap_ms": _dist_dict(gaps),
        "age_ms": _dist_dict(ages),
        "future_count": future_count,
        "nonmonotonic_count": nonmonotonic_count,
    }


def summarize_worker(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    source_gaps: list[float] = []
    source_clock_gaps: list[float] = []
    wall_gaps: list[float] = []
    ages: list[float] = []
    total = 0
    outcomes: dict[str, int] = {}
    for rows in entries:
        measured_ts: list[float] = []
        source_clock_ts: list[float] = []
        walls: list[float] = []
        for row in rows:
            total += 1
            if "source" in row and row["source"] != "measured":
                raise ValueError(
                    "source-time worker source must be 'measured' when present"
                )
            outcome = row.get("outcome", "") or "fresh"
            if outcome not in {"fresh", "held"}:
                raise ValueError(
                    "source-time worker outcome must be 'fresh' or 'held'"
                )
            outcomes[outcome] = outcomes.get(outcome, 0) + 1
            walls.append(_number(row, "wall_start_s", required=True))
            obs_ts = _number(row, "obs_ts", required=True)
            src_now = _number(row, "src_now_s", required=True)
            source_clock_ts.append(src_now)
            ages.append((src_now - obs_ts) * 1000.0)
            if outcome == "fresh":
                measured_ts.append(obs_ts)
        source_gaps.extend(delta * 1000.0 for delta in _deltas(measured_ts))
        source_clock_gaps.extend(
            delta * 1000.0 for delta in _deltas(source_clock_ts)
        )
        wall_gaps.extend(delta * 1000.0 for delta in _deltas(walls))
    return {
        "rows": total,
        "outcomes": outcomes,
        "source_gap_ms": _dist_dict(source_gaps),
        "source_clock_gap_ms": _dist_dict(source_clock_gaps),
        "wall_gap_ms": _dist_dict(wall_gaps),
        "age_ms": _dist_dict(ages),
    }


def _numbers(
    rows: Iterable[Mapping[str, str]],
    field: str,
    *,
    required: bool,
) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = _number(row, field, required=required)
        if value is not None:
            values.append(value)
    return values


def _number(
    row: Mapping[str, str],
    field: str,
    *,
    required: bool = False,
) -> float | None:
    raw = row.get(field, "")
    text = raw.strip() if isinstance(raw, str) else ""
    if not text:
        if required:
            raise ValueError(f"source-time field {field!r} is blank")
        return None
    try:
        number = float(text)
    except ValueError as exc:
        raise ValueError(f"source-time field {field!r} is not numeric: {raw!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"source-time field {field!r} is not finite: {raw!r}")
    return number


def _deltas(values: Iterable[float]) -> list[float]:
    output: list[float] = []
    previous: float | None = None
    for value in values:
        if previous is not None:
            output.append(value - previous)
        previous = value
    return output


def _dist_dict(values: Sequence[float]) -> dict[str, Any] | None:
    stats = distribution(values)
    return stats.as_dict() if stats is not None else None
