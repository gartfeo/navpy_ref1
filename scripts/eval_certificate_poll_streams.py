"""Summaries of the detector poll-timing and frame-lineage debug streams.

Triage only, never a gate. ``thread_time`` is coarse on Windows (15.625 ms
steps), so CPU is compared with wall time per wall second, not per row: a
second whose CPU fraction is far below one was mostly off-CPU (preemption,
GIL/lock, I/O); close to one, the thread was running.
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping, Sequence

from scripts.eval_certificate_statistics import distribution

POLL_COLUMNS = frozenset({
    "start_wall_s", "outcome", "work_ms", "work_cpu_ms", "requested_wait_ms",
    "actual_wait_ms", "read_ms", "project_ms", "publish_ms", "log_ms",
    "residual_before_ms",
})
EMITTED_OUTCOMES = frozenset({"emitted", "emitted_reanchor"})
LINEAGE_COLUMNS = frozenset({"stage", "epoch", "frame_ts", "disposition"})
_POLL_TIMES = (
    "work_ms", "requested_wait_ms", "actual_wait_ms", "read_ms", "project_ms",
    "publish_ms", "log_ms", "residual_before_ms",
)


def summarize_poll(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    outcomes: dict[str, int] = {}
    times: dict[str, list[float]] = {name: [] for name in _POLL_TIMES}
    rows_seen = 0
    per_second: list[list[float]] = []
    for rows in entries:
        per_second.extend(cpu_fraction_by_second(rows, "start_wall_s", "work_ms"))
        for row in rows:
            rows_seen += 1
            outcomes[row["outcome"]] = outcomes.get(row["outcome"], 0) + 1
            for name in _POLL_TIMES:
                value = _optional(row, name)
                if value is not None:
                    times[name].append(value)
    return {
        "rows": rows_seen,
        "outcomes": outcomes,
        **{name: _dist(values) for name, values in times.items()},
        "cpu_fraction_per_second": _dist([cpu / wall for cpu, wall in per_second if wall > 0.0]),
    }


def summarize_lineage(entries: Sequence[list[dict[str, str]]]) -> dict[str, Any]:
    counts: dict[str, int] = {}
    total = 0
    for rows in entries:
        for row in rows:
            total += 1
            key = f"{row['stage']}/{row['disposition']}"
            counts[key] = counts.get(key, 0) + 1
    return {"rows": total, "dispositions": counts}


def reconcile_lineage(
    per_vehicle: Iterable[Mapping[str, Sequence[Mapping[str, str]]]],
) -> dict[str, int]:
    """Join emit, observation and disposition rows of each UAV by frame_ts.

    The join key is the frame's source time: the emit-stage epoch counts
    drained discontinuities while observation and worker rows carry the
    vision_nav generation, and the two agree only while every discontinuity
    reaches ingress, so epochs are not joined on. A source time is unique only
    within one epoch, so a UAV whose rows span more than one epoch, or whose
    emit stream is missing, is counted as skipped rather than joined. An
    emitted frame with no observation includes frames with no dock in view.
    """
    totals = dict.fromkeys((
        "emitted", "emitted_without_observation", "fresh",
        "fresh_without_disposition", "disposition_without_emit",
        "vehicles_skipped_multi_epoch", "vehicles_without_emit_stream",
    ), 0)
    for streams in per_vehicle:
        epochs = {
            row.get("epoch", "") for stream in ("detector_pose", "observation", "frame_lineage")
            for row in streams.get(stream, ()) if row.get("epoch", "").strip()
        }
        if len(epochs) > 1:
            totals["vehicles_skipped_multi_epoch"] += 1
            continue
        observed = [row for row in streams.get("observation", ()) if row["obs_ts"].strip()]
        fresh = {_key(row["obs_ts"]) for row in observed if row["outcome"] == "fresh"}
        disposed = {_key(row["frame_ts"]) for row in streams.get("frame_lineage", ())}
        totals["fresh"] += len(fresh)
        # Inferred "unfinished": admitted, never consumed, replaced or dropped.
        totals["fresh_without_disposition"] += len(fresh - disposed)
        if "detector_pose" not in streams:
            totals["vehicles_without_emit_stream"] += 1
            continue
        emitted = [
            _key(row["frame_ts"]) for row in streams["detector_pose"]
            if row["outcome"] in EMITTED_OUTCOMES
        ]
        totals["emitted"] += len(emitted)  # every emission, repeats included
        observed_keys = {_key(row["obs_ts"]) for row in observed}
        totals["emitted_without_observation"] += len(set(emitted) - observed_keys)
        totals["disposition_without_emit"] += len(disposed - set(emitted))
    return totals


def _key(text: str) -> float:
    return round(float(text), 6)


def cpu_fraction_by_second(
    rows: Sequence[Mapping[str, str]],
    wall_field: str,
    work_field: str,
) -> list[list[float]]:
    """[cpu_ms, wall_ms] summed per whole wall second of ``wall_field``."""
    buckets: dict[int, list[float]] = {}
    for row in rows:
        wall_s, work, cpu = (
            _optional(row, wall_field), _optional(row, work_field), _optional(row, "work_cpu_ms")
        )
        if wall_s is None or work is None or cpu is None:
            continue
        bucket = buckets.setdefault(math.floor(wall_s), [0.0, 0.0])
        bucket[0] += cpu
        bucket[1] += work
    return [buckets[key] for key in sorted(buckets)]


def _optional(row: Mapping[str, str], field: str) -> float | None:
    text = (row.get(field) or "").strip()
    if not text:
        return None
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(f"source-time field {field!r} is not finite: {text!r}")
    return value


def _dist(values: Sequence[float]) -> dict[str, Any] | None:
    stats = distribution(values)
    return stats.as_dict() if stats is not None else None


__all__ = [
    "EMITTED_OUTCOMES",
    "LINEAGE_COLUMNS",
    "POLL_COLUMNS",
    "cpu_fraction_by_second",
    "reconcile_lineage",
    "summarize_lineage",
    "summarize_poll",
]
