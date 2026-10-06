"""Command-stability evidence from one case's compact navigation log."""

from __future__ import annotations

import csv
from pathlib import Path


MIDCOURSE_MIN_DISTANCE_M = 100.0
MAX_MIDCOURSE_PITCH_STEP_DEG = 2.0
ALIGNMENT_MAX_DISTANCE_M = 300.0
MAX_ALIGNMENT_MEAN_ABS_BEARING_DEG = 1.0
MAX_ALIGNMENT_MEAN_ABS_ROLL_DEG = 2.0


def _command_stability(case_dir: Path) -> dict[str, float | int]:
    paths = list((case_dir / "navpy-logs").glob("*_navigation_compact.csv"))
    if len(paths) != 1:
        raise RuntimeError(
            f"expected one compact navigation log, found {len(paths)}"
        )
    rows: list[tuple[float, float, float, float, float]] = []
    with paths[0].open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append((
                    float(row["dist"]),
                    float(row["cmd_r"]),
                    float(row["cmd_p"]),
                    float(row["act_r"]),
                    float(row["yaw_err"]),
                ))
            except (KeyError, TypeError, ValueError):
                continue
    if not rows:
        raise RuntimeError("compact navigation log has no issued command rows")
    far_limit_m = rows[0][0] * 0.5
    midcourse = [
        row for row in rows
        if MIDCOURSE_MIN_DISTANCE_M < row[0] < far_limit_m
    ]
    if len(midcourse) < 10:
        raise RuntimeError(
            f"insufficient midcourse command evidence: {len(midcourse)} rows"
        )
    pitch = [row[2] for row in midcourse]
    roll = [row[1] for row in midcourse]
    alignment = [
        row for row in rows
        if MIDCOURSE_MIN_DISTANCE_M < row[0] < ALIGNMENT_MAX_DISTANCE_M
    ]
    if len(alignment) < 10:
        raise RuntimeError(
            f"insufficient final-approach alignment evidence: {len(alignment)} rows"
        )
    return {
        "sample_count": len(midcourse),
        "pitch_span_deg": max(pitch) - min(pitch),
        "pitch_max_step_deg": max(
            abs(current - previous)
            for previous, current in zip(pitch, pitch[1:])
        ),
        "roll_span_deg": max(roll) - min(roll),
        "roll_max_step_deg": max(
            abs(current - previous)
            for previous, current in zip(roll, roll[1:])
        ),
        "alignment_sample_count": len(alignment),
        "alignment_cmd_roll_mean_abs_deg": sum(
            abs(row[1]) for row in alignment
        ) / len(alignment),
        "alignment_actual_roll_mean_abs_deg": sum(
            abs(row[3]) for row in alignment
        ) / len(alignment),
        "alignment_bearing_mean_abs_deg": sum(
            abs(row[4]) for row in alignment
        ) / len(alignment),
    }


__all__ = [
    "ALIGNMENT_MAX_DISTANCE_M",
    "MAX_ALIGNMENT_MEAN_ABS_BEARING_DEG",
    "MAX_ALIGNMENT_MEAN_ABS_ROLL_DEG",
    "MAX_MIDCOURSE_PITCH_STEP_DEG",
    "MIDCOURSE_MIN_DISTANCE_M",
    "_command_stability",
]
