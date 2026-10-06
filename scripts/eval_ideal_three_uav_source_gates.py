"""Source-cadence acceptance gates for the ideal three-UAV workflow."""

from __future__ import annotations

import math
from typing import Any

from scripts.eval_gcs_demo_constants import SIM_SPEEDUP
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
)


POSE_STREAM_SCHEDULER_FRACTION = 0.8
MAX_ATTITUDE_P99_PERIODS = 2.0
MIN_COMMAND_P50_PERIODS = 0.75
MAX_COMMAND_P50_PERIODS = 1.5
MAX_COMMAND_P99_PERIODS = 8.0
MAX_OBSERVATION_AGE_P95_PERIODS = 10.0
MAX_COMMAND_AGE_P95_PERIODS = 12.0
MAX_COMMAND_AGE_PERIODS = 20.0
MIN_STAGE_ROWS = 10
MAX_SOURCE_STAGE_GAP_MS = FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S * 1000.0
REQUIRED_SOURCE_STREAMS = (
    "attitude_arrival",
    "detector_pose",
    "observation",
    "worker",
)


def source_time_errors(
    summary: dict[str, Any],
    scheduler_rate_hz: float,
) -> list[str]:
    streams = summary.get("streams") or {}
    raw_period_ms = 1000.0 / (
        scheduler_rate_hz * POSE_STREAM_SCHEDULER_FRACTION
    )
    command_wall_period_ms = 1000.0 / (
        scheduler_rate_hz * SIM_SPEEDUP
    )
    errors: list[str] = []
    for stream in REQUIRED_SOURCE_STREAMS:
        rows = int((streams.get(stream) or {}).get("rows", 0))
        if rows < MIN_STAGE_ROWS:
            errors.append(
                f"source-time {stream} rows {rows} < {MIN_STAGE_ROWS}"
            )
    attitude = streams.get("attitude_arrival") or {}
    attitude_gap = attitude.get("source_gap_ms") or {}
    errors.extend(_nonnegative_distribution_errors(
        "raw ATTITUDE source gap",
        attitude_gap,
    ))
    _maximum_error(
        errors,
        "raw ATTITUDE source-gap p99",
        attitude_gap.get("p99"),
        MAX_ATTITUDE_P99_PERIODS * raw_period_ms,
    )
    detector = streams.get("detector_pose") or {}
    emitted = sum(
        int((detector.get("outcomes") or {}).get(name, 0))
        for name in ("emitted", "emitted_reanchor")
    )
    if emitted < MIN_STAGE_ROWS:
        errors.append(f"emitted ideal frames {emitted} < {MIN_STAGE_ROWS}")
    frame_gap = detector.get("frame_source_gap_ms") or {}
    errors.extend(_positive_distribution_errors(
        "ideal frame source gap",
        frame_gap,
    ))
    _maximum_error(
        errors,
        "ideal frame maximum source gap",
        frame_gap.get("max"),
        MAX_SOURCE_STAGE_GAP_MS,
    )
    observation = streams.get("observation") or {}
    observation_outcomes = observation.get("outcomes") or {}
    fresh_observations = int(observation_outcomes.get("fresh", 0))
    if fresh_observations < MIN_STAGE_ROWS:
        errors.append(
            f"fresh final-approach observations {fresh_observations} < "
            f"{MIN_STAGE_ROWS}"
        )
    if int(observation.get("future_count", 0)):
        errors.append("final-approach observations contain future source timestamps")
    if int(observation.get("nonmonotonic_count", 0)):
        errors.append("final-approach observations contain nonmonotonic source timestamps")
    observation_gap = observation.get("source_gap_ms") or {}
    errors.extend(_positive_distribution_errors(
        "final-approach observation source gap",
        observation_gap,
    ))
    _maximum_error(
        errors,
        "final-approach observation maximum source gap",
        observation_gap.get("max"),
        MAX_SOURCE_STAGE_GAP_MS,
    )
    _maximum_error(
        errors,
        "final-approach observation age p95",
        (observation.get("age_ms") or {}).get("p95"),
        MAX_OBSERVATION_AGE_P95_PERIODS * raw_period_ms,
    )
    worker = streams.get("worker") or {}
    outcomes = worker.get("outcomes") or {}
    if int(outcomes.get("fresh", 0)) < MIN_STAGE_ROWS:
        errors.append(
            f"fresh final-approach commands {int(outcomes.get('fresh', 0))} < "
            f"{MIN_STAGE_ROWS}"
        )
    source_gap = worker.get("source_gap_ms") or {}
    errors.extend(_positive_distribution_errors(
        "fresh final-approach command source gap",
        source_gap,
    ))
    _maximum_error(
        errors,
        "fresh final-approach command maximum source gap",
        source_gap.get("max"),
        MAX_SOURCE_STAGE_GAP_MS,
    )
    wall_gap = worker.get("wall_gap_ms") or {}
    errors.extend(_positive_distribution_errors(
        "final-approach command wall gap",
        wall_gap,
    ))
    # Fixed-grid scheduling has no positive lower bound on one isolated
    # inter-start gap: a late slot may finish immediately before the next
    # nominal slot. Sustained overspeed/underspeed is gated by p50, stalls by
    # p99, and missed-slot replay is covered at the worker mechanism boundary.
    _minimum_error(
        errors,
        "final-approach command wall-gap p50",
        wall_gap.get("p50"),
        MIN_COMMAND_P50_PERIODS * command_wall_period_ms,
    )
    _maximum_error(
        errors,
        "final-approach command wall-gap p50",
        wall_gap.get("p50"),
        MAX_COMMAND_P50_PERIODS * command_wall_period_ms,
    )
    _maximum_error(
        errors,
        "final-approach command wall-gap p99",
        wall_gap.get("p99"),
        MAX_COMMAND_P99_PERIODS * command_wall_period_ms,
    )
    age = worker.get("age_ms") or {}
    _maximum_error(
        errors,
        "final-approach command age p95",
        age.get("p95"),
        MAX_COMMAND_AGE_P95_PERIODS * raw_period_ms,
    )
    _maximum_error(
        errors,
        "final-approach command maximum age",
        age.get("max"),
        MAX_COMMAND_AGE_PERIODS * raw_period_ms,
    )
    return errors


def _maximum_error(
    errors: list[str],
    label: str,
    value: object,
    maximum: float,
) -> None:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        errors.append(f"{label} is missing or non-finite")
    elif float(value) > maximum and not math.isclose(
        float(value), maximum, abs_tol=1e-9
    ):
        errors.append(f"{label} {float(value):.3f}ms > {maximum:.3f}ms")


def _minimum_error(
    errors: list[str],
    label: str,
    value: object,
    minimum: float,
) -> None:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        errors.append(f"{label} is missing or non-finite")
    elif float(value) < minimum and not math.isclose(
        float(value), minimum, abs_tol=1e-9
    ):
        errors.append(f"{label} {float(value):.3f}ms < {minimum:.3f}ms")


def _nonnegative_distribution_errors(
    label: str,
    distribution: dict[str, Any],
) -> list[str]:
    minimum = distribution.get("min")
    if not isinstance(minimum, (int, float)) or not math.isfinite(float(minimum)):
        return [f"{label} distribution is missing or non-finite"]
    if float(minimum) < 0.0:
        return [f"{label} contains a negative delta {float(minimum):.3f}ms"]
    return []


def _positive_distribution_errors(
    label: str,
    distribution: dict[str, Any],
) -> list[str]:
    minimum = distribution.get("min")
    if not isinstance(minimum, (int, float)) or not math.isfinite(float(minimum)):
        return [f"{label} distribution is missing or non-finite"]
    if float(minimum) <= 0.0:
        return [f"{label} contains a nonpositive delta {float(minimum):.3f}ms"]
    return []


__all__ = ["REQUIRED_SOURCE_STREAMS", "source_time_errors"]
