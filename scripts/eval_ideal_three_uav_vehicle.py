"""Per-vehicle acceptance gates for the ideal three-UAV workflow."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

from scripts.eval_gcs_demo_command_bounds import (
    command_bounds_errors,
    load_command_bounds,
)
from scripts.eval_gcs_demo_constants import SIM_SPEEDUP
from scripts.eval_gcs_demo_evidence import (
    mission_navigation_segment,
    parse_terminal_command_episodes,
)
from scripts.eval_gcs_demo_limits import CadenceLimits
from scripts.eval_gcs_demo_metrics import score_terminal_commands, terminal_timing
from scripts.eval_gcs_demo_models import EpisodeMetrics, VehicleReport
from scripts.eval_gcs_demo_snap import (
    navigation_snap_distance,
    parse_compact_snap_distances,
    parse_debug_snap_distances,
)
from scripts.eval_gcs_demo_terminal_gates import cadence_errors
from scripts.eval_gcs_demo_vehicle import navigation_log


MIN_ISSUED_COMMANDS = 10
IDEAL_PROFILE = "ideal_360"
_FATAL_MARKERS = (
    "NAVIGATION_EVIDENCE_FAILURE",
    "NAV_FAIL:",
    "GUIDED not accepted within",
    "DetectorSim ideal source overload",
    "Loop time exceeded:",
    "nav loop error:",
    "NAVIGATION WORKER FAILED",
    "Traceback (most recent call last):",
)


def _truth_errors(
    metrics: EpisodeMetrics,
    navigation_text: str,
    compact_path: Path,
    debug_path: Path,
    max_snap_distance_m: float,
) -> list[str]:
    errors: list[str] = []
    navigation_count = navigation_text.count("SNAP(VISION-NAV")
    compact_snaps = parse_compact_snap_distances(compact_path)
    debug_snaps = parse_debug_snap_distances(debug_path)
    if navigation_count != 1:
        errors.append(
            f"expected exactly one navigation SNAP episode, found {navigation_count}"
        )
    if len(compact_snaps) != 1:
        errors.append(
            f"expected exactly one compact SNAP episode, found {len(compact_snaps)}"
        )
    if len(debug_snaps) != 1:
        errors.append(
            f"expected exactly one debug SNAP episode, found {len(debug_snaps)}"
        )
    navigation_snap = navigation_snap_distance(navigation_text)
    if navigation_snap is None:
        errors.append("navigation log did not contain a truth SNAP distance")
    candidates = [
        value
        for value in (
            navigation_snap,
            compact_snaps[-1] if compact_snaps else None,
            debug_snaps[-1] if debug_snaps else None,
        )
        if value is not None
    ]
    if any(not math.isfinite(value) for value in candidates):
        errors.append(f"truth SNAP distance is non-finite ({candidates})")
        return errors
    if candidates:
        metrics.snap_distance_m = candidates[-1]
    if len(candidates) == 3 and max(candidates) - min(candidates) > 0.25:
        errors.append(f"navigation/compact/debug SNAP disagreement: {candidates}")
    if metrics.snap_distance_m is not None and (
        metrics.snap_distance_m >= max_snap_distance_m
    ):
        errors.append(
            f"truth SNAP distance {metrics.snap_distance_m:.3f}m is not < "
            f"{max_snap_distance_m:.3f}m"
        )
    return errors


def analyze_ideal_vehicle(
    log_dir: Path,
    sys_id: int,
    role: str,
    *,
    approval_count: int,
    max_snap_distance_m: float,
    source_metrics: dict[str, object] | None = None,
    source_errors: Sequence[str] = (),
) -> VehicleReport:
    metrics = EpisodeMetrics()
    _apply_source_metrics(metrics, source_metrics or {})
    errors = list(source_errors)
    paths = {
        "navigation": navigation_log(log_dir, sys_id),
        "compact": log_dir / f"uav_{sys_id}_navigation_compact.csv",
        "debug": log_dir / f"uav_{sys_id}_navigation_debug.csv",
    }
    missing = [path.name for path in paths.values() if not path.is_file()]
    if missing:
        return VehicleReport(
            sys_id,
            role,
            False,
            metrics,
            [f"missing {name}" for name in missing],
        )
    navigation_text = paths["navigation"].read_text(
        encoding="utf-8",
        errors="replace",
    )
    mission_text = mission_navigation_segment(navigation_text)
    if f"vision_profile='{IDEAL_PROFILE}'" not in navigation_text:
        errors.append("missing ideal_360 argument marker")
    if "static ideal 360 enabled" not in navigation_text.lower():
        errors.append("missing static ideal_360 activation marker")
    for marker in _FATAL_MARKERS:
        if marker in mission_text:
            errors.append(f"fatal runtime marker present: {marker}")
    required_markers = (
        "confirmed by ground station.",
        "INIT: NAV MODE",
        "RESET: PASSED TARGET",
        "SNAP(VISION-NAV",
    )
    for marker in required_markers:
        if marker not in mission_text:
            errors.append(f"missing workflow marker: {marker}")
    if approval_count != 1:
        errors.append(f"expected exactly one approval, found {approval_count}")
    try:
        episodes = parse_terminal_command_episodes(paths["debug"])
        if len(episodes) != 1:
            errors.append(
                f"expected exactly one terminal command episode, found {len(episodes)}"
            )
        if episodes:
            commands = episodes[-1]
            bounds = load_command_bounds(
                log_dir / "demo_command_bounds.json"
            ).for_vehicle(sys_id)
            score = score_terminal_commands(
                commands,
                roll_limit_deg=bounds.roll_limit_deg,
                saturation_margin_deg=0.5,
                significant_roll_deg=10.0,
            )
            timing = terminal_timing(commands)
            metrics.sample_count = score.sample_count
            metrics.median_wall_gap_s = timing.median_wall_gap_s
            metrics.max_wall_gap_s = timing.max_wall_gap_s
            metrics.max_source_gap_s = timing.max_source_gap_s
            metrics.observed_speedup = timing.observed_speedup
            errors.extend(command_bounds_errors(commands, bounds))
            errors.extend(
                cadence_errors(
                    timing,
                    score,
                    CadenceLimits(),
                    expected_speedup=SIM_SPEEDUP,
                )
            )
            issued_count = sum(command.issued for command in commands)
            if issued_count < MIN_ISSUED_COMMANDS:
                errors.append(
                    f"issued terminal command count {issued_count} < "
                    f"{MIN_ISSUED_COMMANDS}"
                )
        else:
            errors.append("issued terminal command count 0 < 10")
        errors.extend(
            _truth_errors(
                metrics,
                mission_text,
                paths["compact"],
                paths["debug"],
                max_snap_distance_m,
            )
        )
    except (OSError, UnicodeError, ValueError) as error:
        errors.append(f"invalid terminal evidence: {error}")
    return VehicleReport(sys_id, role, not errors, metrics, errors)


def _apply_source_metrics(
    metrics: EpisodeMetrics,
    source: dict[str, object],
) -> None:
    mapping = {
        "process_pid": "source_process_pid",
        "scheduler_rate_hz": "scheduler_rate_hz",
        "attitude_count": "attitude_source_count",
        "attitude_source_gap_p99_ms": "attitude_source_gap_p99_ms",
        "frame_source_gap_p99_ms": "frame_source_gap_p99_ms",
        "obs_count": "observation_source_count",
        "obs_source_gap_p99_ms": "observation_source_gap_p99_ms",
        "obs_age_p95_ms": "observation_age_p95_ms",
        "cmd_count": "command_source_count",
        "cmd_wall_gap_p50_ms": "command_wall_gap_p50_ms",
        "cmd_wall_gap_p99_ms": "command_wall_gap_p99_ms",
        "cmd_age_p95_ms": "command_age_p95_ms",
        "cmd_age_max_ms": "command_age_max_ms",
    }
    for source_name, metric_name in mapping.items():
        value = source.get(source_name)
        if value != "" and value is not None:
            setattr(metrics, metric_name, value)


__all__ = ["analyze_ideal_vehicle"]
