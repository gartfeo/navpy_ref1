"""One-vehicle final-approach evidence analysis."""

from __future__ import annotations

import math
from pathlib import Path

from scripts.eval_gcs_demo_command_bounds import (
    VehicleCommandBounds,
    command_bounds_errors,
)
from scripts.eval_gcs_demo_constants import FINAL_APPROACH_ROLL_LIMIT_DEG
from scripts.eval_gcs_demo_constants import SIM_SPEEDUP
from scripts.eval_gcs_demo_evidence import (
    mission_navigation_segment,
    parse_final_approach_command_episodes,
)
from scripts.eval_gcs_demo_gates import confirmation_image_errors, workflow_errors
from scripts.eval_gcs_demo_metrics import score_final_approach_commands, final_approach_timing
from scripts.eval_gcs_demo_models import (
    EpisodeMetrics,
    GateLimits,
    FinalApproachCommand,
    VehicleReport,
    finite_number,
    positive_int,
)
from scripts.eval_gcs_demo_snap import (
    navigation_snap_distance,
    parse_compact_snap_distances,
    parse_debug_snap_distances,
)
from scripts.eval_gcs_demo_final_approach_gates import (
    cadence_errors,
    roll_quality_errors,
)


def navigation_log(log_dir: Path, sys_id: int) -> Path:
    return log_dir / f"uav_{sys_id}_navigation.log"


def _truth_errors(
    metrics: EpisodeMetrics,
    *,
    navigation_snap: float | None,
    compact_snaps: list[float],
    debug_snaps: list[float],
    limit_m: float,
) -> list[str]:
    errors: list[str] = []
    if navigation_snap is not None:
        metrics.snap_distance_m = navigation_snap
    if navigation_snap is None:
        errors.append("navigation log did not contain a truth SNAP distance")
    elif not math.isfinite(navigation_snap):
        errors.append(f"truth SNAP distance is non-finite ({navigation_snap})")
    if len(compact_snaps) != 1:
        errors.append(f"expected exactly one compact SNAP episode, found {len(compact_snaps)}")
    if len(debug_snaps) != 1:
        errors.append(f"expected exactly one debug SNAP episode, found {len(debug_snaps)}")
    if not debug_snaps:
        return errors
    metrics.snap_distance_m = debug_snaps[-1]
    if not math.isfinite(metrics.snap_distance_m):
        message = f"truth SNAP distance is non-finite ({metrics.snap_distance_m})"
        if message not in errors:
            errors.append(message)
        return errors
    if compact_snaps:
        compact = compact_snaps[-1]
        if not math.isfinite(compact):
            errors.append(f"compact SNAP distance is non-finite ({compact})")
        elif abs(metrics.snap_distance_m - compact) > 0.25:
            errors.append(
                f"compact/debug SNAP disagreement: {compact:.2f}m vs "
                f"{metrics.snap_distance_m:.2f}m"
            )
    if navigation_snap is not None and math.isfinite(navigation_snap):
        if abs(metrics.snap_distance_m - navigation_snap) > 0.25:
            errors.append(
                f"navigation/debug SNAP disagreement: {navigation_snap:.2f}m vs "
                f"{metrics.snap_distance_m:.2f}m"
            )
    if metrics.snap_distance_m >= limit_m:
        errors.append(
            f"truth SNAP distance {metrics.snap_distance_m:.2f}m is not < "
            f"{limit_m:.2f}m"
        )
    return errors


def _command_errors(
    metrics: EpisodeMetrics,
    commands: list[FinalApproachCommand],
    limits: GateLimits,
    command_bounds: VehicleCommandBounds | None,
    *,
    expected_speedup: float,
) -> list[str]:
    errors: list[str] = []
    roll_limit = (
        FINAL_APPROACH_ROLL_LIMIT_DEG
        if command_bounds is None
        else command_bounds.roll_limit_deg
    )
    try:
        timing = final_approach_timing(commands)
        score = score_final_approach_commands(
            commands,
            roll_limit_deg=roll_limit,
            saturation_margin_deg=limits.roll_quality.saturation_margin_deg,
            significant_roll_deg=limits.roll_quality.significant_roll_deg,
        )
    except ValueError as error:
        return [f"invalid final-approach command episode: {error}"]
    metrics.sample_count = score.sample_count
    metrics.median_wall_gap_s = timing.median_wall_gap_s
    metrics.max_wall_gap_s = timing.max_wall_gap_s
    metrics.max_source_gap_s = timing.max_source_gap_s
    metrics.observed_speedup = timing.observed_speedup
    metrics.significant_reversals = score.significant_reversals
    metrics.saturation_fraction = score.saturation_fraction
    metrics.max_saturation_run = score.max_saturation_run
    metrics.max_roll_step_deg = score.max_roll_step_deg
    if command_bounds is not None:
        errors.extend(command_bounds_errors(commands, command_bounds))
    else:
        outside_roll = [
            item.cmd_roll
            for item in commands
            if item.issued
            and item.cmd_roll is not None
            and abs(item.cmd_roll) > roll_limit
        ]
        if outside_roll:
            errors.append(
                "final-approach roll command exceeds configured ROLL_LIMIT_DEG: "
                f"{outside_roll}"
            )
    errors.extend(
        cadence_errors(
            timing,
            score,
            limits.cadence,
            expected_speedup=expected_speedup,
        )
    )
    errors.extend(roll_quality_errors(score, limits.roll_quality))
    return errors


def analyze_vehicle(
    log_dir: Path,
    sys_id: int,
    role: str,
    *,
    approval_count: int,
    limits: GateLimits = GateLimits(),
    navigation_speedup: float = 0.0,
    launch_speedup: float = SIM_SPEEDUP,
    command_bounds: VehicleCommandBounds | None = None,
    assignment_acked: bool = False,
) -> VehicleReport:
    checked_sys_id = positive_int("sys_id", sys_id)
    if role not in {"owner", "peer"}:
        raise ValueError(f"invalid demo role {role!r}")
    if type(approval_count) is not int or approval_count < 0:
        raise ValueError("approval_count must be a non-negative integer")
    if command_bounds is not None:
        if not isinstance(command_bounds, VehicleCommandBounds):
            raise TypeError("command_bounds must be VehicleCommandBounds")
        if command_bounds.sys_id != checked_sys_id:
            raise ValueError("command_bounds sys_id does not match vehicle")
    requested_speedup = finite_number("navigation_speedup", navigation_speedup)
    launch_speed = finite_number("launch_speedup", launch_speedup)
    if launch_speed <= 0.0:
        raise ValueError("launch_speedup must be positive")
    expected_speedup = requested_speedup if requested_speedup > 0.0 else launch_speed
    metrics = EpisodeMetrics()
    errors: list[str] = []
    paths = {
        "navigation": navigation_log(log_dir, checked_sys_id),
        "compact": log_dir / f"uav_{checked_sys_id}_navigation_compact.csv",
        "debug": log_dir / f"uav_{checked_sys_id}_navigation_debug.csv",
    }
    missing = [path.name for path in paths.values() if not path.is_file()]
    if missing:
        return VehicleReport(
            checked_sys_id,
            role,
            False,
            metrics,
            [f"missing {name}" for name in missing],
        )
    navigation_text = paths["navigation"].read_text(encoding="utf-8", errors="replace")
    mission_text = mission_navigation_segment(navigation_text)
    errors.extend(
        workflow_errors(
            mission_text,
            role=role,
            approval_count=approval_count,
            navigation_speedup=requested_speedup,
            launch_speedup=launch_speed,
            assignment_acked=assignment_acked,
        )
    )
    errors.extend(
        confirmation_image_errors(
            log_dir,
            checked_sys_id,
            mission_text,
            metrics,
        )
    )
    try:
        episodes = parse_final_approach_command_episodes(paths["debug"])
        compact_snaps = parse_compact_snap_distances(paths["compact"])
        debug_snaps = parse_debug_snap_distances(paths["debug"])
    except (OSError, UnicodeError, ValueError) as error:
        errors.append(f"invalid final-approach evidence: {error}")
        return VehicleReport(checked_sys_id, role, False, metrics, errors)
    if len(episodes) != 1:
        errors.append(f"expected exactly one atomic final-approach command episode, found {len(episodes)}")
    if episodes:
        errors.extend(
            _command_errors(
                metrics,
                episodes[-1],
                limits,
                command_bounds,
                expected_speedup=expected_speedup,
            )
        )
    else:
        errors.append(f"observation count 0 < {limits.min_observations}")
    errors.extend(
        _truth_errors(
            metrics,
            navigation_snap=navigation_snap_distance(mission_text),
            compact_snaps=compact_snaps,
            debug_snaps=debug_snaps,
            limit_m=limits.truth.max_snap_distance_m,
        )
    )
    return VehicleReport(checked_sys_id, role, not errors, metrics, errors)


__all__ = ["analyze_vehicle", "navigation_log"]
