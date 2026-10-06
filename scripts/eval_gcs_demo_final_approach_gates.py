"""Independent cadence and roll-quality gates for final-approach evidence."""

from __future__ import annotations

from scripts.eval_gcs_demo_limits import CadenceLimits, RollQualityLimits
from scripts.eval_gcs_demo_models import FinalApproachScore, FinalApproachTiming


def cadence_errors(
    timing: FinalApproachTiming,
    score: FinalApproachScore,
    limits: CadenceLimits,
    *,
    expected_speedup: float,
) -> list[str]:
    errors: list[str] = []
    if score.sample_count < limits.min_observations:
        errors.append(
            f"observation count {score.sample_count} < {limits.min_observations}"
        )
    if timing.median_wall_gap_s is None or timing.max_wall_gap_s is None:
        errors.append("insufficient final-approach timestamps for cadence gate")
    else:
        if timing.median_wall_gap_s > limits.max_median_wall_gap_s:
            errors.append(
                f"median observation gap {timing.median_wall_gap_s:.3f}s > "
                f"{limits.max_median_wall_gap_s:.3f}s"
            )
        if timing.max_wall_gap_s > limits.max_wall_gap_s:
            errors.append(
                f"max observation gap {timing.max_wall_gap_s:.3f}s > "
                f"{limits.max_wall_gap_s:.3f}s"
            )
    minimum, maximum = limits.speedup_bounds(expected_speedup)
    if timing.observed_speedup is None:
        errors.append("insufficient paired timestamps for final-approach speedup gate")
    elif not minimum <= timing.observed_speedup <= maximum:
        errors.append(
            f"final-approach observation speedup {timing.observed_speedup:.3f}x outside "
            f"[{minimum:.3f}, {maximum:.3f}]x for expected "
            f"{expected_speedup:.3f}x"
        )
    return errors


def roll_quality_errors(
    score: FinalApproachScore,
    limits: RollQualityLimits,
) -> list[str]:
    errors: list[str] = []
    if score.significant_reversals > limits.max_significant_reversals:
        errors.append(
            f"significant roll reversals {score.significant_reversals} > "
            f"{limits.max_significant_reversals}"
        )
    if score.saturation_fraction > limits.max_saturation_fraction:
        errors.append(
            f"roll saturation fraction {score.saturation_fraction:.3f} > "
            f"{limits.max_saturation_fraction:.3f}"
        )
    if score.max_saturation_run > limits.max_saturation_run:
        errors.append(
            f"consecutive saturated commands {score.max_saturation_run} > "
            f"{limits.max_saturation_run}"
        )
    if score.max_roll_step_deg > limits.max_roll_step_deg:
        errors.append(
            f"max roll-command step {score.max_roll_step_deg:.2f}deg > "
            f"{limits.max_roll_step_deg:.2f}deg"
        )
    return errors


__all__ = ["cadence_errors", "roll_quality_errors"]
