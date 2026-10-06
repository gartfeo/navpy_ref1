"""Raw source-time cadence and issued-command scoring."""

from __future__ import annotations

import statistics
from collections.abc import Sequence

from scripts.eval_gcs_demo_evidence import positive_deltas
from scripts.eval_gcs_demo_models import (
    FinalApproachCommand,
    FinalApproachScore,
    FinalApproachTiming,
    finite_number,
)


def _max_true_run(values: Sequence[bool]) -> int:
    best = 0
    current = 0
    for value in values:
        current = current + 1 if value else 0
        best = max(best, current)
    return best


def _significant_reversals(rolls: Sequence[float], threshold: float) -> int:
    signs = [1 if value > 0 else -1 for value in rolls if abs(value) >= threshold]
    return sum(left != right for left, right in zip(signs, signs[1:]))


def _validated_episode(commands: Sequence[FinalApproachCommand]) -> list[FinalApproachCommand]:
    episode = list(commands)
    if not episode:
        return episode
    if len({command.source_key for command in episode}) != 1:
        raise ValueError("final-approach episode changed atomic source identity")
    pass_indices = [index for index, command in enumerate(episode) if command.passed]
    expected_pass_indices = (
        list(range(pass_indices[0], len(episode))) if pass_indices else []
    )
    if not pass_indices or pass_indices != expected_pass_indices or any(
        command.issued for command in episode[pass_indices[0]:]
    ):
        raise ValueError(
            "final-approach episode requires trailing pass-suppressed rows"
        )
    return episode


def final_approach_timing(commands: Sequence[FinalApproachCommand]) -> FinalApproachTiming:
    episode = _validated_episode(commands)
    first_pass = next(
        (index for index, command in enumerate(episode) if command.passed),
        len(episode) - 1,
    )
    # Pass-suppressed rows are retained to prove the controller remains
    # inactive until SNAP, but they are not part of the issued navigation phase
    # and must not be allowed to improve its cadence/speedup score.
    timed_episode = episode[:first_pass + 1]
    wall_gaps = positive_deltas(command.wall_s for command in timed_episode)
    source_gaps = positive_deltas(command.obs_ts for command in timed_episode)
    if any(gap <= 0.0 for gap in wall_gaps):
        raise ValueError("final-approach wall timestamps must be strictly increasing")
    if any(gap <= 0.0 for gap in source_gaps):
        raise ValueError("final-approach source timestamps must be strictly increasing")
    ratios = [
        source_gap / wall_gap
        for source_gap, wall_gap in zip(source_gaps, wall_gaps)
        if source_gap > 0.0 and wall_gap > 0.0
    ]
    return FinalApproachTiming(
        median_wall_gap_s=statistics.median(wall_gaps) if wall_gaps else None,
        max_wall_gap_s=max(wall_gaps) if wall_gaps else None,
        max_source_gap_s=max(source_gaps) if source_gaps else None,
        observed_speedup=statistics.median(ratios) if ratios else None,
    )


def score_final_approach_commands(
    commands: Sequence[FinalApproachCommand],
    *,
    roll_limit_deg: float,
    saturation_margin_deg: float,
    significant_roll_deg: float,
) -> FinalApproachScore:
    limit = finite_number("roll_limit_deg", roll_limit_deg)
    margin = finite_number("saturation_margin_deg", saturation_margin_deg)
    threshold = finite_number("significant_roll_deg", significant_roll_deg)
    if limit <= 0.0 or margin > limit:
        raise ValueError("roll limit must be positive and margin <= limit")
    issued: list[FinalApproachCommand] = []
    for command in _validated_episode(commands):
        if command.issued:
            issued.append(command)
        if command.passed:
            break
    rolls = [command.cmd_roll for command in issued if command.cmd_roll is not None]
    if len(rolls) != len(issued):
        raise ValueError("issued final-approach command is missing roll evidence")
    saturation = [abs(roll) >= limit - margin for roll in rolls]
    roll_steps = [abs(right - left) for left, right in zip(rolls, rolls[1:])]
    return FinalApproachScore(
        sample_count=len(rolls),
        significant_reversals=_significant_reversals(rolls, threshold),
        saturation_fraction=(sum(saturation) / len(saturation)) if saturation else 0.0,
        max_saturation_run=_max_true_run(saturation),
        max_roll_step_deg=max(roll_steps, default=0.0),
    )


__all__ = ["score_final_approach_commands", "final_approach_timing"]
