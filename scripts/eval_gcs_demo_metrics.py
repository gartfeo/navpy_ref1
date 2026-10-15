"""Raw source-time cadence, issued-command and achieved-roll scoring."""

from __future__ import annotations

import math
import statistics
from collections import Counter
from collections.abc import Sequence
from typing import Optional, Tuple

from scripts.eval_gcs_demo_evidence import positive_deltas
from scripts.eval_gcs_demo_models import (
    AchievedRollScore,
    FinalApproachCommand,
    FinalApproachResponse,
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
    """One source identity and no miss: the parser ends it at DOCKED."""
    episode = list(commands)
    if not episode:
        return episode
    if len({command.source_key for command in episode}) != 1:
        raise ValueError("final-approach episode changed atomic source identity")
    if any(command.missed for command in episode):
        raise ValueError("final-approach episode missed its dock")
    return episode


def final_approach_timing(commands: Sequence[FinalApproachCommand]) -> FinalApproachTiming:
    episode = _validated_episode(commands)
    wall_gaps = positive_deltas(command.wall_s for command in episode)
    source_gaps = positive_deltas(command.obs_ts for command in episode)
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
    issued = [command for command in _validated_episode(commands) if command.issued]
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


_PairKey = Tuple[str, Optional[float]]


def _evidence_error(reason: str, keys: Sequence[_PairKey]) -> str:
    source, obs_ts = keys[0]
    return (
        f"achieved-roll evidence {reason} for {len(keys)} issued command(s) "
        f"(first source={source} obs_ts={obs_ts})"
    )


def _paired_achieved_rolls(
    issued: Sequence[FinalApproachCommand],
    responses: Sequence[FinalApproachResponse],
) -> tuple[list[float], list[str]]:
    """Each issued command's own achieved roll, paired by (source, obs_ts).

    Fail closed: a missing, non-finite, duplicate, repeated-field or
    unmatched achieved roll is a named evidence error, never a skipped sample.
    """
    command_keys: list[_PairKey] = [(item.source, item.obs_ts) for item in issued]
    by_key: dict[_PairKey, list[FinalApproachResponse]] = {}
    for response in responses:
        by_key.setdefault((response.source, response.obs_ts), []).append(response)
    command_counts = Counter(command_keys)
    rolls: list[float] = []
    failed: dict[str, list[_PairKey]] = {}
    for key in command_keys:
        matches = by_key.get(key, [])
        roll = matches[0].aircraft_roll_deg if len(matches) == 1 else None
        if len(matches) > 1 or command_counts[key] > 1:
            failed.setdefault("duplicated", []).append(key)
        elif matches and matches[0].repeated_field:
            failed.setdefault("repeated-field", []).append(key)
        elif roll is None:
            failed.setdefault("missing", []).append(key)
        elif not math.isfinite(roll):
            failed.setdefault("non-finite", []).append(key)
        else:
            rolls.append(roll)
    errors = [_evidence_error(reason, keys) for reason, keys in failed.items()]
    unmatched = [key for key in by_key if key not in command_counts]
    if unmatched:
        rows = sum(len(by_key[key]) for key in unmatched)
        source, obs_ts = unmatched[0]
        errors.append(
            f"achieved-roll evidence unmatched: {rows} response row(s) with no "
            f"issued command (first source={source} obs_ts={obs_ts})"
        )
    return rolls, errors


def score_achieved_roll(
    commands: Sequence[FinalApproachCommand],
    responses: Sequence[FinalApproachResponse],
    *,
    significant_roll_deg: float,
) -> AchievedRollScore:
    """Sign reversals of the achieved roll over the whole episode.

    Counts sign changes among the issued commands' achieved rolls of at
    least *significant_roll_deg*. No reversal count is given when any
    sample's evidence is invalid.
    """
    threshold = finite_number("significant_roll_deg", significant_roll_deg)
    issued = [command for command in _validated_episode(commands) if command.issued]
    rolls, errors = _paired_achieved_rolls(issued, responses)
    if errors:
        return AchievedRollScore(evidence_errors=tuple(errors))
    return AchievedRollScore(reversals=_significant_reversals(rolls, threshold))


__all__ = [
    "final_approach_timing",
    "score_achieved_roll",
    "score_final_approach_commands",
]
