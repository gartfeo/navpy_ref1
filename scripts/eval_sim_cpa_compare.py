"""Exact common-episode comparison of module and stream CPA scores.

The module's epoch-global minimum spans its whole configuration epoch;
the certified stream score spans the scored episode.  Comparing them
raw compares different flights.  This module derives one common episode
both sides cover exactly: the scored stream window trimmed inward to
the module's 20 ms interval grid.  The module side is the minimum over
the full interval rows inside that window -- exact by the interval
contract.  The stream side is recomputed over its track clipped to the
identical boundaries and must reproduce the official certified CPA to
float precision, which proves the trim did not change the scored event;
when it cannot (a CPA outside or at the very edge of the window), the
comparison is inadmissible rather than approximately right.

Pure logic only: inputs are parsed rows, track records and the case
target; no file IO beyond what the caller hands in.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Sequence

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_models import PositionSample, TargetLocation  # noqa: E402
from eval_navigation_truth_cpa import canonical_samples, segment_cpa  # noqa: E402
from eval_navigation_truth_samples import TruthRecord  # noqa: E402
from eval_sim_cpa_bin import (  # noqa: E402
    EpochEvidence, FLAG_PARTIAL, INTERVAL_US,
)
from eval_sim_cpa_block import (  # noqa: E402
    COMPARISON_INADMISSIBLE, EVIDENCE_ACCEPTED, EVIDENCE_INCOMPLETE,
    compare_scores, default_comparison,
)

# The clipped recompute must give back the official stream CPA to within
# float-noise; anything larger means the trim removed or moved the scored
# event and the two sides no longer measure the same episode.
REPRODUCE_DIST_TOL_M = 1e-6
REPRODUCE_TIME_TOL_S = 1e-6


def scoring_active_samples_from_records(
    records: Sequence[TruthRecord],
) -> list[PositionSample]:
    """Canonical scored samples rebuilt from a persisted truth track.

    Uses exactly the rows the live scorer scored: scored AND converted.
    ``rel_alt_m`` is not persisted and not used by the CPA geometry
    (``_local_vector`` reads lat/lon/abs_alt only), so it is zeroed.
    """
    samples = [
        PositionSample(
            lat_deg=record.lat_deg,
            lon_deg=record.lon_deg,
            abs_alt_m=record.abs_alt_m,
            rel_alt_m=0.0,
            received_wall_time_s=record.received_wall_time_s,
            source_time_s=record.source_time_s,
        )
        for record in records
        if record.scoring_active and record.converted
        and record.lat_deg is not None
        and record.lon_deg is not None
        and record.abs_alt_m is not None
        and record.source_time_s is not None
    ]
    canonical, _ = canonical_samples(samples)
    return canonical


def _interpolated(
    before: PositionSample, after: PositionSample, time_s: float
) -> PositionSample:
    span = after.source_time_s - before.source_time_s
    fraction = 0.0 if span <= 0.0 else (time_s - before.source_time_s) / span
    return PositionSample(
        lat_deg=before.lat_deg + fraction * (after.lat_deg - before.lat_deg),
        lon_deg=before.lon_deg + fraction * (after.lon_deg - before.lon_deg),
        abs_alt_m=(
            before.abs_alt_m + fraction * (after.abs_alt_m - before.abs_alt_m)
        ),
        rel_alt_m=0.0,
        received_wall_time_s=before.received_wall_time_s,
        source_time_s=time_s,
    )


def clip_track(
    canonical: Sequence[PositionSample],
    start_s: float,
    end_s: float,
) -> list[PositionSample]:
    """The piecewise-linear track restricted to [start_s, end_s].

    Boundary points are linearly interpolated so the clipped track is the
    same curve over the retained span, not a resampling of it.
    """
    inside = [
        sample for sample in canonical
        if start_s <= sample.source_time_s <= end_s
    ]
    clipped: list[PositionSample] = []
    for index in range(1, len(canonical)):
        before, after = canonical[index - 1], canonical[index]
        if before.source_time_s < start_s < after.source_time_s:
            clipped.append(_interpolated(before, after, start_s))
        if before.source_time_s < end_s < after.source_time_s:
            inside.append(_interpolated(before, after, end_s))
    clipped.extend(inside)
    clipped.sort(key=lambda sample: sample.source_time_s)
    return clipped


def common_episode(
    canonical: Sequence[PositionSample],
    epoch_us: int,
) -> dict[str, Any] | None:
    """The 20 ms-grid-aligned episode inside the scored stream window."""
    if len(canonical) < 2:
        return None
    start_us = round(canonical[0].source_time_s * 1e6)
    end_us = round(canonical[-1].source_time_s * 1e6)
    if start_us < epoch_us:
        # The epoch opened after scoring interval began (a mid-scoring interval
        # enable/retarget): the module never observed the early scored
        # span, so no window over the remainder measures the same episode.
        # Shrinking the window here would silently reintroduce the episode
        # mismatch this whole module exists to prevent.
        return None
    first_seq = -(-(start_us - epoch_us) // INTERVAL_US)  # ceil division
    last_boundary_seq = (end_us - epoch_us) // INTERVAL_US
    if last_boundary_seq <= first_seq:
        return None
    return {
        "start_us": epoch_us + first_seq * INTERVAL_US,
        "end_us": epoch_us + last_boundary_seq * INTERVAL_US,
        "first_seq": int(first_seq),
        # Interval N spans [EpUS+N*20000, EpUS+(N+1)*20000): the last FULL
        # interval inside the window ends at the last boundary.
        "last_seq": int(last_boundary_seq) - 1,
    }


def module_window_score(
    evidence: EpochEvidence,
    episode: dict[str, Any],
) -> tuple[dict[str, Any] | None, list[str]]:
    """Minimum over the full interval rows covering the episode exactly.

    Every Seq in [first_seq, last_seq] must be present as a FULL interval
    row: a missing or partial row inside the window means the module did
    not observe the whole episode and no minimum over the remainder is
    admissible evidence.
    """
    needed = range(episode["first_seq"], episode["last_seq"] + 1)
    by_seq = {row["Seq"]: row for row in evidence.rows}
    problems: list[str] = []
    missing = [seq for seq in needed if seq not in by_seq]
    if missing:
        problems.append(
            f"{len(missing)} interval row(s) missing inside the common "
            f"episode (first {missing[:5]})"
        )
    partial = [
        seq for seq in needed
        if seq in by_seq and by_seq[seq].get("Fl", 0) & FLAG_PARTIAL
    ]
    if partial:
        problems.append(
            f"partial interval row(s) inside the common episode: {partial[:5]}"
        )
    if problems:
        return None, problems
    best = min((by_seq[seq] for seq in needed), key=lambda row: row["D3"])
    return {
        "d3_m": best["D3"],
        "dh_m": best["DH"],
        "dv_m": best["DV"],
        "cpa_time_s": best["CpaUS"] / 1e6,
        "seq": best["Seq"],
        "interval_rows": len(list(needed)),
    }, []


def stream_window_score(
    canonical: Sequence[PositionSample],
    target: TargetLocation,
    episode: dict[str, Any],
    official: dict[str, Any],
) -> tuple[dict[str, Any] | None, list[str]]:
    """The stream CPA over the identical episode, proven identical.

    The official certified CPA time must lie inside the episode and the
    clipped recompute must reproduce the official distance and time to
    float precision.  Otherwise the trim changed the event being scored
    and the comparison is inadmissible.
    """
    problems: list[str] = []
    official_d3 = official.get("dist_3d_m")
    official_time = official.get("cpa_source_time_s")
    if not isinstance(official_d3, (int, float)) or isinstance(
        official_d3, bool
    ) or not isinstance(official_time, (int, float)) or isinstance(
        official_time, bool
    ):
        return None, ["official stream CPA is not present"]
    start_s = episode["start_us"] / 1e6
    end_s = episode["end_us"] / 1e6
    if not start_s < official_time < end_s:
        return None, [
            f"official stream CPA time {official_time:.6f}s is not interior "
            f"to the common episode [{start_s:.6f}, {end_s:.6f}]"
        ]
    clipped = clip_track(canonical, start_s, end_s)
    recomputed = segment_cpa(clipped, target)
    if recomputed is None:
        return None, ["clipped track produced no CPA"]
    if abs(recomputed.dist_3d_m - official_d3) > REPRODUCE_DIST_TOL_M or (
        abs(recomputed.source_time_s - official_time) > REPRODUCE_TIME_TOL_S
    ):
        problems.append(
            "clipped stream recompute does not reproduce the official CPA "
            f"(d3 {recomputed.dist_3d_m:.9f} vs {official_d3:.9f}, "
            f"t {recomputed.source_time_s:.9f} vs {official_time:.9f}); "
            "the episode trim changed the scored event"
        )
        return None, problems
    # Components come from the official truth block: the recompute proved
    # the same event, so the certified split is the authoritative one.
    return {
        "d3_m": float(official_d3),
        "dh_m": official.get("horizontal_m"),
        "dv_m": official.get("vertical_m"),
        "cpa_time_s": float(official_time),
    }, []


def windowed_comparison(
    evidence: EpochEvidence,
    records: Sequence[TruthRecord],
    target: TargetLocation,
    official_truth: dict[str, Any],
) -> tuple[dict[str, Any] | None, dict[str, Any] | None, dict[str, Any]]:
    """(episode, module_score, comparison) for one case.

    The caller decides how an inadmissible comparison lands in the block;
    this returns the evidence-layer facts and the comparison layer.
    """
    comparison = default_comparison()
    if evidence.status != EVIDENCE_ACCEPTED:
        comparison["status"] = COMPARISON_INADMISSIBLE
        comparison["error"] = f"module evidence {evidence.status}"
        return None, None, comparison
    canonical = scoring_active_samples_from_records(records)
    episode = (
        None if evidence.epoch_us is None
        else common_episode(canonical, evidence.epoch_us)
    )
    if episode is None:
        comparison["status"] = COMPARISON_INADMISSIBLE
        comparison["error"] = (
            "no grid-aligned common episode inside the engaged stream window"
        )
        return None, None, comparison
    module_score, module_problems = module_window_score(evidence, episode)
    if module_score is None:
        comparison["status"] = COMPARISON_INADMISSIBLE
        comparison["error"] = "; ".join(module_problems)
        return episode, None, comparison
    stream_score, stream_problems = stream_window_score(
        canonical, target, episode, official_truth
    )
    if stream_score is None:
        comparison["status"] = COMPARISON_INADMISSIBLE
        comparison["error"] = "; ".join(stream_problems)
        return episode, module_score, comparison
    return episode, module_score, compare_scores(module_score, stream_score)


__all__ = [
    "EVIDENCE_INCOMPLETE",
    "REPRODUCE_DIST_TOL_M",
    "REPRODUCE_TIME_TOL_S",
    "clip_track",
    "common_episode",
    'scoring_active_samples_from_records',
    "module_window_score",
    "stream_window_score",
    "windowed_comparison",
]
