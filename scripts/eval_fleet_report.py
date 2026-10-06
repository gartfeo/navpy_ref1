"""Read a fleet verdict back as a per-cell table.

Medians per cell, never the best run and never the worst. Run-to-run scatter
on this bench has been measured at sd 0.096 m over five identical solo runs, so
a cell read from its best aircraft says nothing about the cell.
"""

from __future__ import annotations

import statistics
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from scripts.eval_direct_pixel_verdict import (  # noqa: E402
    SCORING_SOURCE_ESTIMATE,
    SCORING_SOURCE_TRUTH,
)


def miss_3d_m(row: dict) -> float | None:
    """The miss from the source that actually scored this row -- ONLY that.

    A truth-scored row's authoritative miss is the certified simulator truth;
    its `coordinate` block is the EKF's own error projection, kept as audit
    data -- measured ANTI-correlated with the real miss, so tabling it for a
    truth-scored fleet would rank the cells backwards. That is exactly why an
    UNCERTIFIED truth-policy row (`scoring_source: none`) must return None
    rather than fall back to the coordinate block: the fallback would feed
    the anti-ranked projection into the medians precisely on the rows where
    certification failed. Estimate rows read `coordinate`, not `closest`:
    `closest` is the scorer's own sample record and comes back null in the
    verdict; reading it once made a fully scored fleet print an empty table.
    """
    source = row.get("scoring_source")
    if source == SCORING_SOURCE_TRUTH:
        block = row.get("truth")
    elif source == SCORING_SOURCE_ESTIMATE:
        block = row.get("coordinate")
    else:
        return None
    value = (block or {}).get("dist_3d_m")
    return None if value is None else float(value)


def by_cell(vehicles: dict[str, dict]) -> dict[tuple[float, float], list[float]]:
    """Certified misses per cell; a row without one stays out of the medians.

    `valid` gates entry: an invalid row's miss may exist but its run does
    not stand as evidence, and a table that quietly averaged it would look
    healthier (or sicker) than the campaign it summarises. `report` prints
    how many rows were left out, so the table can never silently pass for
    complete.
    """
    cells: dict[tuple[float, float], list[float]] = {}
    for row in vehicles.values():
        miss = miss_3d_m(row)
        if row.get("valid") is True and miss is not None:
            key = (row["wind_speed_mps"], row["wind_dir_deg"])
            cells.setdefault(key, []).append(miss)
    return cells


def report(vehicles: dict[str, dict], goal: float) -> None:
    cells = by_cell(vehicles)
    excluded = len(vehicles) - sum(len(values) for values in cells.values())
    if excluded:
        print(
            f"\nEXCLUDED {excluded} of {len(vehicles)} rows: no certified "
            "score (invalid, unscored, or uncertified truth)"
        )
    print(f"\n{'wind':>14}   n   median 3d    best    worst   under {goal}")
    for (speed, direction), values in sorted(cells.items()):
        label = "calm" if speed == 0 else f"{speed:g} m/s @ {direction:g}"
        met = sum(1 for value in values if value < goal)
        print(
            f"{label:>14} {len(values):>3}     {statistics.median(values):.4f}  "
            f"{min(values):.4f}  {max(values):.4f}   {met}/{len(values)}"
        )
    every = [value for values in cells.values() for value in values]
    if every:
        met = sum(1 for value in every if value < goal)
        print(
            f"{'ALL':>14} {len(every):>3}     {statistics.median(every):.4f}  "
            f"{min(every):.4f}  {max(every):.4f}   {met}/{len(every)}"
        )


__all__ = ["by_cell", "miss_3d_m", "report"]
