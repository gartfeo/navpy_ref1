"""How late in the scored leg the vertical descent actually happens.

Closest-approach distance is a poor observable for a deferred descent: it mixes
the law's behaviour with how much leg was left to recover in.  Two runs with
different leg lengths or different height drops cannot be compared by CPA at
all.

The deferral fraction is dimensionless -- "what fraction of the horizontal leg
had already been flown when a given fraction of the height had been lost" -- so
it compares directly across geometries.  A law that starts descending at
the final-approach start produces roughly equal fractions (10% of the height gone by ~10% of
the leg).  A rate-only law, which commands nothing until the line-of-sight rate
grows, produces fractions far above the diagonal.

Measured on a calm 888 m leg with 340 m to lose: 10% of the height was gone
only after 46% of the leg, and 50% only after 85%.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

# Fractions of the total height drop to report the leg position for.
DESCENT_FRACTIONS = (0.1, 0.25, 0.5, 0.9)


@dataclass(frozen=True)
class DescentDeferral:
    """Where the height went, as fractions of the horizontal leg."""

    entry_horizontal_m: float
    entry_vertical_m: float
    sample_count: int
    # descent fraction -> fraction of the leg already flown when it was reached
    leg_fraction_at: dict[float, float] = field(default_factory=dict)

    @property
    def worst_deferral(self) -> float:
        """How far any checkpoint sits above the ideal diagonal.

        Zero means the height came off in step with the ground covered.
        Positive means the descent lagged the run-in.
        """
        if not self.leg_fraction_at:
            return float("nan")
        return max(
            leg - descent for descent, leg in self.leg_fraction_at.items()
        )


def _rows(path: Path) -> list[tuple[float, float]]:
    """(h_dist, v_dist) pairs, skipping the trailing SNAP summary row."""
    pairs: list[tuple[float, float]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                horizontal = float(row["h_dist"])
                vertical = float(row["v_dist"])
            except (TypeError, ValueError, KeyError):
                continue
            pairs.append((horizontal, vertical))
    return pairs


def deferral(path: Path) -> DescentDeferral:
    """Read a navigation compact log and report when the descent happened."""
    pairs = _rows(path)
    if len(pairs) < 3:
        raise ValueError(f"insufficient navigation rows in {path}: {len(pairs)}")
    entry_h, entry_v = pairs[0]
    if entry_h <= 0.0:
        raise ValueError(f"entry horizontal distance must be positive: {entry_h}")
    if entry_v <= 0.0:
        raise ValueError(
            f"no height to lose at entry ({entry_v} m): deferral is undefined"
        )
    reached: dict[float, float] = {}
    for horizontal, vertical in pairs:
        lost = entry_v - vertical
        flown = (entry_h - horizontal) / entry_h
        for fraction in DESCENT_FRACTIONS:
            if fraction in reached:
                continue
            if lost >= fraction * entry_v:
                reached[fraction] = max(0.0, min(1.0, flown))
    return DescentDeferral(
        entry_horizontal_m=entry_h,
        entry_vertical_m=entry_v,
        sample_count=len(pairs),
        leg_fraction_at=reached,
    )


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()
    header = "  ".join(f"{int(f * 100):>3}%" for f in DESCENT_FRACTIONS)
    print(f"{'log':<34}{'leg m':>8}{'drop m':>8}   {header}   worst")
    for path in args.logs:
        try:
            result = deferral(path)
        except (OSError, ValueError) as error:
            print(f"{path.parent.parent.name:<34} {error}")
            continue
        cells = "  ".join(
            (
                f"{result.leg_fraction_at[f] * 100:>3.0f}%"
                if f in result.leg_fraction_at
                else "   -"
            )
            for f in DESCENT_FRACTIONS
        )
        print(
            f"{path.parent.parent.name:<34}{result.entry_horizontal_m:>8.0f}"
            f"{result.entry_vertical_m:>8.0f}   {cells}   "
            f"{result.worst_deferral * 100:>4.0f}%"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
