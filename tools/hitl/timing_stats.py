"""Statistics for step 12's timing check: skew summaries and the fitted constants' uncertainty.

A constant C is the median of its calibration pairs. Neighbouring pairs share receipts and the video map's rolling
envelope, so they are not independent; the median's 95 % interval is a non-overlapping block bootstrap (Carlstein
1986; Kuensch 1989 for its moving-block form), the established resampling for dependent series. A block is one
time bin as long as the map's slowest memory (its 10 s rate window), so pairs in one bin stay together, and every
occupied bin is drawn with the same chance whatever its pair count. The bootstrap is seeded, so a fit is
reproducible.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

BOOTSTRAP_RESAMPLES = 2000
BOOTSTRAP_SEED = 12
CONFIDENCE = 0.95
# Fewer blocks than this cannot show the spread between blocks: the fit refuses (my choice, not in the plan).
MIN_BLOCKS = 3


@dataclass(frozen=True)
class SkewSummary:
    count: int
    median_s: float
    p95_abs_s: float
    max_abs_s: float

    def record(self) -> dict[str, float | int]:
        return {"count": self.count, "median_s": self.median_s, "p95_abs_s": self.p95_abs_s,
                "max_abs_s": self.max_abs_s}


def summarize(values: Sequence[float]) -> SkewSummary:
    """Signed median, and p95 and max of the magnitude (numpy's linear percentile)."""
    if not values:
        raise ValueError("no values to summarize")
    data = np.asarray(values, dtype=float)
    magnitude = np.abs(data)
    return SkewSummary(len(data), float(np.median(data)), float(np.percentile(magnitude, 95.0)),
                       float(magnitude.max()))


@dataclass(frozen=True)
class MedianFit:
    value_s: float
    lower_s: float
    upper_s: float
    pairs: int
    blocks: int
    span_s: float

    @property
    def uncertainty_s(self) -> float:
        """The larger distance from the median to its interval's ends: the interval need not be centred."""
        return max(abs(self.value_s - self.lower_s), abs(self.upper_s - self.value_s))

    def record(self) -> dict[str, float | int]:
        return {"correction_s": self.value_s, "ci_s": [self.lower_s, self.upper_s],
                "uncertainty_s": self.uncertainty_s, "pairs": self.pairs, "blocks": self.blocks,
                "span_s": self.span_s}


def _bins(times_s: Sequence[float], block_s: float) -> list[int]:
    """The block_s bin of each time, counted from the first time."""
    first = min(times_s)
    return [int(math.floor((time - first) / block_s)) for time in times_s]


def block_count(times_s: Sequence[float], block_s: float) -> int:
    """How many block_s bins from the first time hold at least one pair: empty stretches are no evidence."""
    return len(set(_bins(times_s, block_s)))


def fit_median(values: Sequence[float], times_s: Sequence[float], block_s: float) -> MedianFit:
    """The median of ``values`` (at ``times_s``) and its block bootstrap interval over occupied block_s bins.

    Each resample draws as many bins as are occupied, uniformly with replacement, and keeps each drawn bin whole.
    """
    if len(values) != len(times_s) or not values:
        raise ValueError("values and times must be non-empty and the same length")
    data = np.asarray(values, dtype=float)
    times = np.asarray(times_s, dtype=float)
    if not np.all(np.isfinite(data)) or not np.all(np.isfinite(times)) or not math.isfinite(block_s) \
            or block_s <= 0.0:
        raise ValueError("values, times and the block length must be finite, the block length positive")
    span = float(times.max() - times.min())
    bins = np.asarray(_bins(times, block_s))
    groups = [data[bins == label] for label in np.unique(bins)]
    blocks = len(groups)
    if blocks < MIN_BLOCKS:
        raise ValueError(f"pairs fill {blocks} blocks of {block_s:g} s, fewer than {MIN_BLOCKS}")
    count = len(data)
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    picks = rng.integers(0, blocks, size=(BOOTSTRAP_RESAMPLES, blocks))
    medians = np.array([np.median(np.concatenate([groups[index] for index in row])) for row in picks])
    tail = (1.0 - CONFIDENCE) / 2.0 * 100.0
    lower, upper = np.percentile(medians, [tail, 100.0 - tail])
    return MedianFit(float(np.median(data)), float(lower), float(upper), count, blocks, span)


__all__ = [
    "BOOTSTRAP_RESAMPLES",
    "BOOTSTRAP_SEED",
    "CONFIDENCE",
    "MIN_BLOCKS",
    "MedianFit",
    "SkewSummary",
    "block_count",
    "fit_median",
    "summarize",
]
