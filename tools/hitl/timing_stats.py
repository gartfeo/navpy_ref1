"""Statistics for step 12's timing check: skew summaries and the fitted constants' uncertainty.

A constant C is the median of its calibration pairs. Neighbouring pairs share receipts and the video map's rolling
envelope, so they are not independent; the median's 95 % interval is a moving-block bootstrap (Kuensch 1989), the
established resampling for dependent series, with blocks as long as the map's slowest memory (its 10 s rate
window). The bootstrap is seeded, so a fit is reproducible.
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


def block_count(times_s: Sequence[float], block_s: float) -> int:
    span = max(times_s) - min(times_s)
    return int(math.floor(span / block_s)) + 1


def fit_median(values: Sequence[float], times_s: Sequence[float], block_s: float) -> MedianFit:
    """The median of ``values`` (in time order, at ``times_s``) and its moving-block bootstrap interval."""
    if len(values) != len(times_s) or not values:
        raise ValueError("values and times must be non-empty and the same length")
    order = np.argsort(np.asarray(times_s, dtype=float), kind="stable")
    data = np.asarray(values, dtype=float)[order]
    times = np.asarray(times_s, dtype=float)[order]
    span = float(times[-1] - times[0])
    blocks = block_count(times, block_s)
    count = len(data)
    # Block length in pairs: the mean number of pairs per block_s of the series.
    length = max(1, min(count, int(round(count / blocks))))
    starts = count - length + 1
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    per_sample = math.ceil(count / length)
    picks = rng.integers(0, starts, size=(BOOTSTRAP_RESAMPLES, per_sample))
    index = (picks[:, :, None] + np.arange(length)[None, None, :]).reshape(BOOTSTRAP_RESAMPLES, -1)[:, :count]
    medians = np.median(data[index], axis=1)
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
