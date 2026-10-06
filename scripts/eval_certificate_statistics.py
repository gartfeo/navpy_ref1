"""Across-run and within-run certificate statistics."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Any, Sequence

from scripts.eval_certificate_values import finite_number, require_finite


@dataclass(frozen=True)
class MetricStats:
    count: int
    mean: float
    sample_sd: float | None
    minimum: float
    maximum: float
    value_range: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "count": self.count,
            "mean": self.mean,
            "sample_sd": self.sample_sd,
            "min": self.minimum,
            "max": self.maximum,
            "range": self.value_range,
        }


def summarize(values: Sequence[float]) -> MetricStats | None:
    """Return sample spread for finite evidence values, or ``None``."""
    numbers = [
        number for number in (finite_number(value) for value in values)
        if number is not None
    ]
    if not numbers:
        return None
    return MetricStats(
        count=len(numbers),
        mean=statistics.fmean(numbers),
        sample_sd=statistics.stdev(numbers) if len(numbers) > 1 else None,
        minimum=min(numbers),
        maximum=max(numbers),
        value_range=max(numbers) - min(numbers),
    )


def percentile(values: Sequence[float], q: float) -> float | None:
    """Return a linear-interpolated percentile for finite evidence values."""
    quantile = require_finite(q, name="q", minimum=0.0, maximum=100.0)
    numbers = sorted(
        number for number in (finite_number(value) for value in values)
        if number is not None
    )
    if not numbers:
        return None
    rank = (len(numbers) - 1) * (quantile / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return numbers[int(rank)]
    fraction = rank - low
    return numbers[low] * (1.0 - fraction) + numbers[high] * fraction


@dataclass(frozen=True)
class DistributionStats:
    count: int
    mean: float
    p50: float
    p95: float
    p99: float
    minimum: float
    maximum: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "count": self.count,
            "mean": self.mean,
            "p50": self.p50,
            "p95": self.p95,
            "p99": self.p99,
            "min": self.minimum,
            "max": self.maximum,
        }


def distribution(values: Sequence[float]) -> DistributionStats | None:
    numbers = [
        number for number in (finite_number(value) for value in values)
        if number is not None
    ]
    if not numbers:
        return None
    return DistributionStats(
        count=len(numbers),
        mean=statistics.fmean(numbers),
        p50=percentile(numbers, 50.0),
        p95=percentile(numbers, 95.0),
        p99=percentile(numbers, 99.0),
        minimum=min(numbers),
        maximum=max(numbers),
    )


__all__ = [
    "DistributionStats",
    "MetricStats",
    "distribution",
    "percentile",
    "summarize",
]
