"""In-run simulator clock-rate measurement."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from scripts.eval_certificate_values import finite_number, require_finite


MIN_RATE_SPAN_S = 2.0
SOURCE_RESET_BACKSTEP_S = 5.0


@dataclass(frozen=True)
class ClockRate:
    """An in-run sim-clock measurement, or the reason there is not one."""

    rate: float | None
    span_s: float
    sample_count: int
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "rate": self.rate,
            "span_s": self.span_s,
            "sample_count": self.sample_count,
            "error": self.error,
        }


class ClockRateTracker:
    """Measure achieved source seconds per monotonic wall second."""

    def __init__(
        self,
        *,
        min_span_s: float = MIN_RATE_SPAN_S,
        reset_backstep_s: float = SOURCE_RESET_BACKSTEP_S,
    ) -> None:
        self._min_span_s = require_finite(
            min_span_s,
            name="min_span_s",
            minimum=0.0,
        )
        self._reset_backstep_s = require_finite(
            reset_backstep_s,
            name="reset_backstep_s",
            minimum=0.0,
        )
        self._earliest: tuple[float, float] | None = None
        self._latest: tuple[float, float] | None = None
        self._max_source_s: float | None = None
        self._count = 0
        self._error: str | None = None

    def add(
        self,
        source_time_s: float | None,
        wall_time_s: float | None,
    ) -> None:
        """Take one sample; missing or non-finite observations are ignored."""
        source = finite_number(source_time_s)
        wall = finite_number(wall_time_s)
        if source is None or wall is None:
            return
        self._count += 1
        if self._max_source_s is not None:
            backwards = self._max_source_s - source
            if backwards > self._reset_backstep_s and self._error is None:
                self._error = (
                    f"source clock went backwards by {backwards:.3f}s, past the "
                    f"{self._reset_backstep_s:.3f}s reorder allowance"
                )
        if self._max_source_s is None or source > self._max_source_s:
            self._max_source_s = source
        if self._earliest is None or source < self._earliest[0]:
            self._earliest = (source, wall)
        if self._latest is None or source > self._latest[0]:
            self._latest = (source, wall)

    @property
    def result(self) -> ClockRate:
        if self._earliest is None or self._latest is None:
            return ClockRate(
                None,
                0.0,
                self._count,
                self._error or "no timestamped position samples",
            )
        wall_span = self._latest[1] - self._earliest[1]
        source_span = self._latest[0] - self._earliest[0]
        if self._error is not None:
            return ClockRate(None, wall_span, self._count, self._error)
        if wall_span <= 0.0 or wall_span < self._min_span_s:
            return ClockRate(
                None,
                wall_span,
                self._count,
                f"sample span {wall_span:.3f}s is under the "
                f"{self._min_span_s:.3f}s minimum, so no rate was measured",
            )
        return ClockRate(source_span / wall_span, wall_span, self._count, None)


__all__ = [
    "ClockRate",
    "ClockRateTracker",
    "MIN_RATE_SPAN_S",
    "SOURCE_RESET_BACKSTEP_S",
]
