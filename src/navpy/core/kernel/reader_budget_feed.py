"""Where a scheduler cadence sends its wall slot: a bus's reader budget."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol


class ReaderBudgetSink(Protocol):
    """One reader-batch budget request on a MAVLink bus."""

    def set(self, wall_slot_s: float) -> None: ...

    def release(self) -> None: ...


@dataclass(frozen=True)
class ReaderBudgetFeed:
    """The bus request and the scheduler period whose wall slot it receives.

    The cadence sets ``scheduler_period_s / speedup`` on every speedup sync,
    so the vehicle's MAVLink reader batch is bounded by the scheduler it
    feeds (docs/design/reader_batch.md), and releases the request on close.
    """

    sink: ReaderBudgetSink
    scheduler_period_s: float

    def __post_init__(self) -> None:
        period = self.scheduler_period_s
        if (
            isinstance(period, bool)
            or not isinstance(period, (int, float))
            or not math.isfinite(period)
            or period <= 0.0
        ):
            raise ValueError(f"scheduler period must be positive: {period!r}")


__all__ = ["ReaderBudgetFeed", "ReaderBudgetSink"]
