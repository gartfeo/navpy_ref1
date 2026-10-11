"""Select wall-call cadence without modifying scheduler/source time."""

from __future__ import annotations

import math
from numbers import Real
from typing import Protocol, Sequence


class CadenceMember(Protocol):
    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float: ...


class SchedulerCadenceFleet:
    def __init__(self, members: Sequence[CadenceMember]) -> None:
        self._members = tuple(members)

    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float:
        periods = []
        for member in self._members:
            try:
                period = _positive_period(
                    member.wall_period_for_scheduler_period(scheduler_period_s)
                )
            except Exception:
                period = None
            if period is not None:
                periods.append(period)
        return min(periods) if periods else scheduler_period_s


def _positive_period(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    period = float(value)
    return period if math.isfinite(period) and period > 0.0 else None


__all__ = ["SchedulerCadenceFleet"]
