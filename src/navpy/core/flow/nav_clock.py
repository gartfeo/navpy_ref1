"""Navigation clocks and ArduPilot scheduler-cadence adaptation."""

from __future__ import annotations

from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.scheduler_cadence import SchedulerCadence


class NavClock:
    """Keep decision timestamps raw while adapting only scheduler cadence."""

    def __init__(self, scheduler_cadence: Optional[SchedulerCadence]) -> None:
        self._scheduler_cadence = scheduler_cadence

    def decision_s(self) -> float:
        return rt.monotonic()

    def wall_s(self) -> float:
        return rt.monotonic()

    def source_fallback_s(self) -> float:
        return rt.time()

    def scheduler_wall_period(self, scheduler_period_s: float) -> float:
        if self._scheduler_cadence is None:
            return scheduler_period_s
        return self._scheduler_cadence.wall_period_for_scheduler_period(
            scheduler_period_s,
        )

    def sync_scheduler_cadence(self) -> None:
        if self._scheduler_cadence is not None:
            self._scheduler_cadence.sync_speed()


__all__ = ["NavClock"]
