"""Navigation clocks and ArduPilot scheduler-cadence adaptation."""

from __future__ import annotations

import time
from typing import Optional

from navpy.modules.common.scheduler_cadence import SchedulerCadence


class NavClock:
    """Keep decision timestamps raw while adapting only scheduler cadence."""

    def __init__(self, scheduler_cadence: Optional[SchedulerCadence]) -> None:
        self._scheduler_cadence = scheduler_cadence

    def decision_s(self) -> float:
        return time.monotonic()

    def wall_s(self) -> float:
        return time.monotonic()

    def source_fallback_s(self) -> float:
        return time.time()

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
