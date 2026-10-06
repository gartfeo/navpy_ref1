"""Ownership boundary for an injected or locally-built scheduler cadence."""

from __future__ import annotations

from typing import Optional

from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.vision.sim.sim_runtime_ports import OptionalFloatReader


class SchedulerCadenceLease:
    """Close a locally-created cadence without touching an injected one."""

    def __init__(self, cadence: SchedulerCadence, *, owned: bool) -> None:
        self._cadence = cadence
        self._owned = bool(owned)
        self._closed = False

    @classmethod
    def create(
        cls,
        speedup_reader: OptionalFloatReader,
        injected: Optional[SchedulerCadence] = None,
    ) -> "SchedulerCadenceLease":
        if injected is not None:
            return cls(injected, owned=False)
        return cls(SchedulerCadence(speedup_reader), owned=True)

    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float:
        return self._cadence.wall_period_for_scheduler_period(
            scheduler_period_s,
        )

    def close(self) -> None:
        if self._owned and not self._closed:
            self._cadence.close()
        self._closed = True


__all__ = ["SchedulerCadenceLease"]
