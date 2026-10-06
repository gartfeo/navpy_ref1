"""Thread-safe simulator target snapshots."""

from __future__ import annotations

import threading
from typing import Optional

from navpy.modules.common.models.location import Location
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.sim.sim_control_ports import TargetProviderMutationPort


class SimTargetCatalog:
    """Serialize provider mutations and publish immutable frame snapshots."""

    def __init__(self, provider: TargetProviderMutationPort) -> None:
        self._provider = provider
        self._lock = threading.Lock()
        self._targets = tuple(provider.targets or ())

    @property
    def targets(self) -> tuple[SimulationObject, ...]:
        with self._lock:
            return self._targets

    def snapshot(self) -> tuple[SimulationObject, ...]:
        with self._lock:
            return self._targets

    def refresh(self) -> None:
        with self._lock:
            self._provider.refresh()
            self._targets = tuple(self._provider.targets or ())

    def set_sim_target(
        self,
        command_index: int,
        location: Location,
        location_type: Optional[str] = None,
    ) -> None:
        with self._lock:
            self._provider.set_sim_target(
                command_index,
                location,
                location_type=location_type,
            )
            self._targets = tuple(self._provider.targets or ())


__all__ = ["SimTargetCatalog"]
