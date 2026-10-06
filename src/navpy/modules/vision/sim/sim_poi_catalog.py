"""Thread-safe simulator POI snapshots."""

from __future__ import annotations

import threading
from typing import Optional

from navpy.modules.common.models.location import Location
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.sim.sim_control_ports import PoiProviderMutationPort


class SimPoiCatalog:
    """Serialize provider mutations and publish immutable frame snapshots."""

    def __init__(self, provider: PoiProviderMutationPort) -> None:
        self._provider = provider
        self._lock = threading.Lock()
        self._pois = tuple(provider.pois or ())

    @property
    def pois(self) -> tuple[SimulationObject, ...]:
        with self._lock:
            return self._pois

    def snapshot(self) -> tuple[SimulationObject, ...]:
        with self._lock:
            return self._pois

    def refresh(self) -> None:
        with self._lock:
            self._provider.refresh()
            self._pois = tuple(self._provider.pois or ())

    def set_sim_poi(
        self,
        command_index: int,
        location: Location,
        location_type: Optional[str] = None,
    ) -> None:
        with self._lock:
            self._provider.set_sim_poi(
                command_index,
                location,
                location_type=location_type,
            )
            self._pois = tuple(self._provider.pois or ())


__all__ = ["SimPoiCatalog"]
