"""Thread-safe simulator POI snapshots."""

from __future__ import annotations

from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.location import Location
from navpy.sim.world.simulation_object import SimulationObject
from navpy.sim.detection.sim_control_ports import PoiProviderMutationPort


class SimPoiCatalog:
    """Serialize provider mutations and publish immutable frame snapshots.

    The mutation lock linearizes ``refresh`` and ``set_sim_poi`` across the
    provider's round trips (parameter reads, terrain); the catalog lock
    guards only the published tuple, so a frame reader never waits on a
    mutation in progress (``lockstep_slice.md``, "Catalog transaction").
    """

    def __init__(self, provider: PoiProviderMutationPort) -> None:
        self._provider = provider
        self._mutation_lock = rt.Lock()
        self._lock = rt.Lock()
        self._pois = tuple(provider.pois or ())

    @property
    def pois(self) -> tuple[SimulationObject, ...]:
        with self._lock:
            return self._pois

    def snapshot(self) -> tuple[SimulationObject, ...]:
        with self._lock:
            return self._pois

    def refresh(self) -> None:
        with self._mutation_lock:
            self._provider.refresh()
            self._publish()

    def set_sim_poi(
        self,
        command_index: int,
        location: Location,
        location_type: Optional[str] = None,
    ) -> None:
        with self._mutation_lock:
            self._provider.set_sim_poi(
                command_index,
                location,
                location_type=location_type,
            )
            self._publish()

    def _publish(self) -> None:
        pois = tuple(self._provider.pois or ())
        with self._lock:
            self._pois = pois


__all__ = ["SimPoiCatalog"]
