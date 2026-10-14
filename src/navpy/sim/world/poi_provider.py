from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Protocol

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.location import Location
from navpy.sim.world.simulation_object import FrozenSimulationObject
from navpy.core.kernel.config.vehicle_arg_ports import MissionParameterReader
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.sim.world.navigation_poi_args import NavigationPoiArgs
from navpy.sim.world.poi_altitude import PoiAltitude
from navpy.core.kernel.logging.cache_logger import ILogger


class PoiProviderVehicle(MissionParameterReader, Protocol):
    @property
    def home_location(self) -> Location | None: ...

    def get_mission_item_location(self, command_index: int) -> Location: ...


@dataclass(frozen=True)
class PoiInput:
    """One POI's captured inputs: every round trip already answered."""

    command_index: int
    lat: float
    lng: float
    #: The POI's height above its ground (the configured or override altitude).
    height: float
    #: ``height`` turned relative to home (terrain answered at capture).
    relative_alt: float
    #: Home's altitude when it was captured.
    home_alt: float
    location_type: str | None = None


@dataclass(frozen=True)
class CatalogInputs:
    """Everything a catalog is resolved from, captured under the mutation lock."""

    pois: tuple[PoiInput, ...] = ()


def resolve_catalog(inputs: CatalogInputs) -> tuple[FrozenSimulationObject, ...]:
    """Pure: the published POI records of captured inputs; uid is the list index."""
    return tuple(
        FrozenSimulationObject(
            uid,
            Location(poi.lat, poi.lng, poi.home_alt + poi.relative_alt, is_absolute=True),
            poi.height,
            location_type=poi.location_type,
        )
        for uid, poi in enumerate(inputs.pois)
    )


class PoiProvider:
    """Build simulator POIs from selected mission waypoints.

    This does not predict a moving platform; outputs are frozen
    ``SimulationObject`` records.

    A catalog transaction (``lockstep_slice.md``, "NavPy participant"): the
    mutation lock linearizes :meth:`refresh` and :meth:`set_sim_poi` across
    capture (parameter reads and terrain may park on round trips), pure
    resolution and publication; the catalog lock guards only the published
    tuple, so a reader never waits on a round trip. A sim-POI override lives
    until the next refresh rebuilds the list.
    """

    def __init__(
        self,
        args,
        vehicle: PoiProviderVehicle,
        zc_util: ZcUtil,
        logger: ILogger,
    ) -> None:
        self._mutation_lock = rt.Lock()
        self._catalog_lock = rt.Lock()
        self._published: tuple[FrozenSimulationObject, ...] = ()
        self._inputs = CatalogInputs()
        self._raw_args = args

        self.args = NavigationPoiArgs(args, vehicle)

        self.vehicle = vehicle
        self.zc_util = zc_util
        self.logger = logger
        self._altitude = PoiAltitude(self._raw_args, self.vehicle, zc_util)

        with self._mutation_lock:
            self._rebuild()

    @property
    def pois(self) -> tuple[FrozenSimulationObject, ...]:
        with self._catalog_lock:
            return self._published

    def refresh(self):
        with self._mutation_lock:
            self.args.refresh()
            self._altitude.refresh()
            self._rebuild()

    def set_sim_poi(self, command_index: int, location: Location,
                       location_type: str | None = None):
        with self._mutation_lock:
            poi = self._capture_poi(self.vehicle.home_location, command_index, location,
                                    location_type)
            self._publish(replace(self._inputs, pois=self._inputs.pois + (poi,)))
            self.logger.info(f'SimT({len(self._inputs.pois)}): WP{command_index + 1}')

    def _rebuild(self) -> None:
        """Capture every waypoint POI (overrides are dropped), resolve, publish, log."""
        home = self.vehicle.home_location
        if home is None:
            self._publish(CatalogInputs())
            return

        if self.vehicle.mission_items_count == 0:
            self._publish(CatalogInputs())
            self.logger.single_warning('Mission is empty', 'EmptyMission')
            return

        poi_log = ''
        pois: list[PoiInput] = []
        for ordinal, mission_idx in self.args.poi_wp_indices.items():
            pois.append(self._capture_poi(home, mission_idx))
            poi_log += f'P{len(pois)}: wp:{ordinal}(seq:{mission_idx}); '
        self._publish(CatalogInputs(tuple(pois)))

        if poi_log:
            self.logger.info(poi_log, "POI")
        else:
            self.logger.info('No POI is set', 'POI')

    def _capture_poi(self, home: Location | None, command_index: int,
                     location: Location | None = None,
                     location_type: str | None = None) -> PoiInput:
        t_loc = self.vehicle.get_mission_item_location(command_index)

        poi_lat = location.lat if location else t_loc.lat
        poi_lng = location.lng if location else t_loc.lng

        alt = location.alt if location is not None else self.args.poi_alt
        relative = self._altitude.relative_alt(poi_lat, poi_lng, alt, home)
        return PoiInput(command_index, poi_lat, poi_lng, alt, relative, home.alt, location_type)

    def _publish(self, inputs: CatalogInputs) -> None:
        """Resolve, then commit the inputs and publish the records (nothing on a failure)."""
        pois = resolve_catalog(inputs)
        self._inputs = inputs
        with self._catalog_lock:
            self._published = pois


__all__ = ["CatalogInputs", "PoiInput", "PoiProvider", "PoiProviderVehicle", "resolve_catalog"]
