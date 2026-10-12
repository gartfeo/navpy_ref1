from __future__ import annotations

from typing import Protocol

from navpy.core.kernel.location import Location
from navpy.sim.world.simulation_object import SimulationObject
from navpy.core.kernel.config.vehicle_arg_ports import MissionParameterReader
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.sim.world.navigation_poi_args import NavigationPoiArgs
from navpy.sim.world.poi_altitude import PoiAltitude
from navpy.core.kernel.logging.cache_logger import ILogger


class PoiProviderVehicle(MissionParameterReader, Protocol):
    @property
    def home_location(self) -> Location | None: ...

    def get_mission_item_location(self, command_index: int) -> Location: ...


class PoiProvider:
    """Build simulator POIs from selected mission waypoints.

    This does not predict a moving platform; outputs are SimulationObject records.
    """

    def __init__(
        self,
        args,
        vehicle: PoiProviderVehicle,
        zc_util: ZcUtil,
        logger: ILogger,
    ) -> None:
        self.pois: list[SimulationObject] | None = None
        self._raw_args = args

        self.args = NavigationPoiArgs(args, vehicle)

        self.vehicle = vehicle
        self.zc_util = zc_util
        self.logger = logger
        self._altitude = PoiAltitude(self._raw_args, self.vehicle, zc_util)

        self._update_get_pois()

    def refresh(self):
        self.args.refresh()
        self._altitude.refresh()
        self._update_get_pois()

    def _update_get_pois(self):
        self.pois = []

        if self.vehicle.home_location is None:
            return

        if self.vehicle.mission_items_count == 0:
            self.logger.single_warning('Mission is empty', 'EmptyMission')
            return

        poi_log = ''

        for ordinal, mission_idx in self.args.poi_wp_indices.items():
            self._add_poi(mission_idx)
            poi_log += f'P{len(self.pois)}: wp:{ordinal}(seq:{mission_idx}); '

        if poi_log:
            self.logger.info(poi_log, "POI")
        else:
            self.logger.info('No POI is set', 'POI')

    def set_sim_poi(self, command_index: int, location: Location,
                       location_type: str | None = None):
        self._add_poi(command_index, location, location_type=location_type)
        self.logger.info(f'SimT({len(self.pois)}): WP{command_index + 1}')

    def _add_poi(self, command_index: int, location=None, location_type: str | None = None):
        t_loc = self.vehicle.get_mission_item_location(command_index)

        poi_lat = location.lat if location else t_loc.lat
        poi_lng = location.lng if location else t_loc.lng

        alt = location.alt if location is not None else self.args.poi_alt

        t_g_loc = self._altitude.absolute_location(poi_lat, poi_lng, alt)
        poi = SimulationObject(len(self.pois), t_g_loc, alt, location_type=location_type)
        self.pois.append(poi)
