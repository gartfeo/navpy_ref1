from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

from navpy.modules.common.models.location import Location
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.args.vehicle_arg_ports import (
    MissionParameterReader,
    ParameterDefaultReader,
)
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.args.navigation_poi_args import NavigationPoiArgs
from navpy.logger.cache_logger import ILogger


def _real_arg_value(args, name: str):
    try:
        values = vars(args)
    except TypeError:
        values = {}
    if name in values:
        return values[name]
    return None


def _real_cli_overrides(args) -> set[str]:
    overrides = _real_arg_value(args, "_cli_overrides")
    return overrides if isinstance(overrides, set) else set()


def _parse_optional_bool(value) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "y", "1", "t", "on"}:
            return True
        if normalized in {"false", "no", "n", "0", "f", "off"}:
            return False
    return None


class PoiProviderVehicle(MissionParameterReader, Protocol):
    @property
    def home_location(self) -> Location | None: ...

    def get_mission_item_location(self, command_index: int) -> Location: ...


@runtime_checkable
class TerrainHeightReader(Protocol):
    def terrain_height_at(self, lat: float, lng: float) -> float: ...


def _poi_terrain_enabled(args, vehicle: PoiProviderVehicle) -> bool:
    resolved_value = _parse_optional_bool(_real_arg_value(args, "use_terrain"))
    if resolved_value is not None:
        return resolved_value

    cli_value = _real_arg_value(args, "AAS_USE_TRN")
    if "AAS_USE_TRN" in _real_cli_overrides(args):
        resolved_value = _parse_optional_bool(cli_value)
        if resolved_value is not None:
            return resolved_value

    default_value = _parse_optional_bool(cli_value)
    if default_value is not None:
        if isinstance(vehicle, ParameterDefaultReader):
            try:
                param_value = vehicle.get_param_or_default(
                    "AAS_USE_TRN",
                    default_value,
                )
            except Exception:
                param_value = default_value
            resolved_value = _parse_optional_bool(param_value)
            if resolved_value is not None:
                return resolved_value
        return default_value

    return False


class PoiProvider:
    """Build simulator delivery references from selected mission waypoints.

    The compatible PoiProvider name does not imply recipient authorization
    or prediction of a moving recipient platform; outputs are SimulationObject records.
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
        self._use_terrain = _poi_terrain_enabled(self._raw_args, self.vehicle)

        self._update_get_pois()

    def refresh(self):
        self.args.refresh()
        self._use_terrain = _poi_terrain_enabled(self._raw_args, self.vehicle)
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

    def _calc_terrain_relative_alt(self, lat, lng, alt, home_location) -> float:
        if not self._use_terrain:
            return float(alt)

        if isinstance(self.vehicle, TerrainHeightReader):
            try:
                terrain_alt = float(self.vehicle.terrain_height_at(lat, lng))
            except Exception:
                terrain_alt = None
            if terrain_alt is not None and math.isfinite(terrain_alt):
                return float(alt + terrain_alt - home_location.alt)

        if self.zc_util is None:
            return alt
        if home_location.lat is not None and home_location.lng is not None:
            home_alt_zc = self.zc_util.get_elevation([home_location.lat, home_location.lng])
        else:
            home_alt_zc = home_location.alt
        poi_alt_zc = self.zc_util.get_elevation([lat, lng])

        if poi_alt_zc is None or home_alt_zc is None:
            return alt

        delta_zc = poi_alt_zc - home_alt_zc
        return float(alt + delta_zc)

    def set_sim_poi(self, command_index: int, location: Location,
                       location_type: str | None = None):
        self._add_poi(command_index, location, location_type=location_type)
        self.logger.info(f'SimT({len(self.pois)}): WP{command_index + 1}')

    def _add_poi(self, command_index: int, location=None, location_type: str | None = None):
        t_loc = self.vehicle.get_mission_item_location(command_index)

        poi_lat = location.lat if location else t_loc.lat
        poi_lng = location.lng if location else t_loc.lng

        alt = location.alt if location is not None else self.args.poi_alt

        poi_r_alt = self._calc_terrain_relative_alt(poi_lat, poi_lng, alt,
                                                       self.vehicle.home_location)

        t_g_loc = Location(poi_lat, poi_lng, self.vehicle.home_location.alt + poi_r_alt,
                           is_absolute=True)
        poi = SimulationObject(len(self.pois), t_g_loc, alt, location_type=location_type)
        self.pois.append(poi)
