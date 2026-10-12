"""Simulator POI altitude: a height above ground turned into an absolute location."""

from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

from navpy.core.kernel.config.vehicle_arg_ports import ParameterDefaultReader
from navpy.core.kernel.location import Location
from navpy.core.perception.geo.zc_util import ZcUtil


def _real_arg_value(args: object, name: str) -> object:
    try:
        values = vars(args)
    except TypeError:
        values = {}
    if name in values:
        return values[name]
    return None


def _real_cli_overrides(args: object) -> set[str]:
    overrides = _real_arg_value(args, "_cli_overrides")
    return overrides if isinstance(overrides, set) else set()


def _parse_optional_bool(value: object) -> bool | None:
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


class HomeVehicle(Protocol):
    @property
    def home_location(self) -> Location | None: ...


@runtime_checkable
class TerrainHeightReader(Protocol):
    def terrain_height_at(self, lat: float, lng: float) -> float: ...


def _poi_terrain_enabled(args: object, vehicle: HomeVehicle) -> bool:
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


class PoiAltitude:
    """Place a simulator POI at a height above its own ground, relative to home."""

    def __init__(self, args: object, vehicle: HomeVehicle, zc_util: ZcUtil | None) -> None:
        self._args = args
        self._vehicle = vehicle
        self._zc_util = zc_util
        self._use_terrain = _poi_terrain_enabled(args, vehicle)

    def refresh(self) -> None:
        self._use_terrain = _poi_terrain_enabled(self._args, self._vehicle)

    def absolute_location(self, lat: float, lng: float, alt: float) -> Location:
        home = self._vehicle.home_location
        relative = self.relative_alt(lat, lng, alt, home)
        return Location(lat, lng, home.alt + relative, is_absolute=True)

    def relative_alt(
        self,
        lat: float,
        lng: float,
        alt: float,
        home_location: Location,
    ) -> float:
        if not self._use_terrain:
            return float(alt)

        if isinstance(self._vehicle, TerrainHeightReader):
            try:
                terrain_alt = float(self._vehicle.terrain_height_at(lat, lng))
            except Exception:
                terrain_alt = None
            if terrain_alt is not None and math.isfinite(terrain_alt):
                return float(alt + terrain_alt - home_location.alt)

        if self._zc_util is None:
            return alt
        if home_location.lat is not None and home_location.lng is not None:
            home_alt_zc = self._zc_util.get_elevation([home_location.lat, home_location.lng])
        else:
            home_alt_zc = home_location.alt
        poi_alt_zc = self._zc_util.get_elevation([lat, lng])

        if poi_alt_zc is None or home_alt_zc is None:
            return alt

        delta_zc = poi_alt_zc - home_alt_zc
        return float(alt + delta_zc)
