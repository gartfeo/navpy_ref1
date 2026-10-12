"""One fixed list of simulator docks shared by every vehicle in a fleet run.

Each NavPy process loads the same file, so a dock has the same id on every
vehicle. Docks are world truth: they never move and are never added at run
time, so ``set_sim_poi`` (a per-vehicle hint about where a dock should be) is
ignored here.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional

from navpy.core.kernel.location import Location
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.sim.world.poi_altitude import HomeVehicle, PoiAltitude
from navpy.sim.world.simulation_object import SimulationObject

FLEET_DOCKS_ENV = "NAVPY_SIM_DOCKS"
SCHEMA_VERSION = 1
_DOCK_KEYS = {"name", "lat", "lon", "height_m", "location_type"}


class FleetDockFileError(ValueError):
    """The fleet dock file is missing or malformed."""


@dataclass(frozen=True)
class DockSpec:
    name: str
    lat: float
    lon: float
    height_m: float
    location_type: Optional[str]


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FleetDockFileError(f"{label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise FleetDockFileError(f"{label} must be finite")
    return number


def _dock(raw: object, index: int) -> DockSpec:
    label = f"dock {index}"
    if not isinstance(raw, dict) or set(raw) != _DOCK_KEYS:
        raise FleetDockFileError(f"{label} must have exactly the keys {sorted(_DOCK_KEYS)}")
    name = raw["name"]
    if not isinstance(name, str) or not name:
        raise FleetDockFileError(f"{label} name must be a non-empty string")
    lat = _finite(raw["lat"], f"{label} lat")
    lon = _finite(raw["lon"], f"{label} lon")
    if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
        raise FleetDockFileError(f"{label} lat/lon out of range")
    location_type = raw["location_type"]
    if location_type is not None and (not isinstance(location_type, str) or not location_type):
        raise FleetDockFileError(f"{label} location_type must be null or a non-empty string")
    return DockSpec(name, lat, lon, _finite(raw["height_m"], f"{label} height_m"), location_type)


def parse_fleet_docks(payload: object) -> tuple[DockSpec, ...]:
    if not isinstance(payload, dict) or set(payload) != {"schema_version", "docks"}:
        raise FleetDockFileError("fleet dock file must have exactly schema_version and docks")
    version = payload["schema_version"]
    if isinstance(version, bool) or version != SCHEMA_VERSION:
        raise FleetDockFileError(f"fleet dock schema_version must be {SCHEMA_VERSION}")
    docks = payload["docks"]
    if not isinstance(docks, list) or not docks:
        raise FleetDockFileError("fleet dock file must list at least one dock")
    specs = tuple(_dock(raw, index) for index, raw in enumerate(docks))
    if len({spec.name for spec in specs}) != len(specs):
        raise FleetDockFileError("fleet dock names must be unique")
    return specs


def load_fleet_docks(path: Path) -> tuple[DockSpec, ...]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise FleetDockFileError(f"cannot read fleet dock file {path}: {error}") from error
    return parse_fleet_docks(payload)


def fleet_docks_path(env: Mapping[str, str] = os.environ) -> Optional[Path]:
    """The dock file a run names; an empty value is a misconfiguration, not absence."""
    raw = env.get(FLEET_DOCKS_ENV)
    if raw is None:
        return None
    if not raw.strip():
        raise FleetDockFileError(f"{FLEET_DOCKS_ENV} is set but empty")
    return Path(raw)


class FleetDockWorld:
    """Fixed fleet docks; uid is the dock's index in the file."""

    def __init__(
        self,
        docks: tuple[DockSpec, ...],
        args: object,
        vehicle: HomeVehicle,
        zc_util: ZcUtil | None,
        logger: ILogger,
    ) -> None:
        self._docks = docks
        self._vehicle = vehicle
        self._logger = logger
        self._altitude = PoiAltitude(args, vehicle, zc_util)
        self.pois: list[SimulationObject] = []
        self._place()

    def refresh(self) -> None:
        self._altitude.refresh()
        self._place()

    def set_sim_poi(
        self,
        command_index: int,
        location: Location,
        location_type: Optional[str] = None,
    ) -> None:
        self._logger.info(
            f"SimT ignored (fleet docks): WP{command_index + 1}",
            "FleetDocks",
        )

    def _place(self) -> None:
        if self._vehicle.home_location is None:
            self.pois = []
            return
        self.pois = [
            SimulationObject(
                uid,
                self._altitude.absolute_location(dock.lat, dock.lon, dock.height_m),
                dock.height_m,
                location_type=dock.location_type,
            )
            for uid, dock in enumerate(self._docks)
        ]
        self._logger.info(
            "Fleet docks: " + "; ".join(
                f"P{uid + 1}={dock.name}" for uid, dock in enumerate(self._docks)
            ),
            "POI",
        )
