"""Stable terrain-query API composed from focused DEM services."""

from __future__ import annotations

from typing import List, Optional, Tuple, Union

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.attitude_ray_converter import AttitudeRayConverter
from navpy.modules.navigation.geo.dem_data import DemData
from navpy.modules.navigation.geo.dem_tile_repository import (
    DemTileRepository,
    default_dem_directory,
    valid_coords,
)
from navpy.modules.navigation.geo.terrain_intersection_search import (
    TerrainIntersectionSearch,
)
from navpy.modules.navigation.geo.terrain_ray_projector import TerrainRayProjector


class ZcUtil:
    """Public composition boundary for terrain elevation and projection."""

    def __init__(
        self,
        max_distance: float = 3000.0,
        degrees: bool = True,
        coarse_samples: int = 10,
        dem_data_dir: str | None = None,
        *,
        tile_repository: DemTileRepository | None = None,
    ) -> None:
        tiles = tile_repository or DemTileRepository(
            dem_data_dir or default_dem_directory()
        )
        search = TerrainIntersectionSearch(max_distance, coarse_samples)
        self._tiles = tiles
        self._attitude_ray = AttitudeRayConverter(degrees)
        self._terrain_ray = TerrainRayProjector(
            max_distance,
            tiles,
            search,
        )

    def get_elevation(
        self,
        coords: Union[List[float], Tuple[float, float]],
    ) -> Optional[float]:
        return self._tiles.get_elevation(coords)

    def ray_to_terrain(
        self,
        ray_origin_gps: Location,
        att: Attitude,
        home_alt: float = 0.0,
        delta_alt: float = 0.0,
        seq: str = "ZYX",
        allow_fallback: bool = True,
    ) -> Optional[Location]:
        if not self.valid_coords(ray_origin_gps.lat, ray_origin_gps.lng):
            return None
        direction_ned = self._attitude_ray.to_ned(att, seq)
        return self.ray_to_terrain_ned(
            ray_origin_gps,
            direction_ned,
            home_alt,
            delta_alt,
            allow_fallback,
        )

    def ray_to_terrain_ned(
        self,
        ray_origin_gps: Location,
        direction_ned: np.ndarray,
        home_alt: float = 0.0,
        delta_alt: float = 0.0,
        allow_fallback: bool = True,
    ) -> Optional[Location]:
        return self._terrain_ray.project(
            ray_origin_gps,
            direction_ned,
            home_alt,
            delta_alt,
            allow_fallback,
        )

    @staticmethod
    def valid_coords(lat: float, lon: float) -> bool:
        return valid_coords(lat, lon)


__all__ = ["DemData", "ZcUtil"]
