"""Terrain projection orchestration for a NED ray."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Optional

import numpy as np
from geopy.distance import geodesic

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.dem_tile_repository import (
    DemTileRepository,
    valid_coords,
)
from navpy.modules.navigation.geo.dem_data import DemData
from navpy.modules.navigation.geo.terrain_intersection_search import (
    TerrainIntersectionSearch,
    TerrainRay,
)


class TerrainRayProjector:
    """Project a NED direction onto terrain or a bounded fallback point."""

    def __init__(
        self,
        max_distance: float,
        tiles: DemTileRepository,
        search: TerrainIntersectionSearch,
    ) -> None:
        self._max_distance = max_distance
        self._tiles = tiles
        self._search = search

    def project(
        self,
        origin: Location,
        direction_ned: Sequence[float],
        home_alt: float = 0.0,
        delta_alt: float = 0.0,
        allow_fallback: bool = True,
    ) -> Optional[Location]:
        lat0, lon0 = origin.lat, origin.lng
        if not valid_coords(lat0, lon0):
            return None
        tile = self._tiles.get_tile((lat0, lon0))
        if tile is None:
            return None

        alt0 = origin.alt + home_alt
        north, east, down = direction_ned
        bearing_deg = np.degrees(np.arctan2(east, north))
        horizontal_norm = np.sqrt(north**2 + east**2)
        if horizontal_norm < 1e-9:
            return self._vertical_result(
                tile,
                lat0,
                lon0,
                alt0,
                down,
                home_alt,
                delta_alt,
                allow_fallback,
            )

        ray = TerrainRay(
            tile=tile,
            lat0=lat0,
            lon0=lon0,
            alt0=alt0,
            down=down,
            delta_alt=delta_alt,
            bearing_deg=bearing_deg,
            horizontal_norm=horizontal_norm,
        )
        distance = self._search.find(ray)
        if distance is None:
            return self._fallback(ray, home_alt) if allow_fallback else None

        final_point = geodesic(meters=distance).destination(
            (lat0, lon0),
            bearing_deg,
        )
        terrain = tile.get_height(
            (final_point.latitude, final_point.longitude)
        )
        if terrain is None:
            return self._fallback(ray, home_alt) if allow_fallback else None
        return Location(
            final_point.latitude,
            final_point.longitude,
            terrain + delta_alt - home_alt,
            is_absolute=home_alt == 0,
        )

    def _vertical_result(
        self,
        tile: DemData,
        lat0: float,
        lon0: float,
        alt0: float,
        down: float,
        home_alt: float,
        delta_alt: float,
        allow_fallback: bool,
    ) -> Optional[Location]:
        terrain = tile.get_height((lat0, lon0))
        if terrain is not None:
            terrain += delta_alt
            if down > 0 and alt0 >= terrain:
                return Location(
                    lat0,
                    lon0,
                    terrain,
                    is_absolute=home_alt == 0,
                )
        if not allow_fallback:
            return None
        ray_alt = alt0 - down * self._max_distance
        return Location(
            lat0,
            lon0,
            ray_alt - home_alt,
            is_absolute=home_alt == 0,
        )

    def _fallback(self, ray: TerrainRay, home_alt: float) -> Location:
        final_point = geodesic(meters=self._max_distance).destination(
            (ray.lat0, ray.lon0),
            ray.bearing_deg,
        )
        ray_alt = ray.alt0 - ray.down * (
            self._max_distance / ray.horizontal_norm
        )
        return Location(
            final_point.latitude,
            final_point.longitude,
            ray_alt - home_alt,
            is_absolute=home_alt == 0,
        )
