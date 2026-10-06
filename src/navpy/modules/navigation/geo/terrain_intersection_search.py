"""Coarse-to-fine search for a ray/terrain intersection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from geopy.distance import geodesic

from navpy.modules.navigation.geo.dem_data import DemData


@dataclass(frozen=True)
class TerrainRay:
    tile: DemData
    lat0: float
    lon0: float
    alt0: float
    down: float
    delta_alt: float
    bearing_deg: float
    horizontal_norm: float


class TerrainIntersectionSearch:
    """Find horizontal distance where a fixed DEM tile meets a ray."""

    def __init__(self, max_distance: float, coarse_samples: int) -> None:
        self._max_distance = max_distance
        self._coarse_samples = coarse_samples

    def find(self, ray: TerrainRay) -> Optional[float]:
        bracket = self._coarse_bracket(ray)
        return None if bracket is None else self._binary_search(ray, bracket)

    def _coarse_bracket(
        self,
        ray: TerrainRay,
    ) -> Optional[tuple[float, float]]:
        distances = np.linspace(
            0,
            self._max_distance,
            self._coarse_samples + 1,
        )
        previous_diff = None
        previous_distance = 0.0
        for distance in distances:
            current_diff = self._alt_diff(ray, distance)
            if previous_diff is not None:
                if previous_diff > 0 and current_diff <= 0:
                    return previous_distance, distance
                if previous_diff < 0 and current_diff >= 0:
                    return previous_distance, distance
            previous_diff = current_diff
            previous_distance = distance
        return None

    def _binary_search(
        self,
        ray: TerrainRay,
        bracket: tuple[float, float],
    ) -> Optional[float]:
        left, right = bracket
        left_diff = self._alt_diff(ray, left)
        right_diff = self._alt_diff(ray, right)
        if left_diff * right_diff > 0:
            return None

        while right - left > 0.1:
            midpoint = 0.5 * (left + right)
            midpoint_diff = self._alt_diff(ray, midpoint)
            if left_diff > 0 and midpoint_diff <= 0:
                right = midpoint
            elif left_diff < 0 and midpoint_diff >= 0:
                right = midpoint
            else:
                left = midpoint
                left_diff = midpoint_diff
        return 0.5 * (left + right)

    @staticmethod
    def _alt_diff(ray: TerrainRay, horizontal_distance: float) -> float:
        point = geodesic(meters=horizontal_distance).destination(
            (ray.lat0, ray.lon0),
            ray.bearing_deg,
        )
        terrain = ray.tile.get_height((point.latitude, point.longitude))
        if terrain is None:
            return 9999.0
        terrain += ray.delta_alt
        ray_alt = ray.alt0 - ray.down * (
            horizontal_distance / ray.horizontal_norm
        )
        return ray_alt - terrain
