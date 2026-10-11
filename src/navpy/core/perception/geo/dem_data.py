"""SRTM digital-elevation tile loading and interpolation."""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Tuple, Union

import numpy as np


LAT_MIN, LAT_MAX = -90, 90
LON_MIN, LON_MAX = -180, 180


class DemData:
    """Load and query a single SRTM ``.hgt`` DEM tile."""

    def __init__(self, dem_data_path: str) -> None:
        data = np.fromfile(dem_data_path, dtype=">i2")
        side = int(np.sqrt(data.size))
        if side * side != data.size:
            raise ValueError("Invalid DEM file size")
        self.dem_data = data.reshape((side, side))

        name = Path(dem_data_path).stem
        lat_sign = 1 if name[0] == "N" else -1
        lon_sign = 1 if name[3] == "E" else -1
        lat0 = lat_sign * int(name[1:3])
        lon0 = lon_sign * int(name[4:7])
        self.resolution = 1.0 / (side - 1)
        self.left = lon0 - self.resolution / 2.0
        self.top = lat0 + 1 + self.resolution / 2.0

        z_len, x_len = self.dem_data.shape

        def interpolate(
            point: tuple[float, float],
        ) -> float | np.floating | None:
            z, x = point
            z0 = int(np.floor(z))
            x0 = int(np.floor(x))
            z1 = z0 + 1
            x1 = x0 + 1
            if z0 < 0 or x0 < 0 or z1 >= z_len or x1 >= x_len:
                return None
            dz = z - z0
            dx = x - x0
            v00 = self.dem_data[z0, x0]
            v01 = self.dem_data[z0, x1]
            v10 = self.dem_data[z1, x0]
            v11 = self.dem_data[z1, x1]
            return (
                (1 - dz) * (1 - dx) * v00
                + (1 - dz) * dx * v01
                + dz * (1 - dx) * v10
                + dz * dx * v11
            )

        self.dem_interpolator = interpolate

    def get_height(
        self,
        coordinates: Union[List[float], Tuple[float, float]],
    ) -> Optional[float]:
        lat, lon = coordinates
        if not (LAT_MIN <= lat <= LAT_MAX and LON_MIN <= lon <= LON_MAX):
            return None
        x_float, z_float = self.gps_to_dem_coords(lat, lon)
        z_int = int(np.floor(z_float))
        x_int = int(np.floor(x_float))
        if not (
            0 <= z_int < self.dem_data.shape[0]
            and 0 <= x_int < self.dem_data.shape[1]
        ):
            return None
        value = self.dem_interpolator((z_float, x_float))
        return float(value) if value is not None else None

    def gps_to_dem_coords(
        self,
        latitude: float,
        longitude: float,
    ) -> Tuple[float, float]:
        x_float = (longitude - self.left) / self.resolution
        z_float = (self.top - latitude) / self.resolution
        return x_float, z_float

    @staticmethod
    def get_hgt_file_name(
        coords: Union[List[float], Tuple[float, float]],
    ) -> str:
        lat = int(coords[0])
        lon = int(coords[1])
        if lat < 0:
            lat_hem = "S"
            lat = abs(lat - 1)
        else:
            lat_hem = "N"
        if lon < 0:
            lon_hem = "W"
            lon = abs(lon - 1)
        else:
            lon_hem = "E"
        return f"{lat_hem}{lat:02d}{lon_hem}{lon:03d}.hgt"
