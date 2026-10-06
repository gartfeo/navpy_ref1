"""DEM tile discovery, download, loading, and caching."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Callable, Optional, Sequence

from navpy.modules.navigation.geo.dem_data import DemData
from navpy.utils.srtm_downloader import SRTMDownloader


def valid_coords(lat: float, lon: float) -> bool:
    return (
        -90 <= lat <= 90
        and -180 <= lon <= 180
        and (lat != 0 or lon != 0)
    )


def default_dem_directory() -> Path:
    project_root = Path(__file__).resolve().parents[5]
    return (project_root / ".terrain" / "dem").resolve()


def download_dem_tile(filename: str, directory: Path) -> None:
    downloader = SRTMDownloader("gart.feo", "#g&Ab86j_43X&Y#")
    print("Downloading DEM file...")
    downloader.download_by_coordinates(filename, directory)
    print("Download complete!")


class DemTileRepository:
    """Resolve and cache the one-degree DEM tile for a coordinate."""

    def __init__(
        self,
        directory: Path | str,
        downloader: Callable[[str, Path], None] = download_dem_tile,
        tile_loader: Callable[[str], DemData] = DemData,
    ) -> None:
        self._directory = Path(directory)
        self._downloader = downloader
        self._tile_loader = tile_loader
        self._tiles: dict[str, DemData] = {}

    def get_elevation(self, coords: Sequence[float]) -> Optional[float]:
        tile = self.get_tile(coords)
        return None if tile is None else tile.get_height(coords)

    def get_tile(self, coords: Sequence[float]) -> Optional[DemData]:
        lat, lon = coords
        if not valid_coords(lat, lon):
            return None
        filename = DemData.get_hgt_file_name(coords)
        cached = self._tiles.get(filename)
        if cached is not None:
            return cached
        return self._load_or_download(filename)

    def _load_or_download(self, filename: str) -> Optional[DemData]:
        full_path = self._directory / filename
        if not os.path.exists(full_path):
            print(f"{full_path} does not exist.")
            try:
                self._downloader(filename, self._directory)
            except Exception as exc:
                print(f"Download failed: {exc}")
                return None
        if not os.path.exists(full_path):
            print(f"Missing DEM file after download attempt: {full_path}")
            return None
        try:
            tile = self._tile_loader(str(full_path))
        except Exception as exc:
            print(f"Error loading {full_path}: {exc}")
            return None
        self._tiles[filename] = tile
        return tile
