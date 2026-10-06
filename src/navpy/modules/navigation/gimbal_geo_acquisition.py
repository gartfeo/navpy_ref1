"""One-shot known-geo acquisition zoom transaction."""

from __future__ import annotations

import math
from typing import Callable

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.gimbal_geo_zoom_selector import GeoZoomSelector
from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalGeoMemory,
    GimbalHardware,
    GimbalSessionCommandGate,
    GimbalSessionFence,
    GimbalTrackers,
)
from navpy.modules.navigation.gimbal_tracking_constants import GIMBAL_COMMAND_ERRORS



class GeoAcquisitionZoom:
    """Compute and atomically commit a one-shot geo acquisition zoom."""

    def __init__(
        self,
        hardware: GimbalHardware,
        trackers: GimbalTrackers,
        gate: GimbalSessionCommandGate,
        fence: GimbalSessionFence,
        geo: GimbalGeoMemory,
        geodetic_to_ned: Callable[
            [float, float, float, float, float, float],
            tuple[float, float, float],
        ],
        selector: GeoZoomSelector,
    ) -> None:
        self._hardware = hardware
        self._trackers = trackers
        self._gate = gate
        self._fence = fence
        self._geo = geo
        self._geodetic_to_ned = geodetic_to_ned
        self._selector = selector

    def prepare(
        self,
        uav_loc: Location | None,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        if uav_loc is None or uav_att is None or self._trackers.zoom is None:
            return False
        min_pixels = float(min_pixels)
        if not math.isfinite(min_pixels) or min_pixels <= 0.0:
            return False
        with self._fence.lock:
            target = self._geo.target
            geo_ref = self._geo.geo_ref
            generation = self._fence.generation
            if target is None or geo_ref is None:
                return False
        try:
            target_ned = self._geodetic_to_ned(
                target.lat,
                target.lng,
                target.alt,
                uav_loc.lat,
                uav_loc.lng,
                uav_loc.alt,
            )
            slant_m = float(np.linalg.norm(target_ned))
            if not math.isfinite(slant_m) or slant_m <= 0.0:
                return False
            command_zoom, projected_px = self._selector.select(
                target_ned,
                self._hardware.mount.get_gimbal_data(),
                uav_att,
                geo_ref,
                class_id,
                min_pixels,
                slant_m,
            )
            if command_zoom is None:
                return False
            with self._gate.lock:
                with self._fence.lock:
                    if (
                        self._geo.target is not target
                        or self._fence.generation != generation
                        or self._geo.zoom_key == command_zoom
                    ):
                        return False
                if not self._hardware.mount.set_zoom(command_zoom):
                    return False
                with self._fence.lock:
                    if (
                        self._geo.target is not target
                        or self._fence.generation != generation
                    ):
                        return False
                    self._geo.zoom_key = command_zoom
        except GIMBAL_COMMAND_ERRORS as exc:
            self._hardware.logger.warning(
                f"GimbalNavigation({self._hardware.mount.name}): "
                f"geo acquisition zoom failed: {exc}"
            )
            return False
        self._hardware.logger.info(
            f"GimbalNavigation({self._hardware.mount.name}): geo acquisition "
            f"zoom set={command_zoom} slant={slant_m:.0f}m "
            f"proj={projected_px:.1f}px min={min_pixels:.1f}px"
        )
        return True


__all__ = ["GeoAcquisitionZoom"]
