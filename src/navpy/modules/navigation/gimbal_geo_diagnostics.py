"""Best-effort known-geo ray diagnostics outside the visual command path."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalGeoMemory,
    GimbalHardware,
    GimbalSessionFence,
)

if TYPE_CHECKING:
    from navpy.modules.vision.geo_ray_diagnostics import GeoRayDiagnostic


class GeoRayDiagnosticLogger:
    def __init__(
        self,
        hardware: GimbalHardware,
        fence: GimbalSessionFence,
        geo: GimbalGeoMemory,
        compute_ray_diagnostic: Callable[..., GeoRayDiagnostic],
        debug_enabled: Callable[[], bool],
    ) -> None:
        self._hardware = hardware
        self._fence = fence
        self._geo = geo
        self._compute = compute_ray_diagnostic
        self._debug_enabled = debug_enabled

    def log(
        self,
        target: Location,
        geo_ref: GeoRefCalc,
        uav_loc: Location,
        uav_att: Attitude,
        command_pitch: float,
        command_yaw: float,
        generation: int,
    ) -> None:
        if not self._debug_enabled():
            return
        try:
            matrix = self._hardware.mount.get_k()
            gimbal_data = self._hardware.mount.get_gimbal_data()
            diagnostic = self._compute(
                target_loc=target,
                uav_loc=uav_loc,
                uav_att=uav_att,
                k=matrix,
                g_data=gimbal_data,
                geo_ref=geo_ref,
                is_valid_pixel=self._hardware.mount.is_valid,
            )
            key = diagnostic.cache_key(command_pitch, command_yaw)
            message = diagnostic.format_log(
                self._hardware.mount.name,
                command_pitch,
                command_yaw,
                gimbal_data.att,
            )
        except Exception as exc:  # noqa: BLE001
            self._hardware.logger.debug(
                f"GimbalNavigation({self._hardware.mount.name}): "
                f"geo ray diagnostic failed: {exc}"
            )
            return

        with self._fence.lock:
            if (
                self._geo.target is not target
                or self._fence.generation != generation
                or self._geo.last_ray_log_key == key
            ):
                return
            self._geo.last_ray_log_key = key
        self._hardware.logger.debug(message)
