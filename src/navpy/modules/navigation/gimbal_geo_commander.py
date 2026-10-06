"""Generation-fenced known-geolocation gimbal commands."""

from __future__ import annotations

import math

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.gimbal_geo_diagnostics import GeoRayDiagnosticLogger
from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalGeoMemory,
    GimbalHardware,
    GimbalSessionCommandGate,
    GimbalSessionFence,
)
from navpy.modules.navigation.gimbal_tracking_constants import GIMBAL_COMMAND_ERRORS


class GimbalGeoCommander:
    """Convert one aircraft pose into an atomic geo-pointing command."""

    def __init__(
        self,
        hardware: GimbalHardware,
        gate: GimbalSessionCommandGate,
        fence: GimbalSessionFence,
        geo: GimbalGeoMemory,
        diagnostics: GeoRayDiagnosticLogger,
    ) -> None:
        self._hardware = hardware
        self._gate = gate
        self._fence = fence
        self._geo = geo
        self._diagnostics = diagnostics

    def update(
        self,
        uav_loc: Location | None,
        uav_att: Attitude | None,
    ) -> None:
        if uav_loc is None or uav_att is None:
            return
        with self._fence.lock:
            target = self._geo.target
            geo_ref = self._geo.geo_ref
            generation = self._fence.generation
            if target is None or geo_ref is None:
                return
        try:
            command = geo_ref.calc_gimbal_lock_att_loc(
                uav_loc,
                target,
                uav_att,
                self._hardware.mount.get_gimbal_data(),
            )
        except GIMBAL_COMMAND_ERRORS as exc:
            self._warn(f"update_geo pose math failed: {exc}")
            return
        command_pitch = float(command.pitch)
        command_yaw = float(command.yaw)
        if not math.isfinite(command_pitch) or not math.isfinite(command_yaw):
            self._warn(
                "update_geo rejected non-finite command "
                f"(pitch={command_pitch}, yaw={command_yaw})"
            )
            return
        self._diagnostics.log(
            target,
            geo_ref,
            uav_loc,
            uav_att,
            command_pitch,
            command_yaw,
            generation,
        )
        with self._gate.lock:
            with self._fence.lock:
                if (
                    self._geo.target is not target
                    or self._fence.generation != generation
                ):
                    return
            try:
                self._hardware.gimbal.set_att(
                    Attitude(command_pitch, command_yaw, 0)
                )
            except GIMBAL_COMMAND_ERRORS as exc:
                self._warn(f"update_geo set_att failed: {exc}")

    def _warn(self, message: str) -> None:
        self._hardware.logger.warning(
            f"GimbalNavigation({self._hardware.mount.name}): {message}"
        )


__all__ = ["GimbalGeoCommander"]
