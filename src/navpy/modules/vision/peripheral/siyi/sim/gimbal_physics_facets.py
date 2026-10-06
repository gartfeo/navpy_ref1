"""Stateless readback and control facets for the SIYI physics adapter."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import (
    GimbalAngularPlant,
    GimbalAngularSnapshot,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_zoom_plant import (
    GimbalZoomPlant,
    GimbalZoomSnapshot,
)


@dataclass(frozen=True)
class GimbalPhysicsSnapshot:
    angular: GimbalAngularSnapshot
    zoom: GimbalZoomSnapshot


@dataclass(frozen=True)
class GimbalPhysicsParts:
    angular: GimbalAngularPlant
    zoom: GimbalZoomPlant


class GimbalPhysicsReadFacet:
    _parts: GimbalPhysicsParts

    @property
    def yaw(self) -> float:
        return self._parts.angular.snapshot().yaw

    @property
    def pitch(self) -> float:
        return self._parts.angular.snapshot().pitch

    @property
    def roll(self) -> float:
        return self._parts.angular.snapshot().roll

    @property
    def yaw_speed(self) -> float:
        return self._parts.angular.snapshot().yaw_speed

    @property
    def pitch_speed(self) -> float:
        return self._parts.angular.snapshot().pitch_speed

    @property
    def roll_speed(self) -> float:
        return self._parts.angular.snapshot().roll_speed

    @property
    def zoom_level(self) -> float:
        return self._parts.zoom.level

    @property
    def motion_mode(self) -> int:
        return self._parts.angular.snapshot().motion_mode

    def snapshot(self) -> GimbalPhysicsSnapshot:
        return GimbalPhysicsSnapshot(
            self._parts.angular.snapshot(),
            self._parts.zoom.snapshot(),
        )


class GimbalPhysicsControlFacet:
    _parts: GimbalPhysicsParts

    def set_speed(self, yaw_cmd: float, pitch_cmd: float) -> None:
        self._parts.angular.set_speed(yaw_cmd, pitch_cmd)

    def set_target_angles(self, yaw_deg: float, pitch_deg: float) -> None:
        self._parts.angular.set_target_angles(yaw_deg, pitch_deg)

    def center(self) -> None:
        self._parts.angular.set_target_angles(0.0, 0.0)

    def set_target_zoom(self, level: float) -> None:
        self._parts.zoom.set_target(level)

    def start_zoom_in(self) -> None:
        self._parts.zoom.start(1)

    def start_zoom_out(self) -> None:
        self._parts.zoom.start(-1)

    def stop_zoom(self) -> None:
        self._parts.zoom.stop()

    def set_motion_mode(self, mode: int) -> None:
        self._parts.angular.set_motion_mode(mode)

    def set_vehicle_attitude(self, attitude: Attitude) -> None:
        self._parts.angular.set_vehicle_attitude(attitude)

    def update(self, dt: float) -> None:
        self._parts.angular.advance(dt)
        self._parts.zoom.advance(dt)


__all__ = [
    "GimbalPhysicsControlFacet",
    "GimbalPhysicsParts",
    "GimbalPhysicsReadFacet",
    "GimbalPhysicsSnapshot",
]
