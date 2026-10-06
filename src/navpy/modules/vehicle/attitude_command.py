"""Low-level attitude-target command creation and transmission."""
from __future__ import annotations

import math
import time
from types import SimpleNamespace
from typing import TYPE_CHECKING

from pymavlink.dialects.v20.ardupilotmega import (
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
)

from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity

if TYPE_CHECKING:
    from navpy.modules.vehicle.pose_telemetry import CommandDiagnostics


MODULE_START_MONOTONIC_S = time.monotonic()


def euler_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


class AttitudeCommand:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        diagnostics: "CommandDiagnostics",
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._diagnostics = diagnostics

    def set(
        self,
        roll: float | None,
        pitch: float | None,
        yaw: float | None = None,
        thrust: float | None = None,
    ) -> None:
        if yaw is None:
            mask = (
                ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
                | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            )
        else:
            mask = (
                ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
                | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
                | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            )
        if thrust is None:
            mask |= ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
        roll_rad = roll or 0
        pitch_rad = pitch or 0
        yaw_rad = yaw or 0
        quaternion = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        self._diagnostics.update_attitude_target(SimpleNamespace(
            roll=math.degrees(roll_rad),
            pitch=math.degrees(pitch_rad),
            yaw=math.degrees(yaw_rad) if yaw is not None else None,
            requested_yaw=math.degrees(yaw) if yaw is not None else None,
            thrust=thrust or 0.0,
            type_mask=mask,
            timestamp_s=time.time(),
        ))
        time_boot_ms = int(
            (time.monotonic() - MODULE_START_MONOTONIC_S) * 1000
        ) & 0xFFFFFFFF
        self._transport.call(
            lambda connection: connection.mav.set_attitude_target_send(
                time_boot_ms,
                self._identity.target_system,
                0,
                mask,
                quaternion,
                0.0,
                0.0,
                0.0,
                thrust or 0.0,
            )
        )
