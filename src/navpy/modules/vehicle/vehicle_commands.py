"""Vehicle command transmission with no telemetry or lifecycle ownership."""
from __future__ import annotations

from typing import Union

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_COMPONENT_ARM_DISARM

from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity

CommandValue = Union[int, float]


class VehicleCommands:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
    ) -> None:
        self._identity = identity
        self._transport = transport

    def disarm(self) -> None:
        self.send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, p1=0, p2=21196)

    def send_command_long(
        self,
        command: int,
        p1: CommandValue = 0,
        p2: CommandValue = 0,
        p3: CommandValue = 0,
        p4: CommandValue = 0,
        p5: CommandValue = 0,
        p6: CommandValue = 0,
        p7: CommandValue = 0,
    ) -> None:
        self._transport.call(
            lambda connection: connection.mav.command_long_send(
                self._identity.target_system,
                0,
                command,
                0,
                p1,
                p2,
                p3,
                p4,
                p5,
                p6,
                p7,
            )
        )

    def send_manual_control(
        self,
        x: int,
        y: int,
        z: int,
        r: int,
        buttons: int = 0,
    ) -> None:
        self._transport.call(
            lambda connection: connection.mav.manual_control_send(
                self._identity.target_system, x, y, z, r, buttons,
            )
        )
