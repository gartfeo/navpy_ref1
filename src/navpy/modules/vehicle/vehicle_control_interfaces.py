"""Vehicle command, mission, parameter, and simulation contracts."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Union

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.flight_mode import FlightMode

ParameterValue = Union[int, float]
CommandValue = Union[int, float]


class VehicleFlightControl(ABC):
    @abstractmethod
    def goto(self, target: Location) -> None: ...

    def goto_loiter(self, target: Location, radius: float) -> None:
        self.goto(target)

    @abstractmethod
    def set_mode(
        self,
        flight_mode: FlightMode | str,
        *,
        fallback_custom_mode: int | None = None,
    ) -> bool: ...

    @property
    @abstractmethod
    def get_mode(self) -> FlightMode | None: ...

    @abstractmethod
    def set_attitude(
        self,
        roll: float | None,
        pitch: float | None,
        yaw: float | None = None,
        thr: float | None = None,
    ) -> None: ...

    @property
    @abstractmethod
    def is_armed(self) -> bool: ...

    @abstractmethod
    def disarm(self) -> None: ...

    @abstractmethod
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
    ) -> None: ...


class VehicleMissionAccess(ABC):
    @property
    @abstractmethod
    def mission_items_count(self) -> int: ...

    @property
    @abstractmethod
    def mission_items_next(self) -> int | None: ...

    @abstractmethod
    def get_mission_item(self, sequence: int) -> MAVLink_message | None: ...

    @abstractmethod
    def get_mission_item_location(self, command_index: int) -> Location | None: ...

    @abstractmethod
    def update_mission_item(
        self,
        sequence: int,
        command: MAVLink_message,
    ) -> None: ...

    @abstractmethod
    def update_mission_item_location(
        self,
        sequence: int,
        command_location: Location,
        alt: float,
    ) -> None: ...

    @abstractmethod
    def download_mission(self) -> int: ...

    @abstractmethod
    def upload_mission(self) -> bool: ...

    @abstractmethod
    def clear_mission(self) -> None: ...

    @abstractmethod
    def restart_mission(self, mission_index: int = 0) -> None: ...

    @abstractmethod
    def set_current(self, index: int) -> None: ...


class VehicleParameters(ABC):
    @abstractmethod
    def get_parameter(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None: ...

    @abstractmethod
    def get_parameter_fresh(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None: ...

    @abstractmethod
    def set_parameter(
        self,
        name: str,
        value: ParameterValue,
        *,
        mav_param_type: int | None = None,
        timeout: float = 2.0,
    ) -> bool: ...

    @abstractmethod
    def send_parameter_unverified(
        self,
        name: str,
        value: ParameterValue,
    ) -> bool: ...

    def get_param_or_default(self, name: str, default: float) -> float:
        value = self.get_parameter(name)
        return value if value is not None else default


class VehicleSimulationAccess(ABC):
    def sim_speedup(self) -> float:
        """Return scheduler cadence multiplier, never a source-time scale."""
        return 1.0

    def is_simulated_autopilot(self) -> bool:
        return False


__all__ = [
    "CommandValue",
    "ParameterValue",
    "VehicleFlightControl",
    "VehicleMissionAccess",
    "VehicleParameters",
    "VehicleSimulationAccess",
]
