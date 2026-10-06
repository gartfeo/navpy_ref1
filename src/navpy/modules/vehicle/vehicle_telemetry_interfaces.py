"""Read-only vehicle identity and telemetry capability contracts."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.common.models.wind import Wind


class VehicleIdentityAccess(ABC):
    @property
    @abstractmethod
    def target_system(self) -> int: ...

    @property
    @abstractmethod
    def source_system(self) -> int: ...

    @property
    @abstractmethod
    def transport_source_system(self) -> int | None: ...

    @property
    @abstractmethod
    def transport_source_component(self) -> int | None: ...


class VehicleMotionTelemetry(ABC):
    @property
    @abstractmethod
    def velocity(self) -> tuple[float, float, float] | None: ...

    @property
    @abstractmethod
    def wind(self) -> Wind | None: ...

    @property
    @abstractmethod
    def ground_speed(self) -> float | None: ...

    @property
    @abstractmethod
    def ground_speed_ned(self) -> np.ndarray | None: ...

    @property
    @abstractmethod
    def air_speed(self) -> float | None: ...

    @property
    @abstractmethod
    def attitude(self) -> Attitude | None: ...

    @property
    @abstractmethod
    def heading(self) -> float | None: ...

    @property
    @abstractmethod
    def home_location(self) -> Location | None: ...

    @abstractmethod
    def location(self, is_relative: bool) -> Location | None: ...


class VehiclePowerTelemetry(ABC):
    @property
    @abstractmethod
    def battery_level(self) -> int | None: ...

    @property
    @abstractmethod
    def battery_voltage(self) -> float | None: ...

    @property
    @abstractmethod
    def battery_current(self) -> float | None: ...


class VehicleLimits(ABC):
    @property
    @abstractmethod
    def max_pitch(self) -> float: ...

    @property
    @abstractmethod
    def min_pitch(self) -> float: ...

    @property
    @abstractmethod
    def lim_roll(self) -> float: ...


__all__ = [
    "VehicleIdentityAccess",
    "VehicleLimits",
    "VehicleMotionTelemetry",
    "VehiclePowerTelemetry",
]
