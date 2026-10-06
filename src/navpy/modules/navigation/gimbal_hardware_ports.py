"""Narrow hardware protocols consumed by gimbal navigation."""

from __future__ import annotations

from typing import Protocol

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class GimbalActuatorPort(Protocol):
    def set_motion_mode(self, mode: int) -> None: ...

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None: ...

    def set_att(self, attitude: Attitude) -> None: ...


class GimbalMountPort(Protocol):
    name: str

    def get_gimbal_data(self) -> GimbalData: ...

    def get_k(self) -> np.ndarray: ...

    def is_valid(self, u: float, v: float) -> bool: ...

    def set_zoom(self, zoom: str) -> bool: ...


__all__ = ["GimbalActuatorPort", "GimbalMountPort"]
