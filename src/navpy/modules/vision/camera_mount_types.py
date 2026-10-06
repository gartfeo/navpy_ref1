"""Immutable values exchanged by camera-mount collaborators."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


@dataclass(frozen=True)
class CameraMountOptics:
    zoom_command: Optional[str]
    zoom_level: Optional[str]
    sample_id: str
    fov_h_rad: float
    fov_v_rad: float
    sample_from_hardware: bool = True


@dataclass(frozen=True)
class CameraMountFrameState:
    gimbal_data: GimbalData
    k: np.ndarray
    dist: np.ndarray
    gimbal_timestamp_s: Optional[float]
    gimbal_is_static: bool
    zoom_command: Optional[str]
    zoom_sample_id: Optional[str]
    zoom_sample_age_s: Optional[float]


@dataclass(frozen=True)
class ZoomWrite:
    supported: bool
    result: object = None

    @property
    def accepted(self) -> bool:
        return self.supported and self.result is not False

    @property
    def committed(self) -> bool:
        return self.supported and self.result is True


@dataclass(frozen=True)
class LiveZoomSample:
    level: float
    sample_id: str


__all__ = [
    "CameraMountFrameState",
    "CameraMountOptics",
    "LiveZoomSample",
    "ZoomWrite",
]
