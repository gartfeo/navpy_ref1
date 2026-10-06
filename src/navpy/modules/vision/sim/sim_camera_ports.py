"""Typed camera operation bundles used by simulator rendering leaves."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


@dataclass(frozen=True)
class FrameSize:
    width_px: int
    height_px: int

    @classmethod
    def with_defaults(
        cls,
        width_px: int | None,
        height_px: int | None,
    ) -> "FrameSize":
        return cls(int(width_px or 1920), int(height_px or 1080))


class CameraMatrixReader(Protocol):
    def __call__(self) -> np.ndarray: ...


class GimbalDataReader(Protocol):
    def __call__(self) -> GimbalData: ...


class PixelValidator(Protocol):
    def __call__(self, x_px: float, y_px: float) -> bool: ...


class ZoomSynchronizer(Protocol):
    def __call__(self) -> bool: ...


class CameraRefresher(Protocol):
    def __call__(self) -> None: ...


@dataclass(frozen=True)
class ProjectionCameraPort:
    read_matrix: CameraMatrixReader
    read_gimbal: GimbalDataReader
    frame_size: FrameSize
    pixel_valid: PixelValidator


@dataclass(frozen=True)
class CaptureCameraPort:
    read_matrix: CameraMatrixReader
    frame_size: FrameSize


@dataclass(frozen=True)
class TrackingCameraPort:
    read_matrix: CameraMatrixReader
    read_gimbal: GimbalDataReader


__all__ = [
    "CameraMatrixReader",
    "CameraRefresher",
    "CaptureCameraPort",
    "FrameSize",
    "GimbalDataReader",
    "PixelValidator",
    "ProjectionCameraPort",
    "TrackingCameraPort",
    "ZoomSynchronizer",
]
