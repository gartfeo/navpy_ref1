"""Narrow dependency contracts used by camera-mount behavior owners."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol

import numpy as np

from navpy.modules.vision.camera_mount_types import (
    LiveZoomSample,
    ZoomWrite,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.worker_failure import FailureHealthPort


class CameraOpticsPort(Protocol):
    def get_k(self) -> np.ndarray: ...

    def get_dist(self) -> np.ndarray: ...

    @property
    def image_width(self) -> int | None: ...

    @property
    def image_height(self) -> int | None: ...

    def is_valid(self, u: float, v: float) -> bool: ...


class CameraCalibrationPort(Protocol):
    def zoom_map(self) -> Mapping[object, object] | None: ...


class CameraZoomPort(Protocol):
    def zoom_key(self) -> str | None: ...

    def apply_zoom(self, zoom: str) -> ZoomWrite: ...


class CameraLifecyclePort(FailureHealthPort, Protocol):
    def refresh(self) -> None: ...


class GimbalFramePort(Protocol):
    def state_is_static(self) -> bool: ...

    def get_data(self) -> GimbalData: ...

    def frame_state_sample(
        self,
    ) -> tuple[GimbalData, object, object, object]: ...


class GimbalZoomCommandPort(Protocol):
    def apply_zoom(self, zoom: str) -> ZoomWrite: ...


class GimbalZoomReadbackPort(Protocol):
    def zoom_level(self) -> object: ...

    def has_zoom_age(self) -> bool: ...

    def zoom_age(self) -> object: ...

    def has_zoom_fresh_flag(self) -> bool: ...

    def zoom_fresh_flag(self) -> object: ...

    def zoom_sample(self) -> object: ...

    def supports_zoom_readback(self) -> bool: ...


class GimbalLifecyclePort(FailureHealthPort, Protocol):
    def start(self) -> None: ...

    def stop(self) -> bool: ...

    def refresh(self) -> None: ...


class FrameOpticsPort(Protocol):
    def get_k_for_frame_locked(
        self,
        frame_w: int,
        frame_h: int,
    ) -> np.ndarray: ...

    def get_dist_locked(self) -> np.ndarray: ...


class LiveOpticsPort(Protocol):
    def get_k(self) -> np.ndarray: ...

    @property
    def image_width(self) -> int | None: ...

    @property
    def image_height(self) -> int | None: ...


class FrameZoomReadbackPort(Protocol):
    def camera_zoom_key(self) -> str | None: ...

    def has_zoom_readback(self) -> bool: ...

    def valid_level(self, level: object) -> float | None: ...

    def zoom_key_from_level(self, level: float) -> str | None: ...


class ControlZoomReadbackPort(Protocol):
    def current_zoom(self) -> str | None: ...

    def zoom_key_from_hardware(self) -> str | None: ...

    def camera_zoom_key(self) -> str | None: ...


class LiveZoomReadbackPort(Protocol):
    def live_sample(self) -> LiveZoomSample | None: ...

    def zoom_key_from_level(self, level: float) -> str | None: ...

    def camera_zoom_key(self) -> str | None: ...

    def has_zoom_readback(self) -> bool: ...


__all__ = [
    "CameraCalibrationPort",
    "CameraLifecyclePort",
    "CameraOpticsPort",
    "CameraZoomPort",
    "ControlZoomReadbackPort",
    "FrameOpticsPort",
    "FrameZoomReadbackPort",
    "GimbalFramePort",
    "GimbalLifecyclePort",
    "GimbalZoomCommandPort",
    "GimbalZoomReadbackPort",
    "LiveOpticsPort",
    "LiveZoomReadbackPort",
]
