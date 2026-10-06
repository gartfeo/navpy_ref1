"""Exact input ports for controller-level detector UI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.gimbal_rate_tracker import GimbalTrackResult
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class AircraftAttitudeReader(Protocol):
    def __call__(self) -> Optional[Attitude]: ...


class DetectorUiStepPort(Protocol):
    def ui_step(self) -> bool: ...


class SimulatorDebugViewPort(Protocol):
    def ui_step(self) -> bool: ...

    def close(self) -> None: ...


class NedVectorCalculator(Protocol):
    def __call__(
        self,
        *,
        u: float,
        v: float,
        k: np.ndarray,
        g_data: GimbalData,
        uas_att: Attitude,
    ) -> np.ndarray | None: ...


@dataclass(frozen=True)
class SimDebugSnapshot:
    source_name: str
    gimbal: GimbalData
    image_width: int
    image_height: int
    intrinsics: np.ndarray | None
    current_zoom: str | None
    detections: tuple[DetectedObject, ...]
    rate_result: GimbalTrackResult | None
    zoom_result: ZoomTrackResult | None
    zoom_target_pixels: float | None


class SimDebugSnapshotReader(Protocol):
    def __call__(self) -> SimDebugSnapshot: ...


@dataclass(frozen=True)
class SimDebugSource:
    read_snapshot: SimDebugSnapshotReader


__all__ = [
    "AircraftAttitudeReader",
    "DetectorUiStepPort",
    "NedVectorCalculator",
    "SimDebugSource",
    "SimDebugSnapshot",
    "SimDebugSnapshotReader",
    "SimulatorDebugViewPort",
]
