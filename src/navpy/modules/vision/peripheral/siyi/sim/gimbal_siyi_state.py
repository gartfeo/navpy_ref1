"""Shared synchronized state for the SIYI simulator capabilities."""

from __future__ import annotations

import threading
from dataclasses import dataclass

from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import (
    GimbalAngularPlant,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_zoom_plant import (
    GimbalZoomPlant,
)


@dataclass
class SiyiSimState:
    data: GimbalData
    angular: GimbalAngularPlant
    zoom: GimbalZoomPlant
    lock: threading.Lock
    zoom_sample_seq: int = 0
    zoom_updated_monotonic_s: float | None = None
    started: bool = False


__all__ = ["SiyiSimState"]
