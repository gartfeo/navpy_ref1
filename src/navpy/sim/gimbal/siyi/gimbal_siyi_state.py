"""Shared synchronized state for the SIYI simulator capabilities."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.sim.gimbal.siyi.gimbal_angles import (
    GimbalAngularPlant,
)
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import (
    GimbalZoomPlant,
)


@dataclass
class SiyiSimState:
    data: GimbalData
    angular: GimbalAngularPlant
    zoom: GimbalZoomPlant
    lock: rt.Lock
    zoom_sample_seq: int = 0
    zoom_updated_monotonic_s: float | None = None
    started: bool = False


__all__ = ["SiyiSimState"]
