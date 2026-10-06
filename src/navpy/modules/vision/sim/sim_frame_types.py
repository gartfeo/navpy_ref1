"""Immutable values carried by one simulator render transaction."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration


@dataclass(frozen=True)
class FrameContext:
    timestamp_s: float
    generation: FrameGeneration
    receipt_timestamp_s: Optional[float]
    air_speed_mps: Optional[float]
    source_discontinuity: Optional[bool]


__all__ = ["FrameContext"]
