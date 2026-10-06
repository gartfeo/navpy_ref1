"""Small mutable state owners used by simulator detection."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from navpy.modules.vision.sim.sim_runtime_ports import (
    BestFrame,
    ConfirmationFrameRenderer,
)


@dataclass
class SimCaptureState:
    renderer: Optional[ConfirmationFrameRenderer] = None
    enabled: bool = True
    best_frames: dict[int, BestFrame] = field(default_factory=dict)

    def reset(self) -> None:
        self.best_frames.clear()
        self.enabled = True


@dataclass
class ForcedGapState:
    anchor_timestamp_s: Optional[float] = None

    def set_anchor_timestamp_s(self, value: float) -> None:
        self.anchor_timestamp_s = float(value)

    def reset(self) -> None:
        self.anchor_timestamp_s = None


__all__ = ["ForcedGapState", "SimCaptureState"]
