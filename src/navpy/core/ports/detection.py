"""Per-frame detection backend contract and the neutral detection record."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navpy.modules.vision.device import DeviceT


class FrameDetector(Protocol):
    """Stateless per-frame inference backend (ChArUco board, YOLO model)."""

    device: DeviceT

    def detect(self, frame: np.ndarray) -> list[Detection]: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class Detection:
    cx: float
    cy: float
    w: float
    h: float
    confidence: float
    class_id: int

    @property
    def xyxy(self) -> np.ndarray:
        x1 = self.cx - self.w * 0.5
        y1 = self.cy - self.h * 0.5
        x2 = self.cx + self.w * 0.5
        y2 = self.cy + self.h * 0.5
        return np.array([x1, y1, x2, y2], dtype=np.float32)
