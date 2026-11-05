"""Per-frame detection backend contract and the neutral detection record."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navpy.core.kernel.device import DeviceT


class FrameDetector(Protocol):
    """Stateless per-frame inference backend (ChArUco board, YOLO model)."""

    device: DeviceT

    def detect(self, frame: np.ndarray) -> list[Detection]: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class BoardCorners:
    """A reference board's found corners in one frame, as measured.

    ``ids`` index the board's interior corners; ``points_px`` are the raw
    (distorted) pixel positions in the same order.
    """

    ids: tuple[int, ...]
    points_px: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        if len(self.ids) != len(self.points_px):
            raise ValueError("board corner ids and points differ in length")


@dataclass(frozen=True)
class Detection:
    cx: float
    cy: float
    w: float
    h: float
    confidence: float
    class_id: int
    # Set only by a board detector; a model detector has no corners.
    board: BoardCorners | None = None

    @property
    def xyxy(self) -> np.ndarray:
        x1 = self.cx - self.w * 0.5
        y1 = self.cy - self.h * 0.5
        x2 = self.cx + self.w * 0.5
        y2 = self.cy + self.h * 0.5
        return np.array([x1, y1, x2, y2], dtype=np.float32)
