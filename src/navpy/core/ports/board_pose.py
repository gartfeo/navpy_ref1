"""Metric board pose from one frame's board corners (HITL step 22).

A board pose places the reference board's centre in the camera frame, in
metres. It is acquisition evidence only (the orbit centre before NAV): it
never enters the guidance command record, `GuidanceVisionFrame`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Union

import numpy as np

from navpy.core.ports.detection import BoardCorners


@dataclass(frozen=True)
class BoardPoseFit:
    """An accepted fit: the board centre in the OpenCV camera frame.

    Camera frame: x right, y down, z forward along the optical axis.
    """

    centre_camera_m: tuple[float, float, float]
    corner_count: int
    reprojection_rms_px: float
    reprojection_p95_px: float
    centre_sigma_m: float
    ambiguity_ratio: float


@dataclass(frozen=True)
class BoardPoseRejection:
    """A refused fit and why; no metric value is kept."""

    reason: str
    corner_count: int = 0


BoardPoseResult = Union[BoardPoseFit, BoardPoseRejection]


class BoardPoseSolver(Protocol):
    """Fit the known board to one frame's corners with that frame's optics."""

    def solve(
        self,
        corners: BoardCorners,
        intrinsic: np.ndarray,
        distortion: np.ndarray,
    ) -> BoardPoseResult: ...


__all__ = [
    "BoardPoseFit",
    "BoardPoseRejection",
    "BoardPoseResult",
    "BoardPoseSolver",
]
