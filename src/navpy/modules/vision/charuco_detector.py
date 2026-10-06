"""ChArUco board detector: the default real per-frame inference backend.

Satisfies the same per-frame contract as ``YoloDetector`` (``device``,
``detect(frame) -> list[Detection]``, ``close()``), so the real detector's
tracking, mapping, and publication pipeline is unchanged.

Per frame it reports at most one detection: the board, as the single dock
class. Confidence is the fraction of the board's interior ChArUco corners
that were found. The bounding box is the axis-aligned box of the *whole*
board outline, projected through the corner homography, so its extent stays
tied to the board's physical size under partial occlusion. When no reliable
homography exists it falls back to the extent of the detected corners and
marker corners. The box is clipped to the frame, as a model detector's is.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import List, Optional

import cv2
import numpy as np

from navpy.modules.vision.charuco_board import (
    CHARUCO_DOCK_CLASS_ID,
    CharucoBoardSpec,
)
from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.device import DeviceT
from navpy.modules.vision.yolo_detector import Detection

# A plane-to-image homography needs at least four point correspondences.
MIN_CHARUCO_CORNERS = 4


def _dictionary(name: str) -> "cv2.aruco.Dictionary":
    dictionary_id = getattr(cv2.aruco, name, None)
    if not isinstance(dictionary_id, int):
        raise ValueError(f"Unknown ArUco dictionary {name!r}")
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def build_charuco_board(spec: CharucoBoardSpec) -> "cv2.aruco.CharucoBoard":
    """Create the OpenCV board in millimetre units."""
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y),
        float(spec.square_mm),
        float(spec.marker_mm),
        _dictionary(spec.dictionary),
    )


class CharucoBoardDetector:
    """Detect one ChArUco reference board per frame."""

    def __init__(
            self,
            spec: CharucoBoardSpec,
            *,
            min_confidence: float = 0.0,
            logger: ILogger | None = None,
    ) -> None:
        self.spec = spec
        self.min_confidence = float(min_confidence)
        self.device: DeviceT = "cpu"
        board = build_charuco_board(spec)
        self._board_points = np.asarray(
            board.getChessboardCorners(), dtype=np.float32,
        )[:, :2]
        width_mm, height_mm = spec.size_mm
        self._outline = np.array(
            [[0.0, 0.0], [width_mm, 0.0], [width_mm, height_mm], [0.0, height_mm]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        self._detector: Optional[cv2.aruco.CharucoDetector] = (
            cv2.aruco.CharucoDetector(board)
        )
        if logger is not None:
            logger.info(
                f"ChArUco detector loaded: dictionary={spec.dictionary} "
                f"squares={spec.squares_x}x{spec.squares_y} "
                f"square={spec.square_mm:g}mm marker={spec.marker_mm:g}mm "
                f"min_conf={self.min_confidence:.2f} "
                f"class_id={CHARUCO_DOCK_CLASS_ID}"
            )

    def close(self) -> None:
        """Release the OpenCV detector. Safe to call repeatedly."""
        self._detector = None

    def detect(self, frame: np.ndarray) -> List[Detection]:
        if self._detector is None:
            raise RuntimeError("ChArUco detector is closed")
        gray = _to_gray(frame)
        corners, ids, marker_corners, _ = self._detector.detectBoard(gray)
        corner_count = 0 if ids is None else int(len(ids))
        if corner_count < MIN_CHARUCO_CORNERS:
            return []
        confidence = corner_count / float(self.spec.interior_corner_count)
        if confidence < self.min_confidence:
            return []
        image_points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        board_points = self._board_points[np.asarray(ids).reshape(-1)]
        box = self._outline_box(board_points, image_points)
        if box is None:
            box = _points_box(image_points, marker_corners)
        box = _clip_box(box, gray.shape[1], gray.shape[0])
        if box is None:
            return []
        x1, y1, x2, y2 = box
        return [
            Detection(
                cx=0.5 * (x1 + x2),
                cy=0.5 * (y1 + y2),
                w=x2 - x1,
                h=y2 - y1,
                confidence=min(1.0, confidence),
                class_id=CHARUCO_DOCK_CLASS_ID,
            )
        ]

    def _outline_box(
            self,
            board_points: np.ndarray,
            image_points: np.ndarray,
    ) -> Optional[tuple[float, float, float, float]]:
        homography, _ = cv2.findHomography(board_points, image_points, 0)
        if homography is None or not np.all(np.isfinite(homography)):
            return None
        outline = cv2.perspectiveTransform(self._outline, homography)
        outline = outline.reshape(-1, 2)
        if not np.all(np.isfinite(outline)):
            return None
        return _bounds(outline)


def _to_gray(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return frame
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def _points_box(
        image_points: np.ndarray,
        marker_corners: Sequence[np.ndarray] | None,
) -> tuple[float, float, float, float]:
    parts = [image_points]
    parts.extend(
        np.asarray(marker, dtype=np.float32).reshape(-1, 2)
        for marker in (marker_corners or ())
    )
    return _bounds(np.concatenate(parts, axis=0))


def _bounds(points: np.ndarray) -> tuple[float, float, float, float]:
    x1, y1 = points.min(axis=0)
    x2, y2 = points.max(axis=0)
    return float(x1), float(y1), float(x2), float(y2)


def _clip_box(
        box: tuple[float, float, float, float],
        width: int,
        height: int,
) -> Optional[tuple[float, float, float, float]]:
    x1, y1, x2, y2 = box
    x1, x2 = max(0.0, x1), min(float(width), x2)
    y1, y2 = max(0.0, y1), min(float(height), y2)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


__all__ = [
    "CharucoBoardDetector",
    "MIN_CHARUCO_CORNERS",
    "build_charuco_board",
]
