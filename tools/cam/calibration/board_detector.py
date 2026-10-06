"""Checkerboard detection wrapper for calibration workflows."""

from __future__ import annotations

import cv2
import numpy as np


class BoardDetector:
    """Checkerboard detector with sub-pixel refinement.

    Tries both (cols, rows) and (rows, cols) orientations to handle
    rotated boards.
    """

    _FLAGS = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_FAST_CHECK
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    def __init__(self, cols: int = 7, rows: int = 5):
        self._pattern = (cols, rows)
        self._pattern_alt = (rows, cols)
        self._criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
        )

    def detect(self, frame: np.ndarray) -> tuple[float, float] | None:
        """Detect checkerboard and return its center (cx, cy), or None."""
        corners = self.detect_corners(frame)
        if corners is None:
            return None
        cx = float(corners[:, 0, 0].mean())
        cy = float(corners[:, 0, 1].mean())
        return (cx, cy)

    def detect_corners(self, frame: np.ndarray) -> np.ndarray | None:
        """Detect checkerboard and return the full corner array, or None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        for pattern in (self._pattern, self._pattern_alt):
            ret, corners = cv2.findChessboardCorners(gray, pattern, self._FLAGS)
            if ret:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self._criteria
                )
                return corners

        return None
