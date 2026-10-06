"""Abstract base class for calibration detectors."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class DetectorAbc(ABC):
    """Interface for objects that can be detected and tracked by the navigator.

    Implementations must provide:
      - detect(): return center pixel (cx, cy) or None
      - detect_bounds(): return (half_w, half_h) of bounding box or None
    """

    @abstractmethod
    def detect(self, frame: np.ndarray) -> tuple[float, float] | None:
        """Detect the object and return its center (cx, cy), or None."""

    @abstractmethod
    def detect_bounds(self, frame: np.ndarray) -> tuple[float, float] | None:
        """Detect the object and return (half_w, half_h) of its bounding box.

        Returns None if detection fails.
        """
