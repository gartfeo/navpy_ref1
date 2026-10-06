"""OpenCV window lifecycle for the stacked simulator view."""

from __future__ import annotations

import cv2
import numpy as np


class VisionDebugWindow:
    def __init__(self, name: str) -> None:
        self._name = name
        self._initialized = False

    def show(self, image: np.ndarray) -> bool:
        if not self._initialized:
            try:
                cv2.namedWindow(self._name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self._name, image.shape[1], image.shape[0])
                self._initialized = True
            except Exception:
                return True
        try:
            cv2.imshow(self._name, image)
            return (cv2.waitKey(1) & 0xFF) != 27
        except Exception:
            return True

    def close(self) -> None:
        if not self._initialized:
            return
        try:
            cv2.destroyWindow(self._name)
        finally:
            self._initialized = False


__all__ = ["VisionDebugWindow"]
