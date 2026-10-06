"""Face detection for calibration navigator using YOLO."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from cam.calibration.detector_abc import DetectorAbc
from navpy.modules.vision.detector import YoloDetector

# Default model path: .models/yolov8n-face-lindevs.pt at project root
_DEFAULT_MODEL = (
    Path(__file__).resolve().parents[3] / ".models" / "yolov8n-face-lindevs.pt"
)


class FaceDetector(DetectorAbc):
    """Detects faces using YOLO (yolov8n-face).

    Wraps the project's YoloDetector with the face model.
    Tracks the largest (highest-confidence) face in the frame.
    """

    def __init__(
        self,
        model_path: str | Path | None = None,
        imgsz: int = 640,
        conf: float = 0.35,
        device: str = "auto",
    ):
        path = str(model_path or _DEFAULT_MODEL)
        self._yolo = YoloDetector(
            model_path=path, imgsz=imgsz, conf=conf, device=device,
        )

    def _detect_best(
        self, frame: np.ndarray,
    ) -> tuple[float, float, float, float] | None:
        """Return (cx, cy, w, h) of the highest-confidence face, or None."""
        dets = self._yolo.detect(frame)
        if not dets:
            return None
        best = max(dets, key=lambda d: d.confidence)
        return (best.cx, best.cy, best.w, best.h)

    def detect(self, frame: np.ndarray) -> tuple[float, float] | None:
        """Detect the best face and return its center (cx, cy), or None."""
        result = self._detect_best(frame)
        if result is None:
            return None
        return (result[0], result[1])

    def detect_bounds(self, frame: np.ndarray) -> tuple[float, float] | None:
        """Detect the best face and return (half_w, half_h)."""
        result = self._detect_best(frame)
        if result is None:
            return None
        return (result[2] / 2.0, result[3] / 2.0)
