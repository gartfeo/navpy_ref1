"""YOLO inference engine.

Wraps ultralytics YOLO to convert camera frames into bounding box
detections. Stateless per-frame: no tracking or temporal logic.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from navpy.modules.vision.device import (
    DeviceT,
    cuda_available,
    free_cuda_cache,
    resolve_auto_device,
    uses_cuda_device,
)

# Substrings identifying an FP16/half-precision inference failure (as opposed to
# an unrelated error such as CUDA OOM, a malformed frame, or a driver fault).
_HALF_ERROR_MARKERS = ("half", "fp16", "float16", "dtype")


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


class YoloDetector:
    def __init__(
            self,
            model_path: str,
            imgsz: int = 640,
            conf: float = 0.35,
            device: DeviceT = "auto",
            classes: Optional[List[int]] = None,
            logger=None,
    ):
        from ultralytics import YOLO
        requested_device = device
        self.model = YOLO(str(model_path))
        self.imgsz = imgsz
        self.conf = conf
        self.classes = classes
        self._logger = logger

        self.device = resolve_auto_device(device)

        if self._uses_cuda_device() and not cuda_available():
            if self._logger:
                self._logger.error(
                    "YOLO device requested CUDA but torch.cuda.is_available() is False. Forcing device='cpu'.")
            self.device = "cpu"
        self.half = self._uses_cuda_device()
        self._log_configuration(model_path, requested_device)

    def close(self) -> None:
        """Release the YOLO model and free CUDA memory. Safe to call repeatedly."""
        self.model = None
        free_cuda_cache(self.device)

    def detect(self, frame) -> List[Detection]:
        try:
            r = self._predict(frame)
        except Exception as exc:
            # Only treat a genuine FP16/dtype failure as a precision problem.
            # Re-raise anything else (CUDA OOM, bad frame, driver fault) so the
            # real cause surfaces instead of being hidden by a silent FP32 retry.
            if not self.half or not _is_half_precision_error(exc):
                raise
            if self._logger:
                self._logger.warning(
                    f"YOLO FP16 inference failed ({exc}); falling back to FP32 for "
                    f"the rest of this detector's lifetime")
            self.half = False
            r = self._predict(frame)

        dets: List[Detection] = []
        if r.boxes is None:
            return dets

        xyxy = _as_numpy(r.boxes.xyxy)
        confidences = _as_numpy(r.boxes.conf)
        classes = _as_numpy(r.boxes.cls)
        if xyxy.size == 0:
            return dets

        for box, confidence, class_id in zip(xyxy, confidences, classes):
            x1, y1, x2, y2 = box
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            w = float(x2 - x1)
            h = float(y2 - y1)

            dets.append(
                Detection(
                    cx=float(cx),
                    cy=float(cy),
                    w=w,
                    h=h,
                    confidence=float(confidence),
                    class_id=int(class_id),
                )
            )
        return dets

    def _predict(self, frame):
        return self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf,
            device=self.device,
            classes=self.classes,
            half=self.half,
            verbose=False,
        )[0]

    def _uses_cuda_device(self) -> bool:
        return uses_cuda_device(self.device)

    def _log_configuration(self, model_path: str, requested_device: DeviceT) -> None:
        if self._logger is None:
            return
        self._logger.info(
            f"YOLO detector loaded: model={model_path} imgsz={self.imgsz} "
            f"conf={self.conf:.2f} device={self.device} half={self.half} "
            f"classes={self.classes}"
        )
        if requested_device == "auto" and self.device == "cpu":
            self._logger.warning(
                "YOLO device auto selected CPU; install a CUDA/TensorRT-enabled "
                "runtime for real-time detector and deep-search performance"
            )


def _is_half_precision_error(exc: BaseException) -> bool:
    """True if ``exc`` looks like an FP16/half-precision failure (vs. OOM/other)."""
    message = str(exc).lower()
    return any(marker in message for marker in _HALF_ERROR_MARKERS)


def _as_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)
