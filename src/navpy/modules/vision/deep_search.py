"""Low-rate deep detector support for selected-POI reacquisition."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from navpy.modules.vision.multi_object_tracker import iou_xyxy
from navpy.modules.vision.yolo_detector import Detection, DeviceT, YoloDetector


@dataclass(frozen=True)
class DeepSearchConfig:
    hz: float = 2.0
    imgsz: int = 960
    conf: float = 0.12
    stale_seconds: float = 1.0
    model_path: Optional[str] = None
    device: DeviceT = "auto"
    classes: Optional[List[int]] = None

    @property
    def period(self) -> float:
        return 1.0 / max(1e-6, float(self.hz))


class DeepSearchDetector:
    """Owns a high-resolution YOLO instance for low-Hz reacquisition."""

    def __init__(
            self,
            config: DeepSearchConfig,
            *,
            default_model_path: str,
            default_device: DeviceT,
            default_classes: Optional[List[int]],
            logger=None,
    ):
        self.config = config
        model_path = config.model_path or default_model_path
        classes = default_classes if config.classes is None else config.classes
        device = default_device if config.device == "auto" else config.device
        self._detector = YoloDetector(
            model_path,
            imgsz=config.imgsz,
            conf=config.conf,
            device=device,
            classes=classes,
            logger=logger,
        )

    def detect(self, frame) -> List[Detection]:
        return self._detector.detect(frame)

    def close(self) -> None:
        """Release the high-resolution YOLO model and its GPU memory."""
        if self._detector is not None:
            self._detector.close()
        self._detector = None


def deep_search_config_from_settings(settings: Optional[dict]) -> Optional[DeepSearchConfig]:
    if not isinstance(settings, dict) or not settings.get("enabled", False):
        return None
    return DeepSearchConfig(
        hz=float(settings.get("hz", 2.0)),
        imgsz=int(settings.get("imgsz", 960)),
        conf=float(settings.get("conf", 0.12)),
        stale_seconds=max(0.0, float(settings.get("stale_seconds", 1.0))),
        model_path=settings.get("model_path"),
        device=settings.get("device", "auto"),
        classes=settings.get("classes"),
    )


def merge_deep_search_detections(
        base_detections: List[Detection],
        deep_detections: List[Detection],
        *,
        duplicate_iou: float = 0.50,
) -> List[Detection]:
    """Merge high-resolution detections into the fast detector batch.

    For the same class and overlapping object, the *higher-confidence* box wins
    so a low-confidence deep box does not clobber a confident fast-detector box
    (and vice-versa). Non-overlapping deep boxes are appended as new candidates.
    """
    if not deep_detections:
        return list(base_detections)
    merged = list(base_detections)
    for deep in deep_detections:
        duplicate = False
        for index, base in enumerate(merged):
            if base.class_id != deep.class_id:
                continue
            if iou_xyxy(base.xyxy, deep.xyxy) >= duplicate_iou:
                if deep.confidence > base.confidence:
                    merged[index] = deep
                duplicate = True
                break
        if not duplicate:
            merged.append(deep)
    return merged
