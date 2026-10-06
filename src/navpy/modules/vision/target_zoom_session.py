"""Recognition demand and per-target zoom session state."""

from __future__ import annotations

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_zoom_types import (
    TargetZoomTrackerConfig,
    ZoomTrackResult,
)
from navpy.modules.vision.vision_class_profile import MIN_TRACK_PIXELS
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


class TargetZoomSession:
    def __init__(self, config: TargetZoomTrackerConfig) -> None:
        self._config = config
        self._size_demand = True
        self._widen_pending = False
        self._last_result = ZoomTrackResult(ZoomTrackingState.IDLE, False)

    @property
    def size_demand(self) -> bool:
        return self._size_demand

    @property
    def widen_pending(self) -> bool:
        return self._widen_pending

    @property
    def last_result(self) -> ZoomTrackResult:
        return self._last_result

    def set_size_demand(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if self._size_demand and not enabled:
            self._widen_pending = True
        self._size_demand = enabled

    def complete_widen(self) -> None:
        self._widen_pending = False

    def start(self) -> None:
        self._size_demand = True
        self._widen_pending = False
        self.reset_result()

    def reset_result(self) -> None:
        self._last_result = ZoomTrackResult(ZoomTrackingState.IDLE, False)

    def record(self, result: ZoomTrackResult) -> ZoomTrackResult:
        self._last_result = result
        return result

    def target_pixels(self, target: object) -> float:
        if not self._size_demand:
            return float(MIN_TRACK_PIXELS)
        class_id = (
            target.classification.class_id
            if isinstance(target, DetectedObject)
            else getattr(target, "class_id", None)
        )
        return self.target_pixels_for_class(class_id)

    def target_pixels_for_class(self, class_id: object) -> float:
        key = "default" if class_id is None else str(class_id)
        default = float(self._config.target_pixels["default"])
        return float(self._config.target_pixels.get(key, default))


__all__ = ["TargetZoomSession"]
