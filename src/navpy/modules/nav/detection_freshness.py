"""Raw-source detection freshness checks for confirmation."""

from __future__ import annotations

import math
from typing import Callable, Optional, Protocol

from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.nav_constants import CONFIRM_FRESH_DETECTION_MAX_AGE_S
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_priority import find_target_by_task_id


class DetectionReceiptAgeQuery(Protocol):
    """Receipt-liveness query used by confirmation freshness."""

    def source_receipt_age_s(
        self,
        target: DetectedObject,
    ) -> Optional[float]: ...


class DetectionFreshnessPolicy:
    """Validate frame age in the producer's own raw timestamp domain."""

    def __init__(
        self,
        detections: DetectionSnapshot,
        source: DetectionReceiptAgeQuery,
        source_fallback_s: Callable[[], float],
        receipt_wall_max_age_s: Callable[[], float],
    ) -> None:
        self._detections = detections
        self._source = source
        self._source_fallback_s = source_fallback_s
        self._receipt_wall_max_age_s = receipt_wall_max_age_s

    def detection_is_fresh_for_confirm(
        self,
        detection: DetectedObject,
        *,
        require_source_receipt: bool = False,
    ) -> bool:
        raw_timestamp = detection.timing.camera_frame_timestamp_s
        if raw_timestamp is None:
            raw_timestamp = detection.timing.tracker_timestamp_s
        try:
            timestamp_s = float(raw_timestamp)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(timestamp_s):
            return False

        now_provider = detection.timing.detection_now_s
        try:
            raw_now_s = (
                now_provider()
                if callable(now_provider)
                else self._source_fallback_s()
            )
            now_s = float(raw_now_s)
        except Exception:  # noqa: BLE001 - external clock failure fails closed
            return False
        age_s = now_s - timestamp_s
        if not (
            math.isfinite(age_s)
            and 0.0 <= age_s <= CONFIRM_FRESH_DETECTION_MAX_AGE_S
        ):
            return False

        try:
            receipt_age_s = self._source.source_receipt_age_s(detection)
        except Exception:  # noqa: BLE001 - external receipt clock fails closed
            return False
        if receipt_age_s is None:
            return not require_source_receipt
        try:
            receipt_wall_max_age_s = float(self._receipt_wall_max_age_s())
        except Exception:  # noqa: BLE001 - cadence provider failure fails closed
            return False
        return (
            math.isfinite(receipt_age_s)
            and math.isfinite(receipt_wall_max_age_s)
            and receipt_wall_max_age_s > 0.0
            and 0.0 <= receipt_age_s <= receipt_wall_max_age_s
        )

    def is_target_fresh_for_confirm(self, target_id: int) -> bool:
        detection = find_target_by_task_id(
            self._detections.targets(),
            target_id,
        )
        return (
            detection is not None
            and self.detection_is_fresh_for_confirm(detection)
        )


__all__ = ["DetectionFreshnessPolicy", "DetectionReceiptAgeQuery"]
