"""Thread-safe POI statuses and active-POI selection."""

from __future__ import annotations

import threading
from enum import Enum, unique
from typing import Optional

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


@unique
class ConfirmationStatus(Enum):
    CONFIRMING = 0
    CONFIRMED = 20
    TIMEOUT_REJECTED = 25
    REJECTED = 30
    PEER_NOTIFIED = 100


class ConfirmationRegistryState:
    """Own active-POI selection and status transitions."""

    def __init__(self, lock: threading.RLock) -> None:
        self._lock = lock
        self._statuses: dict[int, ConfirmationStatus] = {}
        self._active_poi: Optional[DetectedObject] = None

    @property
    def active_poi(self) -> Optional[DetectedObject]:
        with self._lock:
            return self._active_poi

    def set_active_poi(self, poi: Optional[DetectedObject]) -> None:
        with self._lock:
            self._active_poi = poi

    def update_status(self, poi: DetectedObject, status: ConfirmationStatus) -> None:
        poi_id = get_poi_task_id(poi)
        if poi_id is not None:
            self.set_status(poi_id, status)

    def set_status(self, poi_id: int, status: ConfirmationStatus) -> None:
        with self._lock:
            self._statuses[poi_id] = status

    def status_by_id(self, poi_id: int) -> Optional[ConfirmationStatus]:
        with self._lock:
            return self._statuses.get(poi_id)

    def get_status(self, poi: DetectedObject) -> Optional[ConfirmationStatus]:
        poi_id = get_poi_task_id(poi)
        return None if poi_id is None else self.status_by_id(poi_id)

    def clear_status(self, poi: DetectedObject) -> None:
        poi_id = get_poi_task_id(poi)
        if poi_id is not None:
            with self._lock:
                self._statuses.pop(poi_id, None)

    def reset(self) -> None:
        with self._lock:
            self._statuses.clear()
            self._active_poi = None


__all__ = ["ConfirmationRegistryState", "ConfirmationStatus"]
