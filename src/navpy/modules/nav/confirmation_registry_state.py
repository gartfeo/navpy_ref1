"""Thread-safe target statuses and active-target selection."""

from __future__ import annotations

import threading
from enum import Enum, unique
from typing import Optional

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id


@unique
class ConfirmationStatus(Enum):
    CONFIRMING = 0
    CONFIRMED = 20
    TIMEOUT_REJECTED = 25
    REJECTED = 30
    PEER_NOTIFIED = 100


class ConfirmationRegistryState:
    """Own active-target selection and status transitions."""

    def __init__(self, lock: threading.RLock) -> None:
        self._lock = lock
        self._statuses: dict[int, ConfirmationStatus] = {}
        self._active_target: Optional[DetectedObject] = None

    @property
    def active_target(self) -> Optional[DetectedObject]:
        with self._lock:
            return self._active_target

    def set_active_target(self, target: Optional[DetectedObject]) -> None:
        with self._lock:
            self._active_target = target

    def update_status(self, target: DetectedObject, status: ConfirmationStatus) -> None:
        target_id = get_target_task_id(target)
        if target_id is not None:
            self.set_status(target_id, status)

    def set_status(self, target_id: int, status: ConfirmationStatus) -> None:
        with self._lock:
            self._statuses[target_id] = status

    def status_by_id(self, target_id: int) -> Optional[ConfirmationStatus]:
        with self._lock:
            return self._statuses.get(target_id)

    def get_status(self, target: DetectedObject) -> Optional[ConfirmationStatus]:
        target_id = get_target_task_id(target)
        return None if target_id is None else self.status_by_id(target_id)

    def clear_status(self, target: DetectedObject) -> None:
        target_id = get_target_task_id(target)
        if target_id is not None:
            with self._lock:
                self._statuses.pop(target_id, None)

    def reset(self) -> None:
        with self._lock:
            self._statuses.clear()
            self._active_target = None


__all__ = ["ConfirmationRegistryState", "ConfirmationStatus"]
