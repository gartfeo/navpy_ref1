"""Thread-safe POI statuses and active-POI selection."""

from __future__ import annotations

from enum import Enum, unique
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id


@unique
class ConfirmationStatus(Enum):
    CONFIRMING = 0
    CONFIRMED = 20
    TIMEOUT_REJECTED = 25
    REJECTED = 30
    # An own POI dropped for an assigned peer task: never offered to peers
    # (any status blocks that), but the peer approach may take it up again.
    DROPPED = 40
    PEER_NOTIFIED = 100


class ConfirmationRegistryState:
    """Own active-POI selection and status transitions."""

    def __init__(self, lock: rt.RLock) -> None:
        self._lock = lock
        self._statuses: dict[int, ConfirmationStatus] = {}
        # HITL step 25: whether the acquisition gates held on the last navigation tick, per POI;
        # absent (no gates) allows CONFIRMED as before.
        self._acquisition_held: dict[int, bool] = {}
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

    def set_status(self, poi_id: int, status: ConfirmationStatus) -> bool:
        """Set the status; False when CONFIRMED is refused because the gates do not hold.

        A refused CONFIRMED clears the status, so the POI is asked again once they hold.
        """
        with self._lock:
            if status is ConfirmationStatus.CONFIRMED and not self._acquisition_held.get(poi_id, True):
                self._statuses.pop(poi_id, None)
                return False
            self._statuses[poi_id] = status
            return True

    def set_acquisition_held(self, poi_id: int, held: bool) -> None:
        with self._lock:
            self._acquisition_held[poi_id] = held

    def status_by_id(self, poi_id: int) -> Optional[ConfirmationStatus]:
        with self._lock:
            return self._statuses.get(poi_id)

    def get_status(self, poi: DetectedObject) -> Optional[ConfirmationStatus]:
        poi_id = get_poi_task_id(poi)
        return None if poi_id is None else self.status_by_id(poi_id)

    def clear_status(self, poi: DetectedObject) -> None:
        poi_id = get_poi_task_id(poi)
        if poi_id is not None:
            self.clear_status_by_id(poi_id)

    def clear_status_by_id(self, poi_id: int) -> None:
        with self._lock:
            self._statuses.pop(poi_id, None)

    def reset(self) -> None:
        with self._lock:
            self._statuses.clear()
            self._acquisition_held.clear()
            self._active_poi = None


__all__ = ["ConfirmationRegistryState", "ConfirmationStatus"]
