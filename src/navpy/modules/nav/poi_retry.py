from __future__ import annotations

from typing import Callable, Optional

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import (
    get_poi_identity,
    get_poi_task_id,
)


class PoiRetryState:
    """Own frame-local cooldowns and bounded confirmation re-asks."""

    def __init__(
        self,
        *,
        wall_clock_s: Callable[[], float],
        cooldown_s: float,
        max_reask_attempts: int,
    ) -> None:
        self._wall_clock_s = wall_clock_s
        self._cooldown_s = cooldown_s
        self._max_reask_attempts = max_reask_attempts
        self.cooldowns: dict[tuple, float] = {}
        self.reask_attempts: dict[int, int] = {}

    @staticmethod
    def identity_key(poi: DetectedObject) -> Optional[tuple]:
        task_id = get_poi_task_id(poi)
        if task_id is not None:
            return "task", task_id
        identity = get_poi_identity(poi)
        if identity is None:
            return None
        return "id", identity.source_name, identity.local_obj_id

    def register_cooldown(self, poi: DetectedObject) -> None:
        key = self.identity_key(poi)
        if key is not None:
            self.cooldowns[key] = self._wall_clock_s() + self._cooldown_s

    def is_cooling_down(self, poi: DetectedObject) -> bool:
        key = self.identity_key(poi)
        if key is None:
            return False
        expiry = self.cooldowns.get(key)
        if expiry is None:
            return False
        if self._wall_clock_s() < expiry:
            return True
        del self.cooldowns[key]
        return False

    def can_reask(self, poi: DetectedObject) -> bool:
        poi_id = get_poi_task_id(poi)
        return (
            poi_id is not None
            and self.reask_attempts.get(poi_id, 0)
            < self._max_reask_attempts
        )

    def begin_reask(self, poi: DetectedObject) -> None:
        poi_id = get_poi_task_id(poi)
        if poi_id is not None:
            self.reask_attempts[poi_id] = (
                self.reask_attempts.get(poi_id, 0) + 1
            )

    def clear_reask(self, poi_id: Optional[int]) -> None:
        if poi_id is not None:
            self.reask_attempts.pop(poi_id, None)

    def full_reset(self) -> None:
        self.cooldowns.clear()
        self.reask_attempts.clear()
