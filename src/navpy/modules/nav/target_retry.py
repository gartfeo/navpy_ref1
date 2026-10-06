from __future__ import annotations

from typing import Callable, Optional

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import (
    get_target_identity,
    get_target_task_id,
)


class TargetRetryState:
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
    def identity_key(target: DetectedObject) -> Optional[tuple]:
        task_id = get_target_task_id(target)
        if task_id is not None:
            return "task", task_id
        identity = get_target_identity(target)
        if identity is None:
            return None
        return "id", identity.source_name, identity.local_obj_id

    def register_cooldown(self, target: DetectedObject) -> None:
        key = self.identity_key(target)
        if key is not None:
            self.cooldowns[key] = self._wall_clock_s() + self._cooldown_s

    def is_cooling_down(self, target: DetectedObject) -> bool:
        key = self.identity_key(target)
        if key is None:
            return False
        expiry = self.cooldowns.get(key)
        if expiry is None:
            return False
        if self._wall_clock_s() < expiry:
            return True
        del self.cooldowns[key]
        return False

    def can_reask(self, target: DetectedObject) -> bool:
        target_id = get_target_task_id(target)
        return (
            target_id is not None
            and self.reask_attempts.get(target_id, 0)
            < self._max_reask_attempts
        )

    def begin_reask(self, target: DetectedObject) -> None:
        target_id = get_target_task_id(target)
        if target_id is not None:
            self.reask_attempts[target_id] = (
                self.reask_attempts.get(target_id, 0) + 1
            )

    def clear_reask(self, target_id: Optional[int]) -> None:
        if target_id is not None:
            self.reask_attempts.pop(target_id, None)

    def full_reset(self) -> None:
        self.cooldowns.clear()
        self.reask_attempts.clear()
