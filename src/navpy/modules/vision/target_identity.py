"""Canonical identity operations for grouped detections."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class TargetIdentity:
    source_name: str
    local_obj_id: int


def get_target_task_id(target: DetectedObject) -> int | None:
    """Return the allocated navigation task identity for one grouped detection."""
    return target.identity.task_id


def find_target_by_task_id(
    targets: Iterable[DetectedObject],
    task_id: int | None,
) -> DetectedObject | None:
    if task_id is None:
        return None
    return next(
        (
            target
            for target in targets
            if get_target_task_id(target) == task_id
        ),
        None,
    )


def same_target_identity(
    left: DetectedObject,
    right: DetectedObject,
) -> bool:
    left_id = get_target_task_id(left)
    right_id = get_target_task_id(right)
    return (
        left_id is not None
        and right_id is not None
        and left_id == right_id
    )


def get_target_identity(target: DetectedObject) -> TargetIdentity:
    return TargetIdentity(
        source_name=target.pixel.source_name,
        local_obj_id=target.identity.obj_id,
    )


class TargetTaskIdAllocator:
    """Assign stable navigation task IDs to detector-local tracks."""

    def __init__(self) -> None:
        self._task_id_by_identity: dict[TargetIdentity, int] = {}
        self._identity_by_task_id: dict[int, TargetIdentity] = {}
        self._next_task_id = 1

    def reset(self) -> None:
        self._task_id_by_identity.clear()
        self._identity_by_task_id.clear()
        self._next_task_id = 1

    def assign(self, target: DetectedObject) -> int:
        identity = get_target_identity(target)
        task_id = self._task_id_by_identity.get(identity)
        if task_id is None:
            task_id = self._next_task_id
            self._task_id_by_identity[identity] = task_id
            self._identity_by_task_id[task_id] = identity
            self._next_task_id += 1
        target.reidentify(task_id=task_id)
        return task_id

    def get_identity(self, task_id: int) -> TargetIdentity | None:
        return self._identity_by_task_id.get(int(task_id))

    def rebind(self, task_id: int, target: DetectedObject) -> bool:
        new_identity = get_target_identity(target)
        task_id = int(task_id)
        old_identity = self._identity_by_task_id.get(task_id)
        if old_identity is not None:
            del self._task_id_by_identity[old_identity]
        stale_task_id = self._task_id_by_identity.get(new_identity)
        if stale_task_id is not None and stale_task_id != task_id:
            del self._identity_by_task_id[stale_task_id]
        self._task_id_by_identity[new_identity] = task_id
        self._identity_by_task_id[task_id] = new_identity
        target.reidentify(task_id=task_id)
        return True


__all__ = [
    "TargetIdentity",
    "TargetTaskIdAllocator",
    "find_target_by_task_id",
    "get_target_identity",
    "get_target_task_id",
    "same_target_identity",
]
