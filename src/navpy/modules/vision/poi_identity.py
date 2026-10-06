"""Canonical identity operations for grouped detections."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class PoiIdentity:
    source_name: str
    local_obj_id: int


def get_poi_task_id(poi: DetectedObject) -> int | None:
    """Return the allocated navigation task identity for one grouped detection."""
    return poi.identity.task_id


def find_poi_by_task_id(
    pois: Iterable[DetectedObject],
    task_id: int | None,
) -> DetectedObject | None:
    if task_id is None:
        return None
    return next(
        (
            poi
            for poi in pois
            if get_poi_task_id(poi) == task_id
        ),
        None,
    )


def same_poi_identity(
    left: DetectedObject,
    right: DetectedObject,
) -> bool:
    left_id = get_poi_task_id(left)
    right_id = get_poi_task_id(right)
    return (
        left_id is not None
        and right_id is not None
        and left_id == right_id
    )


def get_poi_identity(poi: DetectedObject) -> PoiIdentity:
    return PoiIdentity(
        source_name=poi.pixel.source_name,
        local_obj_id=poi.identity.obj_id,
    )


class PoiTaskIdAllocator:
    """Assign stable navigation task IDs to detector-local tracks."""

    def __init__(self) -> None:
        self._task_id_by_identity: dict[PoiIdentity, int] = {}
        self._identity_by_task_id: dict[int, PoiIdentity] = {}
        self._next_task_id = 1

    def reset(self) -> None:
        self._task_id_by_identity.clear()
        self._identity_by_task_id.clear()
        self._next_task_id = 1

    def assign(self, poi: DetectedObject) -> int:
        identity = get_poi_identity(poi)
        task_id = self._task_id_by_identity.get(identity)
        if task_id is None:
            task_id = self._next_task_id
            self._task_id_by_identity[identity] = task_id
            self._identity_by_task_id[task_id] = identity
            self._next_task_id += 1
        poi.reidentify(task_id=task_id)
        return task_id

    def get_identity(self, task_id: int) -> PoiIdentity | None:
        return self._identity_by_task_id.get(int(task_id))

    def rebind(self, task_id: int, poi: DetectedObject) -> bool:
        new_identity = get_poi_identity(poi)
        task_id = int(task_id)
        old_identity = self._identity_by_task_id.get(task_id)
        if old_identity is not None:
            del self._task_id_by_identity[old_identity]
        stale_task_id = self._task_id_by_identity.get(new_identity)
        if stale_task_id is not None and stale_task_id != task_id:
            del self._identity_by_task_id[stale_task_id]
        self._task_id_by_identity[new_identity] = task_id
        self._identity_by_task_id[task_id] = new_identity
        poi.reidentify(task_id=task_id)
        return True


__all__ = [
    "PoiIdentity",
    "PoiTaskIdAllocator",
    "find_poi_by_task_id",
    "get_poi_identity",
    "get_poi_task_id",
    "same_poi_identity",
]
