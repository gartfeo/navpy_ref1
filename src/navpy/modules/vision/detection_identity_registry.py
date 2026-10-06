"""Coordinator-owned POI identity and publication-mode registry."""

from __future__ import annotations

import weakref
import threading
from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING

from navpy.modules.vision.poi_identity import (
    PoiIdentity,
    PoiTaskIdAllocator,
)
from navpy.modules.vision.poi_priority import prioritize_pois

if TYPE_CHECKING:
    from navpy.modules.vision.models.detect_data import DetectedObject


class DetectionIdentityRegistry:
    """Own task allocation and per-POI source publication mode."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._allocator = PoiTaskIdAllocator()
        self._source_driven = weakref.WeakKeyDictionary()

    def reset(self) -> None:
        with self._lock:
            self._allocator.reset()
            self._source_driven.clear()

    def remember_publication_mode(
            self,
            pois: Iterable["DetectedObject"],
            *,
            source_driven: bool,
    ) -> None:
        with self._lock:
            for poi in pois:
                try:
                    self._source_driven[poi] = bool(source_driven)
                except TypeError:
                    continue

    def poi_uses_source_driven_events(
            self,
            poi: "DetectedObject",
    ) -> bool | None:
        with self._lock:
            try:
                return self._source_driven.get(poi)
            except TypeError:
                return None

    def normalize_pois(
            self,
            pois: list["DetectedObject"],
            primary_candidates: list["DetectedObject"],
            *,
            select_primary: Callable[
                [Iterable["DetectedObject"]],
                "DetectedObject | None",
            ],
    ) -> list["DetectedObject"]:
        with self._lock:
            for poi in pois:
                self._allocator.assign(poi)
            for poi in primary_candidates:
                self._allocator.assign(poi)
            primary_poi = select_primary(primary_candidates or pois)
            return prioritize_pois(pois, primary_poi)

    def identity_for_task(self, task_id: int) -> PoiIdentity | None:
        with self._lock:
            return self._allocator.get_identity(task_id)

    def rebind_task_id(self, task_id: int, poi: "DetectedObject") -> bool:
        with self._lock:
            return self._allocator.rebind(task_id, poi)
