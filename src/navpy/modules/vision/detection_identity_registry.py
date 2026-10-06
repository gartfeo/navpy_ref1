"""Coordinator-owned target identity and publication-mode registry."""

from __future__ import annotations

import weakref
import threading
from collections.abc import Callable, Iterable
from typing import TYPE_CHECKING

from navpy.modules.vision.target_identity import (
    TargetIdentity,
    TargetTaskIdAllocator,
)
from navpy.modules.vision.target_priority import prioritize_targets

if TYPE_CHECKING:
    from navpy.modules.vision.models.detect_data import DetectedObject


class DetectionIdentityRegistry:
    """Own task allocation and per-target source publication mode."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._allocator = TargetTaskIdAllocator()
        self._source_driven = weakref.WeakKeyDictionary()

    def reset(self) -> None:
        with self._lock:
            self._allocator.reset()
            self._source_driven.clear()

    def remember_publication_mode(
            self,
            targets: Iterable["DetectedObject"],
            *,
            source_driven: bool,
    ) -> None:
        with self._lock:
            for target in targets:
                try:
                    self._source_driven[target] = bool(source_driven)
                except TypeError:
                    continue

    def target_uses_source_driven_events(
            self,
            target: "DetectedObject",
    ) -> bool | None:
        with self._lock:
            try:
                return self._source_driven.get(target)
            except TypeError:
                return None

    def normalize_targets(
            self,
            targets: list["DetectedObject"],
            primary_candidates: list["DetectedObject"],
            *,
            select_primary: Callable[
                [Iterable["DetectedObject"]],
                "DetectedObject | None",
            ],
    ) -> list["DetectedObject"]:
        with self._lock:
            for target in targets:
                self._allocator.assign(target)
            for target in primary_candidates:
                self._allocator.assign(target)
            primary_target = select_primary(primary_candidates or targets)
            return prioritize_targets(targets, primary_target)

    def identity_for_task(self, task_id: int) -> TargetIdentity | None:
        with self._lock:
            return self._allocator.get_identity(task_id)

    def rebind_task_id(self, task_id: int, target: "DetectedObject") -> bool:
        with self._lock:
            return self._allocator.rebind(task_id, target)
