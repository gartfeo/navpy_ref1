"""Classification of already-live raw identity bindings."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_appearance import IdentityGallery
from navpy.modules.vision.track_identity_registry import IdentityRegistry
from navpy.modules.vision.track_identity_types import IdentityCounters


@dataclass(frozen=True)
class LiveBindingPartition:
    mapped: list[tuple[TrackedObject, int]]
    unmapped: list[TrackedObject]
    used: set[int]


class LiveBindingClassifier:
    """Keep healthy live bindings and release persistent appearance swaps."""

    def __init__(
        self,
        registry: IdentityRegistry,
        gallery: IdentityGallery,
        counters: IdentityCounters,
    ) -> None:
        self._registry = registry
        self._gallery = gallery
        self._counters = counters

    def classify(
        self,
        tracks: list[TrackedObject],
        embeddings: dict[int, np.ndarray],
    ) -> LiveBindingPartition:
        used: set[int] = set()
        mapped: list[tuple[TrackedObject, int]] = []
        unmapped: list[TrackedObject] = []
        for track in tracks:
            raw_id = int(track.id)
            binding = self._registry.binding_of(raw_id)
            if binding is None or binding[0] in used:
                unmapped.append(track)
                continue
            stable_id, state = binding
            if self._gallery.is_live_swap(
                state,
                embeddings.get(raw_id),
            ):
                self._registry.drop_live_mapping(raw_id)
                self._counters.bump("swaps")
                unmapped.append(track)
                continue
            mapped.append((track, stable_id))
            used.add(stable_id)
        return LiveBindingPartition(mapped, unmapped, used)


__all__ = ["LiveBindingClassifier", "LiveBindingPartition"]
