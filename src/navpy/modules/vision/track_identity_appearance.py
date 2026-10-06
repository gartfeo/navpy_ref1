"""Appearance-gallery maintenance, swap detection, and late re-identification."""

from __future__ import annotations

from typing import Optional

import numpy as np

from navpy.modules.vision.appearance import cosine_distance
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_registry import IdentityRegistry
from navpy.modules.vision.track_identity_types import (
    IdentityAppearanceMemory,
    IdentityAppearancePolicy,
    IdentityCounters,
    IdentityRetentionPolicy,
    IdentityState,
)

_INF = float("inf")


def gallery_distance(
    embedding: np.ndarray,
    prototypes: list[np.ndarray],
) -> float:
    if not prototypes:
        return 1.0
    return min(cosine_distance(embedding, prototype) for prototype in prototypes)


def blend_embedding(
    previous: Optional[np.ndarray],
    new: np.ndarray,
    *,
    momentum: float = 0.5,
) -> np.ndarray:
    new_vector = np.asarray(new, dtype=np.float32).ravel()
    if previous is None:
        vector = new_vector
    else:
        previous_vector = np.asarray(previous, dtype=np.float32).ravel()
        vector = (
            new_vector
            if previous_vector.shape != new_vector.shape
            else momentum * previous_vector + (1.0 - momentum) * new_vector
        )
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-12 else vector


class IdentityGallery:
    """Own live-swap debounce and multi-viewpoint appearance memory."""

    def __init__(self, policy: IdentityAppearancePolicy) -> None:
        self._policy = policy

    def is_live_swap(
        self,
        state: IdentityState | None,
        embedding: np.ndarray | None,
    ) -> bool:
        if (
            embedding is None
            or state is None
            or not state.appearance.prototypes
        ):
            return False
        memory = state.appearance
        diverged = (
            gallery_distance(embedding, memory.prototypes)
            > float(self._policy.swap_threshold)
        )
        if not diverged:
            memory.swap_mismatch_count = 0
            return False
        abrupt = (
            memory.last_embedding is None
            or cosine_distance(embedding, memory.last_embedding)
            > self._policy.continuity_threshold
        )
        if abrupt or memory.swap_mismatch_count > 0:
            memory.swap_mismatch_count += 1
            return memory.swap_mismatch_count >= max(
                1,
                int(self._policy.swap_patience),
            )
        return False

    def refresh(
        self,
        previous: IdentityState | None,
        embedding: np.ndarray | None,
        continuing_raw: bool,
    ) -> IdentityAppearanceMemory:
        previous_memory = (
            previous.appearance
            if previous is not None
            else IdentityAppearanceMemory()
        )
        prototypes = list(previous_memory.prototypes)
        last_embedding = previous_memory.last_embedding
        if embedding is not None:
            prototypes = self._update_prototypes(
                prototypes,
                embedding,
                previous_memory.last_embedding,
                previous_memory.swap_mismatch_count,
            )
            last_embedding = np.asarray(
                embedding,
                dtype=np.float32,
            ).ravel()
        return IdentityAppearanceMemory(
            prototypes=prototypes,
            last_embedding=last_embedding,
            swap_mismatch_count=(
                previous_memory.swap_mismatch_count if continuing_raw else 0
            ),
        )

    def _update_prototypes(
        self,
        prototypes: list[np.ndarray],
        embedding: np.ndarray,
        previous_last: np.ndarray | None,
        previous_mismatches: int,
    ) -> list[np.ndarray]:
        vector = np.asarray(embedding, dtype=np.float32).ravel()
        if not prototypes:
            return [vector]
        distances = [
            cosine_distance(vector, prototype)
            for prototype in prototypes
        ]
        nearest = int(np.argmin(distances))
        if distances[nearest] <= self._policy.prototype_match_threshold:
            prototypes[nearest] = blend_embedding(
                prototypes[nearest],
                vector,
            )
            return prototypes
        continuous = (
            previous_last is not None
            and cosine_distance(vector, previous_last)
            <= self._policy.continuity_threshold
        )
        if (
            continuous
            and previous_mismatches == 0
            and distances[nearest] <= float(self._policy.swap_threshold)
        ):
            if len(prototypes) >= max(1, int(self._policy.max_prototypes)):
                redundancy = [
                    min(
                        cosine_distance(prototype, other)
                        for index, other in enumerate(prototypes)
                        if index != prototype_index
                    )
                    for prototype_index, prototype in enumerate(prototypes)
                ]
                prototypes.pop(int(np.argmin(redundancy)))
            prototypes.append(vector)
        return prototypes


class LateIdentityMerger:
    """Merge a young fail-closed fork into an older strong appearance match."""

    def __init__(
        self,
        registry: IdentityRegistry,
        appearance: IdentityAppearancePolicy,
        retention: IdentityRetentionPolicy,
        counters: IdentityCounters,
    ) -> None:
        self._registry = registry
        self._appearance = appearance
        self._retention = retention
        self._counters = counters

    def maybe_merge(
        self,
        track: TrackedObject,
        stable_id: int,
        now: float,
        embeddings: dict[int, np.ndarray],
    ) -> int:
        raw_id = int(track.id)
        embedding = embeddings.get(raw_id)
        if embedding is None:
            return stable_id
        candidates = [
            (
                gallery_distance(
                    embedding,
                    state.appearance.prototypes,
                ),
                state.stable_id,
            )
            for state in self._registry.late_merge_candidates(
                stable_id,
                int(track.class_id),
                now,
                float(self._retention.merge_window),
            )
        ]
        if not candidates:
            return stable_id
        candidates.sort(key=lambda candidate: candidate[0])
        best, best_id = candidates[0]
        second = candidates[1][0] if len(candidates) > 1 else _INF
        if (
            best > self._appearance.strong_threshold
            or second - best < float(self._appearance.margin)
        ):
            return stable_id
        self._registry.merge(raw_id, stable_id, best_id)
        self._counters.bump("late_merges")
        return best_id


__all__ = [
    "IdentityGallery",
    "LateIdentityMerger",
    "blend_embedding",
    "gallery_distance",
]
