"""Fail-closed assignment and raw binding of unmatched tracks."""

from __future__ import annotations

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_matching import LostIdentityMatcher
from navpy.modules.vision.track_identity_registry import IdentityRegistry
from navpy.modules.vision.track_identity_types import IdentityCounters


class UnmatchedIdentityBinder:
    """Assign lost identities or allocate forks, then commit raw bindings."""

    def __init__(
        self,
        registry: IdentityRegistry,
        matcher: LostIdentityMatcher,
        counters: IdentityCounters,
    ) -> None:
        self._registry = registry
        self._matcher = matcher
        self._counters = counters

    def bind(
        self,
        tracks: list[TrackedObject],
        used: set[int],
        current_raw: set[int],
        frame_width: int,
        frame_height: int,
        embeddings: dict[int, np.ndarray],
        now: float,
    ) -> list[tuple[TrackedObject, int]]:
        candidates = self._registry.rebind_candidates(used, current_raw)
        assignment = self._matcher.assign(
            tracks,
            candidates,
            frame_width,
            frame_height,
            embeddings,
            now,
        )
        bindings: list[tuple[TrackedObject, int]] = []
        for index, track in enumerate(tracks):
            stable_id = assignment.get(index)
            if stable_id is None:
                self._counters.bump("forks")
                release_previous = False
            else:
                counter = (
                    "rebinds_appearance"
                    if int(track.id) in embeddings
                    else "rebinds_geometry"
                )
                self._counters.bump(counter)
                release_previous = True
            stable_id = self._registry.bind(
                int(track.id),
                stable_id,
                release_previous=release_previous,
            )
            bindings.append((track, stable_id))
        return bindings


__all__ = ["UnmatchedIdentityBinder"]
