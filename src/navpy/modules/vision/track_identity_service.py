"""Stable-identity update transaction and explicit policy composition."""

from __future__ import annotations

import time
from collections.abc import Iterable
from typing import Callable, Protocol

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_assignment import (
    UnmatchedIdentityBinder,
)
from navpy.modules.vision.track_identity_appearance import (
    IdentityGallery,
    LateIdentityMerger,
)
from navpy.modules.vision.track_identity_commit import (
    IdentityObservationCommitter,
)
from navpy.modules.vision.track_identity_live import LiveBindingClassifier
from navpy.modules.vision.track_identity_matching import LostIdentityMatcher
from navpy.modules.vision.track_identity_registry import IdentityRegistry
from navpy.modules.vision.track_identity_types import (
    IdentityCounters,
    IdentityResolverDiagnostics,
    TrackIdentityPolicies,
)


class IdentityResolutionPort(Protocol):
    @property
    def counters(self) -> dict[str, int]: ...

    def update(
        self,
        tracks: Iterable[TrackedObject],
        frame_width: int,
        frame_height: int,
        *,
        now: float | None = None,
        embeddings: dict[int, np.ndarray] | None = None,
    ) -> list[TrackedObject]: ...

    def reset(self) -> None: ...

    def pin(self, stable_id: int | None) -> None: ...

    def stable_of(self, raw_id: int) -> int | None: ...

    def hint_position(
        self,
        stable_id: int,
        nx: float,
        ny: float,
        now: float,
    ) -> None: ...

    def diagnostics(self) -> IdentityResolverDiagnostics: ...


class TrackIdentityService:
    """Coordinate one identity update over focused state and policy owners."""

    def __init__(
        self,
        registry: IdentityRegistry,
        live_bindings: LiveBindingClassifier,
        unmatched_bindings: UnmatchedIdentityBinder,
        observations: IdentityObservationCommitter,
        counters: IdentityCounters,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._registry = registry
        self._live_bindings = live_bindings
        self._unmatched_bindings = unmatched_bindings
        self._observations = observations
        self._counters = counters
        self._monotonic = monotonic

    @property
    def counters(self) -> dict[str, int]:
        return self._counters.values

    def update(
        self,
        tracks: Iterable[TrackedObject],
        frame_width: int,
        frame_height: int,
        *,
        now: float | None = None,
        embeddings: dict[int, np.ndarray] | None = None,
    ) -> list[TrackedObject]:
        update_time = self._monotonic() if now is None else float(now)
        frame_embeddings = embeddings or {}
        self._registry.expire(update_time)

        track_list = list(tracks)
        current_raw = {int(track.id) for track in track_list}
        self._registry.begin_frame(current_raw, update_time)

        partition = self._live_bindings.classify(
            track_list,
            frame_embeddings,
        )
        unmatched = self._unmatched_bindings.bind(
            partition.unmapped,
            partition.used,
            current_raw,
            frame_width,
            frame_height,
            frame_embeddings,
            update_time,
        )
        return self._observations.commit(
            [*partition.mapped, *unmatched],
            frame_width,
            frame_height,
            update_time,
            frame_embeddings,
        )

    def reset(self) -> None:
        self._registry.reset()

    def pin(self, stable_id: int | None) -> None:
        self._registry.pin(stable_id)

    def stable_of(self, raw_id: int) -> int | None:
        binding = self._registry.binding_of(int(raw_id))
        return None if binding is None else binding[0]

    def hint_position(
        self,
        stable_id: int,
        nx: float,
        ny: float,
        now: float,
    ) -> None:
        self._registry.hint_position(stable_id, nx, ny, now)

    def diagnostics(self) -> IdentityResolverDiagnostics:
        return self._registry.diagnostics(self._counters.snapshot())


def build_track_identity_service(
    policies: TrackIdentityPolicies = TrackIdentityPolicies(),
) -> TrackIdentityService:
    """Compose policy owners without passing one broad bag into behavior."""
    counters = IdentityCounters()
    registry = IdentityRegistry(policies.retention)
    gallery = IdentityGallery(policies.appearance)
    matcher = LostIdentityMatcher(
        policies.geometry,
        policies.appearance,
        counters,
    )
    merger = LateIdentityMerger(
        registry,
        policies.appearance,
        policies.retention,
        counters,
    )
    return TrackIdentityService(
        registry=registry,
        live_bindings=LiveBindingClassifier(
            registry,
            gallery,
            counters,
        ),
        unmatched_bindings=UnmatchedIdentityBinder(
            registry,
            matcher,
            counters,
        ),
        observations=IdentityObservationCommitter(
            registry,
            gallery,
            merger,
        ),
        counters=counters,
    )


__all__ = [
    "IdentityResolutionPort",
    "TrackIdentityService",
    "build_track_identity_service",
]
