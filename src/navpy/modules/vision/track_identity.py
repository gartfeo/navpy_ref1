"""Stable public boundary for fail-closed track identity resolution."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_service import (
    IdentityResolutionPort,
    build_track_identity_service,
)
from navpy.modules.vision.track_identity_types import (
    IdentityAppearancePolicy,
    IdentityGeometryPolicy,
    IdentityResolverDiagnostics,
    IdentityRetentionPolicy,
    TrackIdentityPolicies,
)


class TrackIdentityResolver:
    """One-field adapter over the stable-identity update transaction."""

    def __init__(
        self,
        service: IdentityResolutionPort | None = None,
    ) -> None:
        self._service = (
            build_track_identity_service()
            if service is None
            else service
        )

    @property
    def counters(self) -> dict[str, int]:
        return self._service.counters

    def update(
        self,
        tracks: Iterable[TrackedObject],
        frame_w: int,
        frame_h: int,
        *,
        now: float | None = None,
        embeddings: dict[int, np.ndarray] | None = None,
    ) -> list[TrackedObject]:
        return self._service.update(
            tracks,
            frame_w,
            frame_h,
            now=now,
            embeddings=embeddings,
        )

    def reset(self) -> None:
        self._service.reset()

    def pin(self, stable_id: int | None) -> None:
        self._service.pin(stable_id)

    def stable_of(self, raw_id: int) -> int | None:
        return self._service.stable_of(raw_id)

    def hint_position(
        self,
        stable_id: int,
        nx: float,
        ny: float,
        now: float,
    ) -> None:
        self._service.hint_position(stable_id, nx, ny, now)

    def diagnostics(self) -> IdentityResolverDiagnostics:
        return self._service.diagnostics()


def build_track_identity_resolver(
    policies: TrackIdentityPolicies,
) -> TrackIdentityResolver:
    """Explicit non-default composition without a broad public constructor."""
    return TrackIdentityResolver(build_track_identity_service(policies))


__all__ = [
    "IdentityAppearancePolicy",
    "IdentityGeometryPolicy",
    "IdentityResolverDiagnostics",
    "IdentityRetentionPolicy",
    "TrackIdentityPolicies",
    "TrackIdentityResolver",
    "build_track_identity_resolver",
]
