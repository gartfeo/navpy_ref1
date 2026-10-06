"""Late merge and observation-state commit for resolved track bindings."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_appearance import (
    IdentityGallery,
    LateIdentityMerger,
)
from navpy.modules.vision.track_identity_registry import IdentityRegistry
from navpy.modules.vision.track_identity_scoring import normalized_box
from navpy.modules.vision.track_identity_types import (
    IdentityKinematics,
    IdentityState,
)


class IdentityObservationCommitter:
    """Merge resolved IDs first, then store observations in binding order."""

    def __init__(
        self,
        registry: IdentityRegistry,
        gallery: IdentityGallery,
        merger: LateIdentityMerger,
    ) -> None:
        self._registry = registry
        self._gallery = gallery
        self._merger = merger

    def commit(
        self,
        bindings: list[tuple[TrackedObject, int]],
        frame_width: int,
        frame_height: int,
        now: float,
        embeddings: dict[int, np.ndarray],
    ) -> list[TrackedObject]:
        merged = [
            (
                track,
                self._merger.maybe_merge(
                    track,
                    stable_id,
                    now,
                    embeddings,
                ),
            )
            for track, stable_id in bindings
        ]
        output: list[TrackedObject] = []
        for track, stable_id in merged:
            self._touch(
                stable_id,
                track,
                frame_width,
                frame_height,
                now,
                embeddings.get(int(track.id)),
            )
            output.append(replace(track, id=stable_id))
        return output

    def _touch(
        self,
        stable_id: int,
        track: TrackedObject,
        frame_width: int,
        frame_height: int,
        now: float,
        embedding: np.ndarray | None,
    ) -> None:
        previous = self._registry.state_of(stable_id)
        nx, ny, nw, nh = normalized_box(
            track,
            frame_width,
            frame_height,
        )
        continuing_raw = (
            previous is not None and previous.raw_id == int(track.id)
        )
        self._registry.store(
            IdentityState(
                stable_id=stable_id,
                raw_id=int(track.id),
                class_id=int(track.class_id),
                kinematics=IdentityKinematics(
                    nx=nx,
                    ny=ny,
                    nw=nw,
                    nh=nh,
                    nvx=float(track.vx) / max(float(frame_width), 1.0),
                    nvy=float(track.vy) / max(float(frame_height), 1.0),
                    last_seen=now,
                ),
                appearance=self._gallery.refresh(
                    previous,
                    embedding,
                    continuing_raw,
                ),
                created_at=(
                    previous.created_at if previous is not None else now
                ),
            )
        )


__all__ = ["IdentityObservationCommitter"]
