"""Pairwise geometry and appearance scores for identity re-association."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_appearance import gallery_distance
from navpy.modules.vision.track_identity_types import (
    IdentityAppearancePolicy,
    IdentityCounters,
    IdentityGeometryPolicy,
    IdentityState,
)

INFINITE_MATCH_COST = float("inf")


@dataclass(frozen=True)
class IdentityPairScores:
    costs: list[list[float]]
    appearance: list[list[float | None]]


def normalized_box(
    track: TrackedObject,
    frame_width: int,
    frame_height: int,
) -> tuple[float, float, float, float]:
    width = max(float(frame_width), 1.0)
    height = max(float(frame_height), 1.0)
    return (
        float(track.cx) / width,
        float(track.cy) / height,
        max(float(track.w), 1.0) / width,
        max(float(track.h), 1.0) / height,
    )


def size_error(
    width_a: float,
    height_a: float,
    width_b: float,
    height_b: float,
) -> float:
    return abs(
        math.log(max(width_a, 1e-6) / max(width_b, 1e-6))
    ) + abs(
        math.log(max(height_a, 1e-6) / max(height_b, 1e-6))
    )


class IdentityPairScorer:
    """Build pair matrices while recording each rejected evidence gate."""

    def __init__(
        self,
        geometry: IdentityGeometryPolicy,
        appearance: IdentityAppearancePolicy,
        counters: IdentityCounters,
    ) -> None:
        self._geometry = geometry
        self._appearance = appearance
        self._counters = counters

    def score(
        self,
        tracks: list[TrackedObject],
        states: list[IdentityState],
        frame_width: int,
        frame_height: int,
        embeddings: dict[int, np.ndarray],
        now: float,
    ) -> IdentityPairScores:
        row_count, column_count = len(tracks), len(states)
        costs = [
            [INFINITE_MATCH_COST] * column_count
            for _ in range(row_count)
        ]
        appearance_costs: list[list[float | None]] = [
            [None] * column_count
            for _ in range(row_count)
        ]
        for row, track in enumerate(tracks):
            embedding = embeddings.get(int(track.id))
            nx, ny, nw, nh = normalized_box(
                track,
                frame_width,
                frame_height,
            )
            for column, state in enumerate(states):
                if state.class_id != int(track.class_id):
                    continue
                motion = state.kinematics
                box_size_error = size_error(nw, nh, motion.nw, motion.nh)
                size_blocked = (
                    box_size_error > float(self._geometry.max_size_error)
                )
                elapsed = max(0.0, now - motion.last_seen)
                horizon = min(
                    elapsed,
                    float(self._geometry.predict_horizon),
                )
                center_distance = math.hypot(
                    nx - (motion.nx + motion.nvx * horizon),
                    ny - (motion.ny + motion.nvy * horizon),
                )
                gate = min(
                    float(self._geometry.max_center_distance)
                    + float(self._geometry.motion_gate_growth) * elapsed,
                    float(self._geometry.motion_gate_cap),
                )
                if embedding is None:
                    if self._geometry_blocked(
                        size_blocked,
                        center_distance,
                        gate,
                    ):
                        continue
                    costs[row][column] = (
                        center_distance + 0.03 * box_size_error
                    )
                    continue
                if not state.appearance.prototypes:
                    continue
                appearance_distance = gallery_distance(
                    embedding,
                    state.appearance.prototypes,
                )
                if appearance_distance <= float(self._appearance.threshold):
                    appearance_costs[row][column] = appearance_distance
                    costs[row][column] = appearance_distance + 0.1 * (
                        center_distance + 0.03 * box_size_error
                    )
                elif (
                    appearance_distance
                    <= self._appearance.contradiction_threshold
                ):
                    if self._geometry_blocked(
                        size_blocked,
                        center_distance,
                        gate,
                    ):
                        continue
                    costs[row][column] = (
                        center_distance + 0.03 * box_size_error
                    )
                else:
                    self._counters.bump("rej_contradict")
        return IdentityPairScores(costs, appearance_costs)

    def _geometry_blocked(
        self,
        size_blocked: bool,
        center_distance: float,
        gate: float,
    ) -> bool:
        if size_blocked:
            self._counters.bump("rej_size")
            return True
        if center_distance > gate:
            self._counters.bump("rej_center")
            return True
        return False


__all__ = [
    "INFINITE_MATCH_COST",
    "IdentityPairScorer",
    "IdentityPairScores",
    "normalized_box",
    "size_error",
]
