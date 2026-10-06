"""Fail-closed global matching for scored lost identities."""

from __future__ import annotations

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity_scoring import (
    INFINITE_MATCH_COST,
    IdentityPairScorer,
)
from navpy.modules.vision.track_identity_types import (
    IdentityAppearancePolicy,
    IdentityCounters,
    IdentityGeometryPolicy,
    IdentityState,
)


class LostIdentityMatcher:
    """Admit only mutually unambiguous pairs in ascending-cost order."""

    def __init__(
        self,
        geometry: IdentityGeometryPolicy,
        appearance: IdentityAppearancePolicy,
        counters: IdentityCounters,
    ) -> None:
        self._geometry = geometry
        self._appearance = appearance
        self._scorer = IdentityPairScorer(
            geometry,
            appearance,
            counters,
        )

    def assign(
        self,
        unmapped: list[TrackedObject],
        lost_states: list[IdentityState],
        frame_width: int,
        frame_height: int,
        embeddings: dict[int, np.ndarray],
        now: float,
    ) -> dict[int, int]:
        if not unmapped or not lost_states:
            return {}
        scores = self._scorer.score(
            unmapped,
            lost_states,
            frame_width,
            frame_height,
            embeddings,
            now,
        )
        row_count, column_count = len(unmapped), len(lost_states)
        candidates: list[tuple[float, int, int]] = []
        for row in range(row_count):
            for column in range(column_count):
                if scores.costs[row][column] == INFINITE_MATCH_COST:
                    continue
                if scores.appearance[row][column] is not None:
                    safe = self._appearance_pair_is_safe(
                        scores.appearance,
                        row,
                        column,
                        row_count,
                        column_count,
                    )
                else:
                    safe = self._geometry_pair_is_safe(
                        scores.costs,
                        scores.appearance,
                        row,
                        column,
                        row_count,
                        column_count,
                    )
                if safe:
                    candidates.append(
                        (scores.costs[row][column], row, column)
                    )

        candidates.sort(key=lambda candidate: candidate[0])
        assignment: dict[int, int] = {}
        used_rows: set[int] = set()
        used_columns: set[int] = set()
        for _, row, column in candidates:
            if row in used_rows or column in used_columns:
                continue
            assignment[row] = lost_states[column].stable_id
            used_rows.add(row)
            used_columns.add(column)
        return assignment

    def _appearance_pair_is_safe(
        self,
        appearance: list[list[float | None]],
        row: int,
        column: int,
        row_count: int,
        column_count: int,
    ) -> bool:
        candidate = appearance[row][column]
        if candidate is None:
            return False
        row_second = min(
            (
                appearance[row][other]
                for other in range(column_count)
                if other != column
                and appearance[row][other] is not None
            ),
            default=INFINITE_MATCH_COST,
        )
        column_second = min(
            (
                appearance[other][column]
                for other in range(row_count)
                if other != row
                and appearance[other][column] is not None
            ),
            default=INFINITE_MATCH_COST,
        )
        if (
            row_second == INFINITE_MATCH_COST
            and column_second == INFINITE_MATCH_COST
        ):
            return candidate <= self._appearance.strong_threshold
        return (
            row_second - candidate >= float(self._appearance.margin)
            and column_second - candidate >= float(self._appearance.margin)
        )

    def _geometry_pair_is_safe(
        self,
        costs: list[list[float]],
        appearance: list[list[float | None]],
        row: int,
        column: int,
        row_count: int,
        column_count: int,
    ) -> bool:
        candidate = costs[row][column]
        row_second = min(
            (
                costs[row][other]
                for other in range(column_count)
                if other != column
                and costs[row][other] != INFINITE_MATCH_COST
                and appearance[row][other] is None
            ),
            default=INFINITE_MATCH_COST,
        )
        column_second = min(
            (
                costs[other][column]
                for other in range(row_count)
                if other != row
                and costs[other][column] != INFINITE_MATCH_COST
                and appearance[other][column] is None
            ),
            default=INFINITE_MATCH_COST,
        )
        margin = float(self._geometry.ambiguity_margin)
        return (
            row_second - candidate >= margin
            and column_second - candidate >= margin
        )


__all__ = ["LostIdentityMatcher"]
