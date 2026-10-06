"""Mutable identity registry, retention, and raw-binding ownership."""

from __future__ import annotations

from collections.abc import Iterable

from navpy.modules.vision.track_identity_types import (
    IdentityResolverDiagnostics,
    IdentityRetentionPolicy,
    IdentityState,
)


class IdentityRegistry:
    """Own all mutable raw/stable identity maps and retention rules."""

    def __init__(self, policy: IdentityRetentionPolicy) -> None:
        self._policy = policy
        self._next_id = 1
        self._raw_to_stable: dict[int, int] = {}
        self._raw_last_seen: dict[int, float] = {}
        self._states: dict[int, IdentityState] = {}
        self._pinned_id: int | None = None

    def begin_frame(self, current_raw: set[int], now: float) -> None:
        for raw_id in current_raw:
            self._raw_last_seen[raw_id] = now
        for raw_id in list(self._raw_to_stable):
            if raw_id in current_raw:
                continue
            if (
                now - self._raw_last_seen.get(raw_id, now)
                <= float(self._policy.grace_seconds)
            ):
                continue
            stable_id = self._raw_to_stable.pop(raw_id)
            self._raw_last_seen.pop(raw_id, None)
            state = self._states.get(stable_id)
            if state is not None:
                state.raw_id = None

    def binding_of(self, raw_id: int) -> tuple[int, IdentityState | None] | None:
        stable_id = self._raw_to_stable.get(int(raw_id))
        if stable_id is None:
            return None
        return stable_id, self._states.get(stable_id)

    def state_of(self, stable_id: int) -> IdentityState | None:
        return self._states.get(int(stable_id))

    def rebind_candidates(
        self,
        used: set[int],
        current_raw: set[int],
    ) -> list[IdentityState]:
        return [
            state
            for state in self._states.values()
            if state.stable_id not in used
            and (
                state.raw_id is None
                or state.raw_id not in current_raw
            )
        ]

    def bind(
        self,
        raw_id: int,
        stable_id: int | None,
        *,
        release_previous: bool,
    ) -> int:
        if stable_id is None:
            stable_id = self._next_id
            self._next_id += 1
        if release_previous:
            state = self._states.get(stable_id)
            if state is not None and state.raw_id is not None:
                self._raw_to_stable.pop(state.raw_id, None)
                self._raw_last_seen.pop(state.raw_id, None)
        self._raw_to_stable[int(raw_id)] = stable_id
        return stable_id

    def drop_live_mapping(self, raw_id: int) -> None:
        stable_id = self._raw_to_stable.pop(int(raw_id), None)
        if stable_id is None:
            return
        state = self._states.get(stable_id)
        if state is not None:
            state.raw_id = None

    def store(self, state: IdentityState) -> None:
        self._states[state.stable_id] = state

    def merge(self, raw_id: int, young_id: int, older_id: int) -> None:
        self._raw_to_stable[int(raw_id)] = older_id
        self._states.pop(young_id, None)
        if self._pinned_id == young_id:
            self._pinned_id = older_id

    def late_merge_candidates(
        self,
        stable_id: int,
        class_id: int,
        now: float,
        merge_window: float,
    ) -> Iterable[IdentityState]:
        young = self._states.get(stable_id)
        born = young.created_at if young is not None else now
        if now - born > merge_window:
            return ()
        return tuple(
            state
            for state in self._states.values()
            if state.stable_id != stable_id
            and state.raw_id is None
            and state.class_id == class_id
            and state.appearance.prototypes
            and state.created_at < born
        )

    def expire(self, now: float) -> None:
        for stable_id, state in list(self._states.items()):
            if stable_id == self._pinned_id:
                continue
            if (
                now - state.kinematics.last_seen
                <= float(self._policy.max_lost_seconds)
            ):
                continue
            if state.raw_id is not None:
                self._raw_to_stable.pop(state.raw_id, None)
            del self._states[stable_id]
        if len(self._states) <= int(self._policy.max_identities):
            return
        lost = sorted(
            (
                state
                for state in self._states.values()
                if state.raw_id is None
                and state.stable_id != self._pinned_id
            ),
            key=lambda state: state.kinematics.last_seen,
        )
        overflow = len(self._states) - int(self._policy.max_identities)
        for state in lost[:overflow]:
            del self._states[state.stable_id]

    def reset(self) -> None:
        self._next_id = 1
        self._raw_to_stable.clear()
        self._raw_last_seen.clear()
        self._states.clear()
        self._pinned_id = None

    def pin(self, stable_id: int | None) -> None:
        self._pinned_id = stable_id

    def hint_position(
        self,
        stable_id: int,
        nx: float,
        ny: float,
        now: float,
    ) -> None:
        state = self._states.get(stable_id)
        if state is None or state.raw_id is not None:
            return
        state.kinematics.nx = float(nx)
        state.kinematics.ny = float(ny)
        state.kinematics.nvx = 0.0
        state.kinematics.nvy = 0.0
        state.kinematics.last_seen = float(now)

    def diagnostics(
        self,
        counters: dict[str, int],
    ) -> IdentityResolverDiagnostics:
        return IdentityResolverDiagnostics(
            identity_count=len(self._states),
            raw_binding_count=len(self._raw_to_stable),
            raw_seen_count=len(self._raw_last_seen),
            counters=dict(counters),
        )


__all__ = ["IdentityRegistry"]
