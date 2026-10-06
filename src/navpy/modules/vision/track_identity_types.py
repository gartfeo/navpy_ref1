"""Policies, state, and diagnostics for stable track identities."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class IdentityRetentionPolicy:
    max_lost_seconds: float = 20.0
    grace_seconds: float = 0.6
    merge_window: float = 3.0
    max_identities: int = 512


@dataclass(frozen=True)
class IdentityGeometryPolicy:
    max_center_distance: float = 0.12
    motion_gate_growth: float = 0.18
    motion_gate_cap: float = 0.40
    predict_horizon: float = 0.6
    max_size_error: float = 1.6
    ambiguity_margin: float = 0.12


@dataclass(frozen=True)
class IdentityAppearancePolicy:
    threshold: float = 0.5
    margin: float = 0.1
    swap_threshold: float = 0.75
    swap_patience: int = 5
    max_prototypes: int = 5

    @property
    def strong_threshold(self) -> float:
        return float(self.threshold) * 0.6

    @property
    def prototype_match_threshold(self) -> float:
        return float(self.threshold) * 0.7

    @property
    def continuity_threshold(self) -> float:
        return float(self.threshold) * 0.6

    @property
    def contradiction_threshold(self) -> float:
        return min(0.95, float(self.threshold) * 1.8)


@dataclass(frozen=True)
class TrackIdentityPolicies:
    retention: IdentityRetentionPolicy = field(
        default_factory=IdentityRetentionPolicy
    )
    geometry: IdentityGeometryPolicy = field(
        default_factory=IdentityGeometryPolicy
    )
    appearance: IdentityAppearancePolicy = field(
        default_factory=IdentityAppearancePolicy
    )


@dataclass
class IdentityKinematics:
    nx: float
    ny: float
    nw: float
    nh: float
    nvx: float
    nvy: float
    last_seen: float


@dataclass
class IdentityAppearanceMemory:
    prototypes: list[np.ndarray] = field(default_factory=list)
    last_embedding: np.ndarray | None = None
    swap_mismatch_count: int = 0


@dataclass
class IdentityState:
    stable_id: int
    raw_id: int | None
    class_id: int
    kinematics: IdentityKinematics
    appearance: IdentityAppearanceMemory = field(
        default_factory=IdentityAppearanceMemory
    )
    created_at: float = 0.0


@dataclass(frozen=True)
class IdentityResolverDiagnostics:
    identity_count: int
    raw_binding_count: int
    raw_seen_count: int
    counters: dict[str, int]


class IdentityCounters:
    """Shared decision counters surfaced through detector diagnostics."""

    def __init__(self) -> None:
        self._values: dict[str, int] = {
            "forks": 0,
            "rebinds_appearance": 0,
            "rebinds_geometry": 0,
            "late_merges": 0,
            "swaps": 0,
            "rej_size": 0,
            "rej_center": 0,
            "rej_contradict": 0,
        }

    @property
    def values(self) -> dict[str, int]:
        return self._values

    def bump(self, key: str) -> None:
        self._values[key] += 1

    def snapshot(self) -> dict[str, int]:
        return dict(self._values)


__all__ = [
    "IdentityAppearanceMemory",
    "IdentityAppearancePolicy",
    "IdentityCounters",
    "IdentityGeometryPolicy",
    "IdentityKinematics",
    "IdentityResolverDiagnostics",
    "IdentityRetentionPolicy",
    "IdentityState",
    "TrackIdentityPolicies",
]
