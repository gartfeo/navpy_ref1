"""Guidance-law configuration records and providers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class GuidanceLawConfig:
    pitch_min_deg: float
    pitch_max_deg: float
    roll_limit_deg: float
    pitch_time_constant_s: float | None
    throttle: float | None


class GuidanceLawConfigProvider(Protocol):
    def read(self) -> GuidanceLawConfig | None: ...


class FixedGuidanceLawConfigProvider:
    def __init__(self, config: GuidanceLawConfig | None) -> None:
        self._config = config

    def read(self) -> GuidanceLawConfig | None:
        return self._config


__all__ = [
    "FixedGuidanceLawConfigProvider",
    "GuidanceLawConfig",
    "GuidanceLawConfigProvider",
]
