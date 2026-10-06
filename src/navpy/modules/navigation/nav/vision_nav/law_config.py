"""Final-approach-law configuration records and providers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class FinalApproachLawConfig:
    pitch_min_deg: float
    pitch_max_deg: float
    roll_limit_deg: float
    pitch_time_constant_s: float | None
    throttle: float | None


class FinalApproachLawConfigProvider(Protocol):
    def read(self) -> FinalApproachLawConfig | None: ...


class FixedFinalApproachLawConfigProvider:
    def __init__(self, config: FinalApproachLawConfig | None) -> None:
        self._config = config

    def read(self) -> FinalApproachLawConfig | None:
        return self._config


__all__ = [
    "FixedFinalApproachLawConfigProvider",
    "FinalApproachLawConfig",
    "FinalApproachLawConfigProvider",
]
