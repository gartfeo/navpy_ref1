"""Terminal-law configuration records and providers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class TerminalLawConfig:
    pitch_min_deg: float
    pitch_max_deg: float
    roll_limit_deg: float
    pitch_time_constant_s: float | None
    throttle: float | None


class TerminalLawConfigProvider(Protocol):
    def read(self) -> TerminalLawConfig | None: ...


class FixedTerminalLawConfigProvider:
    def __init__(self, config: TerminalLawConfig | None) -> None:
        self._config = config

    def read(self) -> TerminalLawConfig | None:
        return self._config


__all__ = [
    "FixedTerminalLawConfigProvider",
    "TerminalLawConfig",
    "TerminalLawConfigProvider",
]
