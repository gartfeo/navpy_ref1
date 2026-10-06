"""Narrow primitive ports for terminal source-time diagnostics."""

from __future__ import annotations

from collections.abc import Callable
from typing import Literal, Protocol


SourceNow = Callable[[], float]
ObservationOutcome = Literal[
    "invalid_source",
    "invalid_frame",
    "regression",
    "duplicate",
    "fresh",
]
CommandOutcome = Literal["fresh", "held"]


class TerminalObservationSourceTimeObserver(Protocol):
    """Nonthrowing observation-hop diagnostic boundary."""

    def record_observation(
        self,
        *,
        source_timestamp_s: float | None,
        source_now_s: SourceNow | None,
        outcome: ObservationOutcome,
    ) -> None: ...


class TerminalCommandSourceTimeObserver(Protocol):
    """Nonthrowing issued-command diagnostic boundary."""

    def record_command(
        self,
        *,
        wall_start_s: float,
        source_timestamp_s: float,
        source_now_s: SourceNow | None,
        execution_ms: float,
        outcome: CommandOutcome = "fresh",
    ) -> None: ...


__all__ = [
    "ObservationOutcome",
    "CommandOutcome",
    "SourceNow",
    "TerminalCommandSourceTimeObserver",
    "TerminalObservationSourceTimeObserver",
]
