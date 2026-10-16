"""Narrow primitive ports for final-approach source-time diagnostics."""

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
# What became of a fresh frame after the observation hop (frame lineage).
FrameDisposition = Literal["replaced", "consumed", "unissued", "stale"]


class FinalApproachObservationSourceTimeObserver(Protocol):
    """Nonthrowing observation-hop diagnostic boundary."""

    def record_observation(
        self,
        *,
        source_timestamp_s: float | None,
        source_now_s: SourceNow | None,
        outcome: ObservationOutcome,
        epoch: int | None = None,
    ) -> None: ...

    def record_frame_disposition(
        self,
        *,
        stage: str,
        epoch: int,
        source_timestamp_s: float,
        disposition: FrameDisposition,
    ) -> None: ...


class FinalApproachCommandSourceTimeObserver(Protocol):
    """Nonthrowing issued-command diagnostic boundary."""

    def record_command(
        self,
        *,
        wall_start_s: float,
        source_timestamp_s: float,
        source_now_s: SourceNow | None,
        execution_ms: float,
        outcome: CommandOutcome = "fresh",
        work_cpu_ms: float | None = None,
        epoch: int | None = None,
    ) -> None: ...

    def record_frame_disposition(
        self,
        *,
        stage: str,
        epoch: int,
        source_timestamp_s: float,
        disposition: FrameDisposition,
    ) -> None: ...


__all__ = [
    "ObservationOutcome",
    "CommandOutcome",
    "FrameDisposition",
    "SourceNow",
    "FinalApproachCommandSourceTimeObserver",
    "FinalApproachObservationSourceTimeObserver",
]
