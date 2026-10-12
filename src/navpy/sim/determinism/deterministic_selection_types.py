"""Immutable inputs and decisions for the unwired source-time selector.

Payload is opaque, lossless sample identity, never receipt time. Production
stream codecs and boot detection are adapter work, not supplied by these types.
SourceSample is a transport record: offer() validates its values and latches
bad input rather than allowing a construction error to evade the final verdict.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal


class SelectionStream(Enum):
    ATTITUDE = "attitude"
    TRUTH = "truth"
    AIRSPEED = "airspeed"


def exact_integer(value: object, minimum: int = 0) -> bool:
    return type(value) is int and value >= minimum


@dataclass(frozen=True)
class SlotKey:
    boot_epoch: int
    index: int

    def __post_init__(self) -> None:
        if not exact_integer(self.boot_epoch) or not exact_integer(self.index):
            raise ValueError("slot key needs nonnegative exact integers")


@dataclass(frozen=True)
class SourceSample:
    boot_epoch: int
    time_us: int
    payload: bytes


@dataclass(frozen=True)
class SelectionConfig:
    boot_epoch: int
    period_us: int
    start_slot: int
    max_truth_gap_us: int
    capacity_per_stream: int
    end_slot_exclusive: int

    def __post_init__(self) -> None:
        for name, minimum in (
            ("boot_epoch", 0), ("period_us", 1), ("start_slot", 1),
            ("max_truth_gap_us", 1), ("capacity_per_stream", 2),
            ("end_slot_exclusive", 1),
        ):
            if not exact_integer(getattr(self, name), minimum):
                raise ValueError(f"{name} must be an exact integer >= {minimum}")
        if self.end_slot_exclusive <= self.start_slot:
            raise ValueError("end_slot_exclusive must follow start_slot")


@dataclass(frozen=True)
class ClosingSamples:
    """First source samples closing the conditions, not poll-time maxima."""

    attitude: SourceSample
    truth: SourceSample | None
    airspeed: SourceSample | None


@dataclass(frozen=True)
class SelectionDecision:
    slot: SlotKey
    outcome: Literal["fresh", "empty"]
    attitude: SourceSample | None
    decimated: tuple[SourceSample, ...]
    truth: tuple[SourceSample, SourceSample] | None
    airspeed: SourceSample | None
    closing: ClosingSamples


@dataclass(frozen=True)
class SelectionViolation:
    reason: str
    stream: SelectionStream | None
    slot: SlotKey
    source_us: int | None


@dataclass(frozen=True)
class SelectionVerdict:
    boot_epoch: int
    start_slot: int
    end_slot_exclusive: int
    violation: SelectionViolation | None

    @property
    def valid(self) -> bool:
        return self.violation is None
