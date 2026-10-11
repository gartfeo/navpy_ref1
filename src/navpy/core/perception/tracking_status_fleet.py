"""Aggregate detector tracking status without owning commands."""

from __future__ import annotations

from typing import Protocol, Sequence


class TrackingStatusMember(Protocol):
    @property
    def is_detection_armed(self) -> bool: ...

    @property
    def loss_hold_sec(self) -> float | None: ...


class TrackingStatusFleet:
    def __init__(
        self,
        members: Sequence[TrackingStatusMember],
        default_loss_hold_sec: float,
    ) -> None:
        self._members = tuple(members)
        self._default_loss_hold_sec = default_loss_hold_sec

    @property
    def is_detection_armed(self) -> bool:
        return any(member.is_detection_armed for member in self._members)

    @property
    def loss_hold_sec(self) -> float:
        values = [
            value
            for member in self._members
            if (value := member.loss_hold_sec) is not None
        ]
        return max(values) if values else self._default_loss_hold_sec


__all__ = ["TrackingStatusFleet"]
