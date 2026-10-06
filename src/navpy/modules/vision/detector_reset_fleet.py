"""Coordinated detector and target-identity reset."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Sequence

if TYPE_CHECKING:
    from navpy.modules.vision.detection_identity_registry import DetectionIdentityRegistry


class ResetMember(Protocol):
    def refresh(self) -> None: ...


class DetectorResetFleet:
    def __init__(
        self,
        members: Sequence[ResetMember],
        identities: "DetectionIdentityRegistry",
    ) -> None:
        self._members = tuple(members)
        self._identities = identities

    def refresh(self) -> None:
        self._identities.reset()
        for member in self._members:
            member.refresh()


__all__ = ["DetectorResetFleet"]
