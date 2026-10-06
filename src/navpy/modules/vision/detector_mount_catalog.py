"""Immutable catalog of the mounts owned by a detector fleet."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Sequence

if TYPE_CHECKING:
    from navpy.modules.vision.camera_mount import CameraMount


class MountMember(Protocol):
    @property
    def mounts(self) -> list["CameraMount"]: ...


class DetectorMountCatalog:
    def __init__(self, members: Sequence[MountMember]) -> None:
        self._mounts = tuple(
            mount for member in members for mount in member.mounts
        )

    @property
    def mounts(self) -> list["CameraMount"]:
        return list(self._mounts)


__all__ = ["DetectorMountCatalog"]
