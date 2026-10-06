"""Exclusive source-publication lease used during terminal NAV."""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum, auto
from typing import Protocol

from navpy.modules.vision.models.detection_publication import DetectionPublication


class DetectionLeaseDispatch(Enum):
    """Outcome of one nonblocking exclusive-publication claim."""

    EMPTY = auto()
    CLOSED = auto()
    ACCEPTED = auto()
    REJECTED = auto()


class DetectionEventLease(Protocol):
    """Waitable current-publication stream with one explicit owner."""

    @property
    def closed(self) -> bool: ...

    def wait_and_dispatch(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> bool | None: ...

    def dispatch_available(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> DetectionLeaseDispatch: ...

    def close(self) -> None: ...


__all__ = ["DetectionEventLease", "DetectionLeaseDispatch"]
