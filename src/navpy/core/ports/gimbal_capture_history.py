"""Recent gimbal attitude readback for capture-time brackets (HITL step 11).

Each sample is one accepted readback, keyed by the connection session it
arrived in and the driver's sequence number, stamped on the capture clock
(``rt.perf_counter``) when it was received. A session ends on reconnect and
its samples go with it.

Zoom readings and zoom commands are kept the same way (HITL step 18): a
reading is one accepted zoom readback, a command one zoom write stamped just
before it was sent (an absolute zoom command in tenths, or None for a manual
zoom start or stop, which leaves the lens set by no known setting).
"""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.kernel.attitude import Attitude
from navpy.core.ports.gimbal_abc import GimbalData


@dataclass(frozen=True)
class GimbalHistorySample:
    session: int
    seq: int
    receipt_s: float
    att: Attitude


@dataclass(frozen=True)
class ZoomReadingSample:
    session: int
    seq: int
    receipt_s: float
    reading: float


@dataclass(frozen=True)
class ZoomCommandSample:
    session: int
    sent_s: float
    absolute_tenths: int | None


@dataclass(frozen=True)
class GimbalCaptureHistory:
    """The current session's samples, oldest first, and the newest full readback."""

    session: int
    samples: tuple[GimbalHistorySample, ...]
    data: GimbalData
    zoom_readings: tuple[ZoomReadingSample, ...] = ()
    zoom_commands: tuple[ZoomCommandSample, ...] = ()


__all__ = ["GimbalCaptureHistory", "GimbalHistorySample", "ZoomCommandSample", "ZoomReadingSample"]
