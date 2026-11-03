"""Recent gimbal attitude readback for capture-time brackets (HITL step 11).

Each sample is one accepted readback, keyed by the connection session it
arrived in and the driver's sequence number, stamped on the capture clock
(``rt.perf_counter``) when it was received. A session ends on reconnect and
its samples go with it.
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
class GimbalCaptureHistory:
    """The current session's samples, oldest first, and the newest full readback."""

    session: int
    samples: tuple[GimbalHistorySample, ...]
    data: GimbalData


__all__ = ["GimbalCaptureHistory", "GimbalHistorySample"]
