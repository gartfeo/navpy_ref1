"""The latest ATTITUDE boot time: the aircraft's own clock, for evidence rows and the HITL contact's freshness only.

Never a navigation or command input.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.vehicle_contract import IVehicle


def attitude_boot_s(vehicle: IVehicle) -> Optional[float]:
    """Latest ATTITUDE boot time (the observation clock); None when there is none."""
    sample = getattr(vehicle, "attitude_sample", None)
    value = getattr(sample, "time_boot_s", None)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def fresh_attitude_boot_s(vehicle: IVehicle, max_age_s: float,
                          now_s: Optional[Callable[[], float]] = None) -> Optional[float]:
    """The latest ATTITUDE boot time while that sample was received at most ``max_age_s`` ago; None for a stale,
    unstamped or missing one. Age is on the monotonic capture clock (``capture_receipt_s``, ``rt.perf_counter``,
    as the inbound router stamps it), never the wall clock, which can step back; a receipt ahead of now is
    refused too."""
    sample = getattr(vehicle, "attitude_sample", None)
    receipt = getattr(sample, "capture_receipt_s", None)
    if isinstance(receipt, bool) or not isinstance(receipt, (int, float)) or not math.isfinite(receipt):
        return None
    age_s = (rt.perf_counter() if now_s is None else now_s()) - receipt
    if not 0.0 <= age_s <= max_age_s:
        return None
    return attitude_boot_s(vehicle)


__all__ = ["attitude_boot_s", "fresh_attitude_boot_s"]
