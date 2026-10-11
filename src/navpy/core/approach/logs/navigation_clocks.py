"""Clock-domain adapters for navigation-log cadence and diagnostics."""

from __future__ import annotations

import math
from datetime import datetime
from typing import Callable, Optional


def current_wall_timestamp() -> str:
    """Format an unscaled wall-clock timestamp for CSV rows."""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def minimum_wall_interval_s(
    cadence_interval: Optional[Callable[[float], float]],
    nominal_interval_s: float,
) -> float:
    """Adapt scheduler cadence without scaling the wall clock itself."""
    if cadence_interval is None:
        return nominal_interval_s
    try:
        value = float(cadence_interval(nominal_interval_s))
    except Exception:
        return nominal_interval_s
    if not math.isfinite(value) or value <= 0.0:
        return nominal_interval_s
    return value


def current_source_time_s(
    time_source: Optional[Callable[[], float]],
) -> Optional[float]:
    """Read a finite diagnostic source timestamp, or leave it absent."""
    if time_source is None:
        return None
    try:
        value = float(time_source())
    except Exception:
        return None
    return value if math.isfinite(value) else None


def primary_log_gate(
    *,
    rate_gate: Optional[bool],
    last_log_time: float,
    current_time: Callable[[], float],
    minimum_interval: Callable[[], float],
) -> tuple[bool, float]:
    """Resolve the primary-row gate without changing either clock domain."""
    if rate_gate is not None and not rate_gate:
        return False, last_log_time
    now = current_time()
    if rate_gate is None and now - last_log_time < minimum_interval():
        return False, last_log_time
    return True, now
