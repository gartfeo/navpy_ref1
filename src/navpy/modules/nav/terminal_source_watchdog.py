"""Speedup-aware wall-receipt watchdog for terminal source sessions."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable


class TerminalSourceReceiptWatchdog:
    """Track source silence without scaling or rewriting producer time."""

    def __init__(
        self,
        wall_s: Callable[[], float],
        maximum_wall_age_s: Callable[[], float],
    ) -> None:
        self._wall_s = wall_s
        self._maximum_wall_age_s = maximum_wall_age_s
        self._lock = threading.Lock()
        self._armed_at_wall_s: float | None = None

    def arm(self) -> None:
        try:
            armed_at_wall_s = float(self._wall_s())
        except Exception:  # noqa: BLE001 - external clock fails closed
            armed_at_wall_s = math.nan
        with self._lock:
            self._armed_at_wall_s = armed_at_wall_s

    def disarm(self) -> None:
        with self._lock:
            self._armed_at_wall_s = None

    def expired(self) -> bool:
        with self._lock:
            armed_at_wall_s = self._armed_at_wall_s
        if armed_at_wall_s is None:
            return False
        try:
            now_wall_s = float(self._wall_s())
            maximum_wall_age_s = float(self._maximum_wall_age_s())
        except Exception:  # noqa: BLE001 - cadence/clock failure fails closed
            return True
        age_s = now_wall_s - armed_at_wall_s
        return not (
            math.isfinite(armed_at_wall_s)
            and math.isfinite(now_wall_s)
            and math.isfinite(age_s)
            and math.isfinite(maximum_wall_age_s)
            and maximum_wall_age_s > 0.0
            and 0.0 <= age_s <= maximum_wall_age_s
        )


__all__ = ["TerminalSourceReceiptWatchdog"]
