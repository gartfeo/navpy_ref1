"""Small read-only terminal status owner."""

from __future__ import annotations

import math
import threading

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame


class TerminalRuntimeStatus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_body_bearing_deg: float | None = None

    def record(self, frame: TerminalVisionFrame) -> None:
        with self._lock:
            self._last_body_bearing_deg = math.degrees(
                math.atan2(frame.body_y, frame.body_x)
            )

    def reset(self) -> None:
        with self._lock:
            self._last_body_bearing_deg = None

    def last_measured_lateral_bearing_deg(self) -> float | None:
        with self._lock:
            return self._last_body_bearing_deg


__all__ = ["TerminalRuntimeStatus"]
