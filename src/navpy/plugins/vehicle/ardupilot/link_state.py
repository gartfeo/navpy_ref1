"""Thread-safe heartbeat and MAVLink packet-quality state."""
from __future__ import annotations

import threading
import time
from collections.abc import Callable

from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAV_TYPE_GCS,
    MAV_TYPE_ONBOARD_CONTROLLER,
)


class HeartbeatState:
    def __init__(
        self,
        target_system: int,
        timeout_s: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._target_system = target_system
        self._timeout_s = timeout_s
        self._clock = clock
        self._lock = threading.Lock()
        self._last_receipt_s = clock()
        self._message = None

    def observe(
        self,
        message: MAVLink_message,
        receipt_time_s: float | None = None,
    ) -> None:
        heartbeat_type = getattr(message, "type", None)
        if heartbeat_type in (MAV_TYPE_GCS, MAV_TYPE_ONBOARD_CONTROLLER):
            return
        with self._lock:
            self._last_receipt_s = (
                self._clock() if receipt_time_s is None else receipt_time_s
            )
            self._message = message

    def touch(self, receipt_time_s: float | None = None) -> None:
        with self._lock:
            self._last_receipt_s = (
                self._clock() if receipt_time_s is None else receipt_time_s
            )

    @property
    def message(self) -> MAVLink_message | None:
        with self._lock:
            return self._message

    @property
    def age_s(self) -> float:
        with self._lock:
            return self._clock() - self._last_receipt_s

    @property
    def link_ok(self) -> bool:
        return self.age_s <= self._timeout_s


class PacketLossTracker:
    def __init__(
        self,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._last_sequence: dict[int, int] = {}
        self._lost = 0.0
        self._received = 1.0
        self._decay_time_s = clock()

    def observe(self, component_id: int, sequence: int) -> None:
        with self._lock:
            previous = self._last_sequence.get(component_id)
            if previous is not None:
                expected = (previous + 1) % 256
                if sequence not in (expected, previous):
                    self._lost += (
                        256 - expected + sequence
                        if sequence < expected
                        else sequence - expected
                    )
            self._received += 1.0
            self._last_sequence[component_id] = sequence
            now = self._clock()
            if now - self._decay_time_s >= 5.0:
                self._decay_time_s = now
                self._lost *= 0.8
                self._received *= 0.8

    @property
    def quality(self) -> int:
        with self._lock:
            total = self._received + self._lost
            return 100 if total <= 0 else min(100, int(self._received / total * 100))
