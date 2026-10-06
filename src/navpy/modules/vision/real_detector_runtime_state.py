"""Synchronization, metrics, run state, and loop timing."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from types import TracebackType

from navpy.modules.vision.real_detector_ports import MonotonicClock, SleepAction


class PipelineMutationGate:
    """Serialize one tracker update with a whole-pipeline reset."""

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def __enter__(self) -> PipelineMutationGate:
        self._lock.acquire()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc_value, traceback
        self._lock.release()


class RuntimeMetrics:
    """Thread-safe counters plus the tracking-loop FPS estimate."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._counters = {
            "detect_batches": 0,
            "track_ticks": 0,
            "track_updates": 0,
            "coast_ticks": 0,
            "embeddings": 0,
            "deep_search_runs": 0,
            "bridge_hits": 0,
        }
        self._fps_estimate = 0.0
        self._fps_count = 0
        self._fps_started_at = time.monotonic()

    @property
    def tracking_fps(self) -> float:
        with self._lock:
            return self._fps_estimate

    def bump(self, key: str, amount: int = 1) -> None:
        with self._lock:
            self._counters[key] = self._counters.get(key, 0) + amount

    def update_fps(self) -> None:
        with self._lock:
            self._fps_count += 1
            now_s = time.monotonic()
            elapsed_s = now_s - self._fps_started_at
            if elapsed_s >= 1.0:
                self._fps_estimate = self._fps_count / max(1e-6, elapsed_s)
                self._fps_count = 0
                self._fps_started_at = now_s

    def snapshot(self) -> dict[str, float]:
        with self._lock:
            result = dict(self._counters)
            result["tracking_fps"] = self._fps_estimate
        return result


class DetectorRunState:
    """Thread-safe running signal plus irreversible resource-stop state."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._running = threading.Event()
        self._stopped = False

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    @property
    def is_stopped(self) -> bool:
        with self._lock:
            return self._stopped

    def begin(self) -> bool:
        with self._lock:
            if self._running.is_set():
                return False
            if self._stopped:
                raise RuntimeError(
                    "Detector cannot be restarted after stop() released its GPU "
                    "resources; construct a new Detector instead."
                )
            self._running.set()
            return True

    def request_stop(self) -> None:
        with self._lock:
            self._running.clear()

    def mark_resources_stopped(self) -> None:
        with self._lock:
            self._stopped = True


@dataclass(frozen=True)
class LoopTiming:
    monotonic: MonotonicClock = time.monotonic
    sleep: SleepAction = time.sleep


def next_loop_deadline(previous_deadline: float, period: float, now: float) -> float:
    """Advance to the first future deadline without burst catch-up."""
    bounded_period = max(1e-6, float(period))
    next_deadline = float(previous_deadline) + bounded_period
    while next_deadline <= now:
        next_deadline += bounded_period
    return next_deadline


__all__ = [
    "DetectorRunState",
    "LoopTiming",
    "PipelineMutationGate",
    "RuntimeMetrics",
    "next_loop_deadline",
]
