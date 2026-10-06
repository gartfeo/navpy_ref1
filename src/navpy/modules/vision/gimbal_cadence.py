"""Fixed-step gimbal scheduling with simulation-aware wall cadence."""

from __future__ import annotations

import threading
import time
from typing import Protocol

from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.vision.worker_failure import WorkerFailureLatch


class GimbalStep(Protocol):
    def advance(self) -> None: ...


class SchedulerCadence(Protocol):
    def wall_period_for_scheduler_period(
        self,
        scheduler_period_s: float,
    ) -> float: ...


class FixedCadenceRunner:
    """Call one fixed-physics step per scheduled wall iteration.

    Slow work never causes catch-up steps. ``SchedulerCadence`` may shorten
    only the wall period; it cannot alter the physics step or any timestamp.
    """

    def __init__(
        self,
        step: GimbalStep,
        scheduler_period_s: float,
        cadence: SchedulerCadence | None,
    ) -> None:
        if scheduler_period_s <= 0.0:
            raise ValueError("scheduler_period_s must be > 0")
        self._step = step
        self._scheduler_period_s = float(scheduler_period_s)
        self._cadence = cadence
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lifecycle_lock = threading.Lock()
        self._launch: ThreadLaunchGate | None = None
        self._failures = WorkerFailureLatch()

    @property
    def is_running(self) -> bool:
        with self._lifecycle_lock:
            thread = self._thread
            launch = self._launch
            if thread is None:
                return False
            return bool(
                launch is not None and not launch.committed
            ) or thread.is_alive()

    def start(self) -> bool:
        """Start once; return ``False`` when already running."""
        with self._lifecycle_lock:
            if self._thread is not None:
                if (
                    self._launch is not None
                    and not self._launch.committed
                ) or self._thread.is_alive():
                    return False
                self._thread = None
                self._launch = None
            stop_event = threading.Event()
            launch = ThreadLaunchGate()
            self._stop_event = stop_event
            thread = threading.Thread(
                target=self._run_generation,
                args=(stop_event, launch),
                daemon=True,
            )
            self._thread = thread
            self._launch = launch
            try:
                thread.start()
            except BaseException:
                stop_event.set()
                if launch.cancel_before_commit() and self._thread is thread:
                    self._thread = None
                    self._launch = None
                raise
            return True

    def stop(self) -> bool:
        """Request shutdown and report whether the worker is quiescent."""
        with self._lifecycle_lock:
            thread = self._thread
            stop_event = self._stop_event
            launch = self._launch
            stop_event.set()
            if (
                thread is not None
                and launch is not None
                and launch.cancel_before_commit()
            ):
                if self._thread is thread:
                    self._thread = None
                    self._launch = None
                return True
        if thread is not None:
            if thread is threading.current_thread():
                return False
            if thread.is_alive():
                thread.join(timeout=2.0)
        with self._lifecycle_lock:
            if (
                thread is not None
                and self._thread is thread
                and not thread.is_alive()
            ):
                self._thread = None
                self._launch = None
            current = self._thread
            return current is None or not current.is_alive()

    def raise_if_failed(self) -> None:
        self._failures.raise_if_failed()

    def _run_generation(
        self,
        stop_event: threading.Event,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        self._failures.run(
            lambda: self._run(stop_event),
            stop_event.set,
        )

    def _run(self, stop_event: threading.Event | None = None) -> None:
        stop_event = self._stop_event if stop_event is None else stop_event
        while not stop_event.is_set():
            started_s = time.monotonic()
            self._step.advance()
            elapsed_s = time.monotonic() - started_s
            wall_period_s = self._scheduler_period_s
            if self._cadence is not None:
                wall_period_s = self._cadence.wall_period_for_scheduler_period(
                    self._scheduler_period_s,
                )
            stop_event.wait(max(0.0, wall_period_s - elapsed_s))


__all__ = [
    "FixedCadenceRunner",
    "GimbalStep",
    "SchedulerCadence",
]
