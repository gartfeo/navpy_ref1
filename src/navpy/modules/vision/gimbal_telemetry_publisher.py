"""Periodic lifecycle for camera-mount MAVLink telemetry."""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Iterable

from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.vision.gimbal_telemetry_messages import (
    GimbalMavlinkPublisher,
    GimbalTelemetryEmitter,
    GimbalTelemetryLogger,
)
from navpy.modules.vision.worker_failure import WorkerFailureLatch

if TYPE_CHECKING:
    from navpy.modules.vision.vision_profile_types import CameraMountSpec


class GimbalTelemetryPublisher:
    """Own one retryable periodic worker around a focused message emitter."""

    def __init__(
        self,
        vehicle: GimbalMavlinkPublisher,
        mount_specs: Iterable["CameraMountSpec"],
        logger: GimbalTelemetryLogger,
        *,
        rate_hz: float = 5.0,
    ) -> None:
        self._emitter = GimbalTelemetryEmitter(vehicle, mount_specs, logger)
        self._logger = logger
        self._period_s = 1.0 / max(float(rate_hz), 0.1)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._lifecycle_lock = threading.Lock()
        self._failures = WorkerFailureLatch()

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._thread is not None:
                if (
                    self._launch is not None
                    and not self._launch.committed
                ) or self._thread.is_alive():
                    return
                self._thread = None
                self._launch = None
            stop_event = threading.Event()
            launch = ThreadLaunchGate()
            thread = threading.Thread(
                target=self._run_generation,
                args=(stop_event, launch),
                name="gimbal-telemetry-publisher",
                daemon=True,
            )
            self._stop = stop_event
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

    def stop(self) -> bool:
        with self._lifecycle_lock:
            self._stop.set()
            thread = self._thread
            launch = self._launch
            if (
                thread is not None
                and launch is not None
                and launch.cancel_before_commit()
            ):
                self._thread = None
                self._launch = None
                return True
        if thread is not None:
            if thread is threading.current_thread():
                return False
            if thread.is_alive():
                thread.join(timeout=2.0)
            if thread.is_alive():
                self._logger.warning(
                    "Gimbal telemetry publisher did not stop within 2.0 seconds"
                )
                return False
        with self._lifecycle_lock:
            if self._thread is thread:
                self._thread = None
                self._launch = None
        return True

    @property
    def is_quiescent(self) -> bool:
        thread = self._thread
        if thread is None:
            return True
        launch = self._launch
        if launch is not None and not launch.committed:
            return False
        try:
            return not thread.is_alive()
        except BaseException:
            return False

    def publish_once(self) -> None:
        self._emitter.publish_once()

    def raise_if_failed(self) -> None:
        self._failures.raise_if_failed()

    def _run_generation(
        self,
        stop_event: threading.Event,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        self._failures.run(lambda: self._run(stop_event))

    def _run(self, stop_event: threading.Event | None = None) -> None:
        stop_event = self._stop if stop_event is None else stop_event
        while not stop_event.is_set():
            self.publish_once()
            stop_event.wait(self._period_s)


__all__ = [
    "GimbalMavlinkPublisher",
    "GimbalTelemetryLogger",
    "GimbalTelemetryPublisher",
]
