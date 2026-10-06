"""Bounded worker ownership for SIYI hardware telemetry polling."""

from __future__ import annotations

import threading
from dataclasses import dataclass

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.vision.peripheral.siyi.hardware.polling import (
    SiyiTelemetryPoller,
)
from navpy.modules.vision.peripheral.siyi.hardware.shutdown import (
    stop_poll_thread,
)
from navpy.modules.vision.worker_failure import WorkerFailureLatch


@dataclass
class SiyiPollOwnership:
    stop_event: threading.Event | None = None
    thread: threading.Thread | None = None
    launch: ThreadLaunchGate | None = None


class SiyiPollWorker:
    """Own one launch-gated poll thread until bounded retirement succeeds."""

    def __init__(
        self,
        poller: SiyiTelemetryPoller,
        logger: ILogger,
    ) -> None:
        self._poller = poller
        self._logger = logger
        self._ownership = SiyiPollOwnership()
        self._failures = WorkerFailureLatch()

    @property
    def ownership(self) -> SiyiPollOwnership:
        return self._ownership

    @property
    def has_thread(self) -> bool:
        return self._ownership.thread is not None

    def start(self) -> None:
        if self.has_thread:
            raise RuntimeError("SIYI polling worker is already owned")
        stop_event = threading.Event()
        launch = ThreadLaunchGate()
        thread = threading.Thread(
            target=self._run,
            args=(stop_event, launch),
            daemon=True,
            name="siyi-hardware-poll",
        )
        self._ownership = SiyiPollOwnership(stop_event, thread, launch)
        try:
            thread.start()
        except BaseException:
            stop_event.set()
            if launch.cancel_before_commit():
                self._ownership = SiyiPollOwnership()
            raise

    def stop(self, timeout_s: float) -> bool:
        owned = self._ownership
        result = stop_poll_thread(
            owned.thread,
            owned.stop_event,
            owned.launch,
            timeout_s,
            self._logger,
        )
        if result.stopped:
            self._ownership = SiyiPollOwnership()
        return result.stopped

    def raise_if_failed(self) -> None:
        self._failures.raise_if_failed()

    def _run(
        self,
        stop_event: threading.Event,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        self._failures.run(
            lambda: self._poller.run(stop_event),
            stop_event.set,
        )


__all__ = ["SiyiPollOwnership", "SiyiPollWorker"]
