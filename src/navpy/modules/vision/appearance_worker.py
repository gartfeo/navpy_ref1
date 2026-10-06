"""Latest-only appearance mailbox and worker-thread ownership."""

from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from typing import Protocol

import numpy as np

from navpy.logger.logger_api import ILogger
from navpy.modules.common.thread_launch import ThreadLaunchGate

_WORKER_WARNING_S = 1.0

AppearanceJob = tuple[np.ndarray, list[Sequence[float]], list[int]]


class AppearanceProcessor(Protocol):
    def compute(
        self,
        frame: np.ndarray,
        boxes: list[Sequence[float]],
        keys: list[int],
    ) -> None: ...


class AppearanceWorker:
    """Own the asynchronous request mailbox and exactly one worker thread."""

    def __init__(
        self,
        processor: AppearanceProcessor,
        logger: ILogger | None,
        on_exit: Callable[[], None],
    ) -> None:
        self._processor = processor
        self._logger = logger
        self._on_exit = on_exit
        self._lock = threading.Lock()
        self._pending: AppearanceJob | None = None
        self._accepting = True
        self._event = threading.Event()
        self._launch = ThreadLaunchGate()
        self._thread = threading.Thread(
            target=self._run_generation,
            daemon=True,
        )

    def start(self) -> None:
        try:
            self._thread.start()
        except BaseException:
            with self._lock:
                self._accepting = False
                self._pending = None
            self._event.set()
            self._launch.cancel_before_commit()
            raise

    @property
    def launch_committed(self) -> bool:
        return self._launch.committed

    def submit(
        self,
        frame: np.ndarray,
        boxes: list[Sequence[float]],
        keys: list[int],
    ) -> bool:
        with self._lock:
            if not self._accepting:
                return False
            self._pending = (frame.copy(), boxes, keys)
        self._event.set()
        return True

    def is_current_thread(self) -> bool:
        return self._thread is threading.current_thread()

    def close(self) -> bool:
        with self._lock:
            self._accepting = False
            self._pending = None
        self._event.set()
        if self._launch.cancel_before_commit():
            return True
        if self.is_current_thread():
            return False
        self._thread.join(timeout=_WORKER_WARNING_S)
        if self._thread.is_alive() and self._logger is not None:
            try:
                self._logger.warning(
                    "Appearance worker did not stop within timeout; "
                    "retaining ownership for retry"
                )
            except Exception:
                pass
        return not self._thread.is_alive()

    def _run_generation(self) -> None:
        if not self._launch.enter():
            return
        self._run()

    def _run(self) -> None:
        try:
            self._consume_jobs()
        finally:
            with self._lock:
                self._accepting = False
                self._pending = None
            self._on_exit()

    def _consume_jobs(self) -> None:
        while True:
            self._event.wait(timeout=0.5)
            with self._lock:
                if not self._accepting:
                    self._pending = None
                    return
                job = self._pending
                self._pending = None
                self._event.clear()
            if job is not None:
                self._processor.compute(*job)


__all__ = ["AppearanceWorker"]
