"""Deterministic FIFO worker used by CacheLogger."""

from __future__ import annotations

import logging
import queue
import threading
from collections.abc import Callable

from navpy.logger.cache_log_message import LogMessage
from navpy.modules.common.thread_launch import ThreadLaunchGate

_diag_log = logging.getLogger(__name__)
_STOP = object()
CACHE_WORKER_JOIN_TIMEOUT_S = 2.0


class CacheLogWorker:
    """Accept messages until close, then drain FIFO and stop exactly once."""

    def __init__(
        self,
        handler: Callable[[LogMessage], None],
        *,
        on_error: Callable[[BaseException], None] | None = None,
    ) -> None:
        self._handler = handler
        self._on_error = on_error or self._log_error
        self._queue: queue.Queue[object] = queue.Queue()
        self._accept_lock = threading.Lock()
        self._accepting = True
        self._launch = ThreadLaunchGate()
        self._thread = threading.Thread(
            target=self._run_generation,
            name="CacheLoggerWorker",
            daemon=True,
        )
        try:
            self._thread.start()
        except BaseException:
            with self._accept_lock:
                self._accepting = False
                self._queue.put(_STOP)
            self._launch.cancel_before_commit()
            raise

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def submit(self, message: LogMessage) -> bool:
        with self._accept_lock:
            if not self._accepting:
                return False
            self._queue.put(message)
            return True

    def wait_until_idle(self) -> None:
        self._queue.join()

    def close(self) -> None:
        with self._accept_lock:
            if self._accepting:
                self._accepting = False
                self._queue.put(_STOP)
        if self._launch.cancel_before_commit():
            return
        if threading.current_thread() is self._thread:
            return
        self._thread.join(timeout=CACHE_WORKER_JOIN_TIMEOUT_S)
        if self._thread.is_alive():
            raise TimeoutError(
                "cache log worker did not stop; output remains owned"
            )

    def _run_generation(self) -> None:
        if not self._launch.enter():
            return
        self._run()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    return
                try:
                    self._handler(item)  # type: ignore[arg-type]
                except Exception as exc:
                    self._on_error(exc)
            finally:
                self._queue.task_done()

    @staticmethod
    def _log_error(exc: BaseException) -> None:
        _diag_log.error(
            "Suppressed error while handling a log message",
            exc_info=(type(exc), exc, exc.__traceback__),
        )


__all__ = ["CacheLogWorker"]
