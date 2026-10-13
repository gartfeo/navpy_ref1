"""Deterministic FIFO worker used by CacheLogger."""

from __future__ import annotations

import logging
from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_log_message import LogMessage
from navpy.core.kernel.logging.log_output_latch import OutputLatch
from navpy.core.kernel.thread_launch import ThreadLaunchGate

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
        latch: OutputLatch | None = None,
    ) -> None:
        self._handler = handler
        self._on_error = on_error or self._log_error
        self._queue: rt.QueuePort = rt.Queue()
        self._accept_lock = rt.Lock()
        self._accepting = True
        self._latch = latch or OutputLatch()
        self._stopped = False
        self._launch = ThreadLaunchGate()
        self._thread = rt.Thread(
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

    @property
    def failed(self) -> bool:
        """Latched once the handler raised for any message (the error is still suppressed)."""
        return self._latch.latched

    def submit(self, message: LogMessage) -> bool:
        with self._accept_lock:
            if not self._accepting:
                return False
            self._queue.put(message)
            return True

    def wait_until_idle(self) -> None:
        self._queue.join()

    def close(self, *, drain_unstarted: bool = False) -> None:
        """Stop accepting, then wait (bounded) for the worker to drain the FIFO.

        If the worker never ran it is cancelled and its queued messages are
        dropped, unless ``drain_unstarted``: then the caller handles them
        itself, unbounded (the strict close; wall close keeps the drop).
        """
        with self._accept_lock:
            if self._accepting:
                self._accepting = False
                self._queue.put(_STOP)
        if self._launch.cancel_before_commit():
            if drain_unstarted:
                with self._accept_lock:
                    if not self._stopped:
                        self._drain_until_stop(block=False)
            return
        if rt.current_thread() is self._thread:
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
        self._drain_until_stop(block=True)

    def _drain_until_stop(self, *, block: bool) -> None:
        """Handle messages in FIFO order up to ``_STOP``.

        Without blocking only after close: ``_STOP`` is then already queued.
        """
        while True:
            item = self._queue.get(block=block)
            try:
                if item is _STOP:
                    self._stopped = True
                    return
                try:
                    self._handler(item)  # type: ignore[arg-type]
                except Exception as exc:
                    self._latch.latch()
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
