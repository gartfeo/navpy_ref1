"""Periodic-flush daemon lifecycle for the pose-cadence recorder.

Extracted from ``pose_cadence_debug`` so each module is one concern: the
recorder owns the buffers and the CSV ``dump``; :class:`PoseCadenceFlusher`
owns only the timer thread that calls that dump on a fixed cadence, plus the
machinery to start, stop, and join a single generation. The flusher takes the
dump as a plain callback, so it never imports the recorder back.
"""
from __future__ import annotations

import atexit
import threading
from typing import Callable

# Bounds a stop() join so a wedged flusher surfaces as a failure instead of a
# silent "clean stop".
_FLUSH_JOIN_TIMEOUT_S = 2.0


class PoseCadenceFlusher:
    """Own one restartable periodic-flush daemon generation.

    The daemon waits on the generation's OWN stop event (never a shared module
    global that an ``importlib.reload`` would replace out from under a live
    thread), so ``Event.wait`` keeps the flush cadence while staying promptly
    joinable and -- unlike ``time.sleep`` -- immune to a test that monkeypatches
    the shared stdlib sleep.

    Production never calls :meth:`stop`: the daemon runs to process exit and the
    atexit hook drains the tail. :meth:`stop` is the test seam that lets a
    reload-based test join each generation before the next reload orphans it.
    """

    def __init__(self, flush: Callable[[], None], interval_s: float) -> None:
        self._flush = flush
        self._interval_s = interval_s
        self._thread: threading.Thread | None = None
        self._stop: threading.Event | None = None
        self._atexit: Callable[[], None] | None = None
        self._started = False

    def start(self) -> None:
        """Launch the daemon once. A second call while running is a no-op.

        Ownership is recorded (and the atexit tail-drain registered) only for a
        generation that actually launched:
          * a failure BEFORE launch (e.g. the OS cannot create a thread) records
            nothing, so stop() stays a no-op and a later start() can retry;
          * a failure AFTER launch (an async KeyboardInterrupt / SystemExit that
            interrupts start() once the daemon is already running) adopts the
            live generation, so stop() can still terminate and join it instead
            of orphaning it.
        Either way the exception propagates -- a start failure is never hidden
        behind a silently-absent diagnostic.
        """
        if self._started:
            return
        stop = threading.Event()
        thread = threading.Thread(
            target=self._loop, args=(stop,), name="pose-cadence-flush",
            daemon=True,
        )
        try:
            thread.start()
        except BaseException:
            if thread.is_alive():
                # Visibly launched: adopt it so stop() can terminate and join it.
                self._adopt(thread, stop)
            else:
                # Not visibly alive: either it never launched (harmless) or its
                # bootstrap is merely delayed. Signal stop unconditionally so a
                # delayed-bootstrap generation exits the moment it reaches the
                # loop instead of running as an unowned daemon.
                stop.set()
            raise
        self._adopt(thread, stop)

    def _adopt(self, thread: threading.Thread, stop: threading.Event) -> None:
        """Take ownership of a launched generation and arm the atexit drain."""
        self._thread = thread
        self._stop = stop
        self._atexit = self._flush
        self._started = True
        atexit.register(self._atexit)

    def stop(self) -> None:
        """Stop and join the daemon if running. Idempotent; does NOT flush.

        A join that times out is raised, not reported as a clean stop, and the
        live handle is kept for a retry.
        """
        thread = self._thread
        stop = self._stop
        atexit_cb = self._atexit
        if stop is not None:
            stop.set()
        if thread is not None:
            thread.join(timeout=_FLUSH_JOIN_TIMEOUT_S)
            if thread.is_alive():
                raise RuntimeError(
                    "pose-cadence flusher did not stop within "
                    f"{_FLUSH_JOIN_TIMEOUT_S:.1f}s"
                )
        if atexit_cb is not None:
            atexit.unregister(atexit_cb)
        self._thread = None
        self._stop = None
        self._atexit = None
        self._started = False

    def _loop(self, stop: threading.Event) -> None:
        while not stop.wait(self._interval_s):
            self._flush()


__all__ = ["PoseCadenceFlusher"]
