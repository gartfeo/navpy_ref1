"""Lifecycle fence for terminal command diagnostics."""

from __future__ import annotations

import threading
import time


POSTPROCESS_DRAIN_TIMEOUT_S = 2.0


class TerminalPostprocessFence:
    """Let phase reset wait for diagnostics already owned by the worker."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._in_flight = 0

    def begin(self) -> None:
        with self._condition:
            self._in_flight += 1

    def finish(self) -> None:
        with self._condition:
            if self._in_flight <= 0:
                raise RuntimeError("terminal postprocess fence finished twice")
            self._in_flight -= 1
            if self._in_flight == 0:
                self._condition.notify_all()

    def drain(self) -> None:
        deadline_s = time.monotonic() + POSTPROCESS_DRAIN_TIMEOUT_S
        with self._condition:
            while self._in_flight:
                remaining_s = deadline_s - time.monotonic()
                if remaining_s <= 0.0:
                    raise TimeoutError(
                        "terminal postprocess did not drain before reset"
                    )
                self._condition.wait(timeout=remaining_s)


__all__ = ["POSTPROCESS_DRAIN_TIMEOUT_S", "TerminalPostprocessFence"]
