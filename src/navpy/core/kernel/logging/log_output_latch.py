"""Whether every log line reached its file: a latch, not new output.

The log worker suppresses handler errors and ``logging.Handler.handleError``
swallows emit errors, so a line can be lost while logging carries on. Both
paths set one :class:`OutputLatch`; wall output is unchanged (the same
suppression, the same stderr report). A caller that must know, such as a
lockstep participant's drain flush, closes with :func:`close_strictly`.
"""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from typing import Protocol


class LogOutputFailed(RuntimeError):
    """A log line failed to reach its output; the file may be missing lines."""


class OutputLatch:
    """Set once by any output failure; never cleared."""

    def __init__(self) -> None:
        self._latched = False

    @property
    def latched(self) -> bool:
        return self._latched

    def latch(self) -> None:
        self._latched = True

    def check(self) -> None:
        if self._latched:
            raise LogOutputFailed("a log line failed to reach its output")


class LatchingRotatingFileHandler(RotatingFileHandler):
    """``RotatingFileHandler`` whose swallowed emit errors set ``latch``."""

    def __init__(self, *args: object, latch: OutputLatch, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.latch = latch

    def handleError(self, record: logging.LogRecord) -> None:
        self.latch.latch()
        super().handleError(record)


class _LatchedLogger(Protocol):
    output_latch: OutputLatch

    def close(self, *, drain_unstarted: bool = False) -> None: ...


def close_strictly(logger: _LatchedLogger) -> None:
    """Close, draining the FIFO even if the worker never ran, then raise
    :class:`LogOutputFailed` if any line was lost."""
    logger.close(drain_unstarted=True)
    logger.output_latch.check()


__all__ = ["LatchingRotatingFileHandler", "LogOutputFailed", "OutputLatch", "close_strictly"]
