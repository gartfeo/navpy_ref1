"""A participant's log records as ``DONE.log_lines`` (``lockstep_slice.md``, "Logs").

A product installs :class:`OutboxLogHandler` on the logger its file log
uses, with that file's line format, so each line reaches ``DONE`` as well as
the file. Each record is stamped by the lockstep clock
(:mod:`navpy.sim.lockstep.log_stamp`, bootstrap step 2), so a line's bytes
are the same on every host: an exception is logged by type and message only
(no traceback), and every host path becomes a run-relative token
(:mod:`.host_paths`, e.g. ``<run>/gcs/backend.log``). The coordinator hands
the lines to the operator, whose tails (``operator_tail.LockstepLogTail``)
read them.
"""

from __future__ import annotations

import logging
import traceback
from pathlib import Path
from typing import Any

from navpy.sim.lockstep.host_paths import HostPathScrubber
from navpy.sim.lockstep.participant_hooks import OutboxBuffer


class HostFreeFormatter(logging.Formatter):
    """The line, and for an exception only its type and message: a traceback
    names host paths (its ``File`` lines), which no ``DONE`` may carry."""

    def format(self, record: logging.LogRecord) -> str:
        cached = record.exc_text  # another handler's full traceback, if it ran first
        record.exc_text = None
        try:
            return super().format(record)
        finally:
            record.exc_text = cached  # the file log formats its own

    def formatException(self, ei: Any) -> str:
        return "".join(traceback.format_exception_only(ei[0], ei[1])).rstrip()

    def formatStack(self, stack_info: str) -> str:
        return ""


class HostPathScrubbingFormatter(logging.Formatter):
    """A file log's own formatter, its lines with host paths as run tokens.

    A participant's file log is a run artifact: it keeps its format and its
    tracebacks, but names no host path (those go to the ``*_host`` logs).
    """

    def __init__(self, inner: logging.Formatter, run_dir: Path) -> None:
        super().__init__()
        self._inner = inner
        self._scrub = HostPathScrubber(run_dir)

    def format(self, record: logging.LogRecord) -> str:
        return self._scrub(self._inner.format(record))


class OutboxLogHandler(logging.Handler):
    """``run_dir``: the participant's host directory (its cwd)."""

    def __init__(self, out: OutboxBuffer, run_dir: Path, *, level: int, line_format: str,
                 date_format: str | None = None) -> None:
        super().__init__(level)
        self._out = out
        self._scrub = HostPathScrubber(run_dir)
        self.setFormatter(HostFreeFormatter(line_format, datefmt=date_format))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._out.log_lines.append(self._scrub(self.format(record)))
        except Exception:
            self.handleError(record)

    def handleError(self, record: logging.LogRecord) -> None:
        """A line that cannot be formatted is lost output: a fault, not stderr."""
        self._out.faults.append(f"LogCaptureFailed:{record.name}")


__all__ = ["HostFreeFormatter", "HostPathScrubbingFormatter", "OutboxLogHandler"]
