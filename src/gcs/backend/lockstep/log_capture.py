"""The backend's log records, as ``DONE.log_lines`` (``lockstep_slice.md``, "Logs").

Installed on the root logger when the backend is built, at the backend file
log's level and format (:class:`navpy.sim.lockstep.log_capture.OutboxLogHandler`:
lockstep-stamped, tracebacks reduced to type and message, host paths as
run-relative tokens such as ``<run>/gcs/backend.log``). The coordinator hands
them to the operator, whose backend tail (``operator_tail.LockstepLogTail``)
reads them.
"""

from __future__ import annotations

import logging
from pathlib import Path

from navpy.sim.lockstep.log_capture import OutboxLogHandler
from navpy.sim.lockstep.participant_hooks import OutboxBuffer

#: The backend file log's line format (``gcs.backend.main._add_file_logging``).
LINE_FORMAT = "%(asctime)s [%(name)s] %(levelname)s: %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def capture_backend_logs(out: OutboxBuffer, run_dir: Path | None = None) -> OutboxLogHandler:
    """``run_dir``: the participant's host directory (default: its cwd)."""
    handler = OutboxLogHandler(out, Path.cwd() if run_dir is None else run_dir, level=logging.INFO,
                               line_format=LINE_FORMAT, date_format=DATE_FORMAT)
    logging.getLogger().addHandler(handler)
    return handler


__all__ = ["DATE_FORMAT", "LINE_FORMAT", "capture_backend_logs"]
