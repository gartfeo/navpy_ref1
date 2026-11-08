"""NavPy's recovery journal (docs/design/navpy_recovery.md, step 8: evals honest).

Every recovery report (``ILogger.recovery``: a NAV recovery stage change, a stuck loop, a component restart, a failed
companion heartbeat) is counted and appended to ``uav_<sys_id>_recovery.jsonl``, apart from the evidence writer
(the navigation CSV streams) and from the log worker's queue: the line is written and flushed in the caller's thread,
so neither a failing evidence write nor a backed-up or deduplicating log loses it. The eval reads it to tell a clean
run from a recovered one (``scripts/eval_gcs_demo_recovery.py``).

The file is opened in append mode with an ``open`` line, so a restarted NavPy adds to its predecessor's journal and
a missing file means no journal at all. A failed write is reported once with RECOVERY_JOURNAL_FAILURE_MARKER, which
makes the eval's evidence invalid; recording goes on.
"""

from __future__ import annotations

import json
import os
from collections.abc import Callable
from typing import IO

from navpy.core.kernel import runtime_time as rt

RECOVERY_JOURNAL_FAILURE_MARKER = "RECOVERY_JOURNAL_FAILURE"


def recovery_journal_path(log_path: str, sys_id: int) -> str:
    return os.path.join(log_path, f"uav_{sys_id}_recovery.jsonl")


class RecoveryJournal:
    def __init__(self, path: str, report_failure: Callable[[str], None]) -> None:
        """``report_failure``: logs the marker line (the NavPy log, not this journal)."""
        self._lock = rt.Lock()
        self._report_failure = report_failure
        self._failed = False
        self._seq = 0
        self._counts: dict[str, int] = {}
        self._file: IO[str] | None = None
        try:
            self._file = open(path, "a", encoding="utf-8")
        except OSError as error:
            self._fail(error)
        self._write({"event": "open", "t": rt.time()})

    @property
    def counts(self) -> dict[str, int]:
        """Recoveries recorded by this process, per source."""
        with self._lock:
            return dict(self._counts)

    def record(self, source: str, message: str) -> None:
        with self._lock:
            self._seq += 1
            self._counts[source] = self._counts.get(source, 0) + 1
            self._write_locked({"seq": self._seq, "t": rt.time(), "source": source, "message": message})

    def close(self) -> None:
        with self._lock:
            file, self._file = self._file, None
        if file is not None:
            try:
                file.close()
            except OSError as error:
                self._fail(error)

    def _write(self, entry: dict[str, object]) -> None:
        with self._lock:
            self._write_locked(entry)

    def _write_locked(self, entry: dict[str, object]) -> None:
        if self._file is None:
            return
        try:
            self._file.write(json.dumps(entry, sort_keys=True) + "\n")
            self._file.flush()
        except (OSError, ValueError) as error:
            self._fail(error)

    def _fail(self, error: BaseException) -> None:
        if self._failed:
            return
        self._failed = True
        try:
            self._report_failure(f"{RECOVERY_JOURNAL_FAILURE_MARKER}: {type(error).__name__}: {error}")
        except Exception:  # noqa: BLE001 - the eval then finds no marker, but the journal's gap or absence
            pass


__all__ = ["RECOVERY_JOURNAL_FAILURE_MARKER", "RecoveryJournal", "recovery_journal_path"]
