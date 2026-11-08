"""NavPy's recovery journal (docs/design/navpy_recovery.md, step 8: evals honest).

Every recovery report (``ILogger.recovery``: a NAV recovery stage change, a stuck loop, a component restart, a failed
companion heartbeat, the autopilot's NavPy watchdog acting) is counted and appended to ``uav_<sys_id>_recovery.jsonl``,
apart from the evidence writer (the navigation CSV streams) and from the log worker's queue and its de-duplication.
The eval reads it to tell a clean run from a recovered one (``scripts/eval_gcs_demo_recovery.py``).

Recording never blocks the recovery that reports (Codex review of step 8, finding 1): ``record`` only counts and
appends to a bounded buffer under a lock that no I/O ever holds; a writer thread of its own writes and flushes. A
full buffer drops the entry and counts the drop.

Each NavPy process writes an ``open`` line first and, when its logger closes, a ``close`` line with how many entries it
recorded and dropped. The file is appended to, so a restarted NavPy adds a second ``open``; a journal whose last
process has no ``close``, or whose counts disagree with its lines, is incomplete evidence. A failed open or write is
reported once with RECOVERY_JOURNAL_FAILURE_MARKER, which also makes the eval's evidence invalid.
"""

from __future__ import annotations

import json
import os
from collections import deque
from collections.abc import Callable
from typing import IO

from navpy.core.kernel import runtime_time as rt

RECOVERY_JOURNAL_FAILURE_MARKER = "RECOVERY_JOURNAL_FAILURE"
#: Entries waiting for the writer. Recoveries are rare (a handful per fault); a backlog this long means the disk has
#: stalled, and later entries are counted as dropped rather than held.
RECOVERY_JOURNAL_BACKLOG = 256
#: How long ``close`` waits for the writer to write what is queued and the close line.
RECOVERY_JOURNAL_CLOSE_S = 2.0


def recovery_journal_path(log_path: str, sys_id: int) -> str:
    return os.path.join(log_path, f"uav_{sys_id}_recovery.jsonl")


class RecoveryJournal:
    def __init__(
        self,
        path: str,
        report_failure: Callable[[str], None],
        open_file: Callable[[str], IO[str]] = lambda path: open(path, "a", encoding="utf-8"),
    ) -> None:
        """``report_failure``: logs the marker line (the NavPy log, not this journal)."""
        self._path = path
        self._report_failure = report_failure
        self._open_file = open_file
        self._cond = rt.Condition()
        self._pending: deque[dict[str, object] | None] = deque([{"event": "open", "t": rt.time()}])
        self._seq = 0
        self._dropped = 0
        self._counts: dict[str, int] = {}
        self._closed = False
        self._failed = False
        self._writer = rt.Thread(target=self._write_all, name="recovery-journal", daemon=True)
        with rt.detached():  # owned by the journal, not the request that made it
            self._writer.start()

    @property
    def counts(self) -> dict[str, int]:
        """Recoveries recorded by this process, per source."""
        with self._cond:
            return dict(self._counts)

    @property
    def dropped(self) -> int:
        with self._cond:
            return self._dropped

    def record(self, source: str, message: str) -> None:
        with self._cond:
            if self._closed:
                return
            self._seq += 1
            self._counts[source] = self._counts.get(source, 0) + 1
            if len(self._pending) >= RECOVERY_JOURNAL_BACKLOG:
                self._dropped += 1
                return
            self._pending.append({"seq": self._seq, "t": rt.time(), "source": source, "message": message})
            self._cond.notify()

    def close(self) -> None:
        with self._cond:
            if self._closed:
                return
            self._closed = True
            # The close line is never dropped: the eval needs it to know the journal is whole.
            self._pending.append({"event": "close", "t": rt.time(), "recorded": self._seq, "dropped": self._dropped})
            self._pending.append(None)
            self._cond.notify()
        self._writer.join(RECOVERY_JOURNAL_CLOSE_S)
        if self._writer.is_alive():
            self._fail(TimeoutError(f"recovery journal writer did not finish in {RECOVERY_JOURNAL_CLOSE_S:g} s"))

    def _write_all(self) -> None:
        file = self._open()
        try:
            while True:
                with self._cond:
                    while not self._pending:
                        self._cond.wait()
                    entry = self._pending.popleft()
                if entry is None:
                    return
                if file is not None:
                    self._write(file, entry)
        finally:
            if file is not None:
                try:
                    file.close()
                except OSError as error:
                    self._fail(error)

    def _open(self) -> IO[str] | None:
        try:
            return self._open_file(self._path)
        except OSError as error:
            self._fail(error)
            return None

    def _write(self, file: IO[str], entry: dict[str, object]) -> None:
        try:
            file.write(json.dumps(entry, sort_keys=True) + "\n")
            file.flush()
        except (OSError, ValueError) as error:
            self._fail(error)

    def _fail(self, error: BaseException) -> None:
        with self._cond:
            if self._failed:
                return
            self._failed = True
        try:
            self._report_failure(f"{RECOVERY_JOURNAL_FAILURE_MARKER}: {type(error).__name__}: {error}")
        except Exception:  # noqa: BLE001 - the eval then finds no close line, so the journal is still invalid
            pass


__all__ = [
    "RECOVERY_JOURNAL_BACKLOG", "RECOVERY_JOURNAL_FAILURE_MARKER", "RecoveryJournal", "recovery_journal_path",
]
