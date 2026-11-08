"""NavPy's recovery journal (navpy_recovery.md, step 8): every recovery report is counted and written apart from
the evidence writer and the log worker."""
from __future__ import annotations

import json
import logging

from navpy.core.kernel.config.logger_args import LoggerArgsStub
from navpy.core.kernel.logging.logger_factory import LOG_DIR_ENV, initialize_logger
from navpy.core.kernel.logging.recovery_journal import (
    RECOVERY_JOURNAL_FAILURE_MARKER,
    RecoveryJournal,
    recovery_journal_path,
)
from tests.logger.log_worker_join import unbounded_worker_join


def _lines(path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_records_are_counted_and_appended_with_sequence_and_source(tmp_path):
    path = tmp_path / "uav_3_recovery.jsonl"
    failures: list[str] = []
    journal = RecoveryJournal(str(path), failures.append)
    journal.record("nav", "NavPy hold: commands fenced, LOITER")
    journal.record("vision", "Detector restarted")
    journal.record("nav", "NavPy healthy again: AUTO")
    journal.close()
    lines = _lines(path)
    assert lines[0]["event"] == "open"
    assert [(e["seq"], e["source"], e["message"]) for e in lines[1:]] == [
        (1, "nav", "NavPy hold: commands fenced, LOITER"),
        (2, "vision", "Detector restarted"),
        (3, "nav", "NavPy healthy again: AUTO"),
    ]
    assert journal.counts == {"nav": 2, "vision": 1} and failures == []


def test_a_restarted_navpy_appends_to_its_predecessors_journal(tmp_path):
    path = tmp_path / "uav_1_recovery.jsonl"
    first = RecoveryJournal(str(path), print)
    first.record("nav", "NavPy hold: commands fenced")
    first.close()
    second = RecoveryJournal(str(path), print)
    second.record("nav", "NavPy healthy again: AUTO")
    second.close()
    assert [e.get("event") or e["message"] for e in _lines(path)] == [
        "open", "NavPy hold: commands fenced", "open", "NavPy healthy again: AUTO",
    ]


def test_a_journal_that_cannot_open_reports_the_marker_once_and_records_on(tmp_path):
    failures: list[str] = []
    journal = RecoveryJournal(str(tmp_path / "missing_dir" / "uav_1_recovery.jsonl"), failures.append)
    journal.record("nav", "NavPy hold: commands fenced")
    journal.record("nav", "NavPy healthy again: AUTO")
    assert len(failures) == 1 and failures[0].startswith(RECOVERY_JOURNAL_FAILURE_MARKER)
    assert journal.counts == {"nav": 2}


def test_a_failed_write_reports_the_marker_once(tmp_path):
    failures: list[str] = []
    journal = RecoveryJournal(str(tmp_path / "uav_1_recovery.jsonl"), failures.append)

    class BrokenFile:
        def write(self, text):
            raise OSError("disk full")

        def close(self):
            pass

    journal._file = BrokenFile()
    journal.record("nav", "a")
    journal.record("nav", "b")
    assert len(failures) == 1 and "disk full" in failures[0]


def test_navpy_s_logger_journals_a_recovery_before_its_log_worker_writes(tmp_path, monkeypatch):
    """The report is in the journal as soon as ``recovery`` returns, whatever the log worker's queue holds."""
    monkeypatch.setenv(LOG_DIR_ENV, str(tmp_path))
    logger = initialize_logger(LoggerArgsStub(), sys_id=4545, status_logger=None)
    try:
        logger.recovery("NavPy hold: commands fenced, LOITER", "nav")
        path = tmp_path / "uav_4545_recovery.jsonl"
        assert str(path) == recovery_journal_path(str(tmp_path), 4545)
        assert [(e["source"], e["message"]) for e in _lines(path)[1:]] == [("nav", "NavPy hold: commands fenced, LOITER")]
    finally:
        with unbounded_worker_join():
            logger.close()
        for handler in logging.getLogger("vehicle_logger_4545").handlers:
            handler.close()
