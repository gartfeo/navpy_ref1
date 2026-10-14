"""A flushed drain certifies recording: diagnostics and the backend file log.

``DiagnosticSession.stop_strictly`` raises when any record, ``session_finished``
included, failed to reach the timeline; ``check_file_logging`` raises when a
backend log line failed to reach its file (the 9b ``OutputLatch``). The
strict shutdown runs both, so either failure ends the backend unflushed.
Wall ``stop`` keeps swallowing.
"""

from __future__ import annotations

import logging

import pytest

from gcs.backend import main
from gcs.backend.diagnostics import DiagnosticSession, DiagnosticsWriteFailed
from navpy.core.kernel.logging.log_output_latch import LogOutputFailed, OutputLatch


def test_a_clean_session_stops_strictly(tmp_path):
    session = DiagnosticSession(tmp_path / "sessions")
    session.start()
    session.emit("vehicle_connect_finished", source="backend")
    session.stop_strictly()
    timeline = next((tmp_path / "sessions").glob("*/timeline.jsonl")).read_text()
    assert '"event":"session_finished"' in timeline


def test_an_unwritable_session_finished_fails_the_strict_stop(tmp_path, monkeypatch):
    session = DiagnosticSession(tmp_path / "sessions")
    session.start()
    timeline = session._timeline_path
    monkeypatch.setattr(session, "_timeline_path", lambda: timeline().parent)  # a directory: open fails
    with pytest.raises(DiagnosticsWriteFailed):
        session.stop_strictly()


def test_the_wall_stop_still_swallows_a_write_failure(tmp_path, monkeypatch):
    session = DiagnosticSession(tmp_path / "sessions")
    session.start()
    monkeypatch.setattr(session, "_timeline_path", lambda: tmp_path)
    session.stop()
    assert session.write_failed and not session.active


def test_a_lost_backend_log_line_fails_the_file_log_check(tmp_path, monkeypatch):
    monkeypatch.setattr(main, "file_log_latch", OutputLatch())
    root = logging.getLogger()
    before = list(root.handlers)
    monkeypatch.setattr(logging, "raiseExceptions", False)  # no stderr report in the test
    try:
        assert main._add_file_logging(tmp_path) == tmp_path / "backend.log"
        main.check_file_logging()  # nothing lost yet
        logging.getLogger("gcs.backend.test").warning("%d", "not a number")  # emit fails
        with pytest.raises(LogOutputFailed):
            main.check_file_logging()
    finally:
        for handler in [h for h in root.handlers if h not in before]:
            root.removeHandler(handler)
            handler.close()
