"""Regression tests for UTF-8 log file encoding.

NavPy log lines can contain non-cp1252 characters (e.g. the '→' in
DetectionCoordinator start_tracking messages). The downloadable
``uav_*_navigation.log`` is written by a RotatingFileHandler, which on Windows
defaults to the cp1252 locale codec and raises UnicodeEncodeError on emit.
logger_factory pins ``encoding="utf-8"`` so the file is always written as UTF-8.

The GCS console-viewer path is fixed at the launch boundary -- the GCS spawns
NavPy with PYTHONUTF8=1 (see tests/gcs/backend/test_navpy_process_manager.py).
A direct CLI run (`python -m navpy.main`) without PYTHONUTF8 keeps a cp1252
console; there the safety net is this UTF-8 file handler plus the crash-proof
logging worker (tests/logger/test_cache_logger_resilience.py), not a UTF-8
console.
"""
from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from unittest.mock import MagicMock

from navpy.args.logger_args import LoggerArgsStub
from navpy.logger.logger_factory import LOG_DIR_ENV, initialize_logger

ARROW_MSG = "DetectionCoordinator: start_tracking task_id=1 → gimbal_0 obj_id=1"


def _file_handler(sys_id: int) -> RotatingFileHandler:
    vlog = logging.getLogger(f"vehicle_logger_{sys_id}")
    handlers = [h for h in vlog.handlers if isinstance(h, RotatingFileHandler)]
    assert handlers, "logger_factory should attach a RotatingFileHandler"
    return handlers[0]


def test_file_handler_uses_utf8_encoding(tmp_path, monkeypatch):
    # logger_factory writes .logs/ relative to cwd; isolate to tmp.
    monkeypatch.chdir(tmp_path)
    cache_logger = initialize_logger(LoggerArgsStub(), sys_id=4242, status_logger=None)

    try:
        handler = _file_handler(4242)
        assert (handler.stream.encoding or "").lower().replace("-", "") == "utf8"
    finally:
        # initialize_logger starts a CacheLogWorker daemon and opens a file
        # handler; close both so neither outlives the test.
        cache_logger.close()
        for handler in logging.getLogger("vehicle_logger_4242").handlers:
            handler.close()


def test_file_handler_writes_arrow_without_crash(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    cache_logger = initialize_logger(LoggerArgsStub(), sys_id=4343, status_logger=None)

    try:
        handler = _file_handler(4343)
        crashed = {"unicode": False}
        original_handle_error = handler.handleError

        def record_handle_error(record):
            import sys
            if sys.exc_info()[0] is UnicodeEncodeError:
                crashed["unicode"] = True
            original_handle_error(record)

        handler.handleError = record_handle_error

        logging.raiseExceptions = True
        logging.getLogger("vehicle_logger_4343").info(ARROW_MSG)
        handler.flush()

        assert not crashed["unicode"], "file handler crashed encoding '→'"
        with open(handler.baseFilename, encoding="utf-8") as fh:
            assert "→" in fh.read()
    finally:
        # initialize_logger starts a CacheLogWorker daemon and opens a file
        # handler; close both so neither outlives the test.
        cache_logger.close()
        for handler in logging.getLogger("vehicle_logger_4343").handlers:
            handler.close()


def test_explicit_log_directory_binds_process_logs(tmp_path, monkeypatch):
    log_dir = tmp_path / "one-evaluator-case"
    monkeypatch.setenv(LOG_DIR_ENV, str(log_dir))
    cache_logger = initialize_logger(
        LoggerArgsStub(), sys_id=4444, status_logger=MagicMock()
    )

    try:
        handler = _file_handler(4444)
        assert cache_logger.log_path == str(log_dir.resolve())
        assert handler.baseFilename == str(
            (log_dir / "uav_4444_navigation.log").resolve()
        )
    finally:
        cache_logger.close()
        for handler in logging.getLogger("vehicle_logger_4444").handlers:
            handler.close()
