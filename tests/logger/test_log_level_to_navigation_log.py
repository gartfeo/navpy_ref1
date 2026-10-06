"""Integration: the chosen log level gates DEBUG lines in navigation.log.

Exercises the full effective-level chain that the ``-ll``/``--log-level`` flag
feeds: ``LoggerArgs.log_level`` -> ``CacheLogger._level`` -> the DEBUG-level
RotatingFileHandler in ``initialize_logger``. At DEBUG the file captures
``debug()`` lines (e.g. gimbal-rate TRACKER debug); at the INFO default it must
not, which is exactly why DEBUG never reached navigation.log before this fix.
"""
import argparse
import logging
from pathlib import Path
from unittest.mock import MagicMock

from navpy.args.logger_args import LoggerArgs
from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.logger_factory import initialize_logger


def _logger_args(level):
    ns = argparse.Namespace(
        log_status_interval=2,
        log_status_dest=["Vehicle", "Network"],
        log_level=level,
        log_status_level=level,
    )
    return LoggerArgs(ns)


def _run(level, sys_id, monkeypatch, tmp_path):
    # Redirect the factory's relative ".logs/..." tree into tmp_path.
    monkeypatch.chdir(tmp_path)
    cache_logger = initialize_logger(
        _logger_args(level), sys_id=sys_id, status_logger=MagicMock(),
    )
    try:
        cache_logger.debug("TRACKER: marker_debug_line")
        cache_logger.info("marker_info_line")
        cache_logger.close()  # drains the worker queue, flushes handlers
        log_file = Path(cache_logger.log_path) / f"uav_{sys_id}_navigation.log"
        return log_file.read_text(encoding="utf-8")
    finally:
        # Release the file handle so tmp_path teardown can delete it on Windows.
        for handler in logging.getLogger(f"vehicle_logger_{sys_id}").handlers:
            handler.close()


def test_debug_level_writes_debug_lines(monkeypatch, tmp_path):
    text = _run(CacheLogLevel.DEBUG, 7, monkeypatch, tmp_path)
    assert "TRACKER: marker_debug_line" in text
    assert "marker_info_line" in text


def test_info_level_suppresses_debug_lines(monkeypatch, tmp_path):
    text = _run(CacheLogLevel.INFO, 8, monkeypatch, tmp_path)
    assert "TRACKER: marker_debug_line" not in text
    assert "marker_info_line" in text
