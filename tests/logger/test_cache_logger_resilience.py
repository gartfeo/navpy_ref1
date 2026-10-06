"""The logging worker thread must never die from a bad log message.

CacheLogger drains queued messages on a background thread. If handling one
message raised, the thread used to exit and ALL further logging and status
broadcasts stopped silently. _process_log_queue now isolates per-message
failures so a single bad line (an un-encodable status, a downstream send
error, etc.) can never break logging for the rest of the process.
"""
from __future__ import annotations

from navpy.args.logger_args import LoggerArgsStub
from navpy.logger.cache_logger import CacheLogger


class _FlakyLogger:
    """Underlying logger whose .info() raises on the poisoned message."""

    def __init__(self):
        self.seen = []

    def info(self, msg):
        if "boom" in msg:
            raise RuntimeError("simulated handler failure")
        self.seen.append(msg)

    def debug(self, msg):
        pass

    def warning(self, msg):
        pass

    def error(self, msg):
        pass


def test_worker_survives_handler_exception():
    flaky = _FlakyLogger()
    cl = CacheLogger(LoggerArgsStub(), flaky, status_logger=None)
    # First message blows up inside the focused worker output...
    cl.info("boom message", key="poison")
    # ...the second must still be processed by the same worker thread.
    cl.info("healthy message", key="ok")

    # Public close is the deterministic drain barrier.
    cl.close()

    assert "healthy message" in flaky.seen


def test_close_without_status_logger_is_safe_and_stops_worker():
    sink = _FlakyLogger()
    cl = CacheLogger(LoggerArgsStub(), sink, status_logger=None)

    cl.close()
    cl.close()

    assert sink.seen.count("Logger closed") == 1
