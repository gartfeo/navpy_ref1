"""Tests for CacheLogger non-string status= tracer at enqueue time.

The tracer fires synchronously on the caller thread so traceback.format_stack()
names the offending caller (the queued worker path loses the real stack).
It must not coerce or mask the bad value -- downstream LogStatusMsg encode
still crashes as before, preserving visibility of the bug.
"""
from __future__ import annotations

import logging

import pytest

from navpy.args.logger_args import LogStatusDest
from navpy.logger import cache_logger as cl_module
from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_logger import (
    CacheLogger,
    _make_non_string_status_tracer,
)


class _LoggerArgsStub:
    """Minimal LoggerArgs shim for CacheLogger construction."""
    log_level = CacheLogLevel.INFO
    status_level = CacheLogLevel.INFO
    status_update_interval = 0.0
    defer_status_logs = False
    status_dest = set()


class _RecordingLogger:
    def __init__(self):
        self.info_messages = []

    def debug(self, msg):
        pass

    def info(self, msg):
        self.info_messages.append(msg)

    def warning(self, msg):
        pass

    def error(self, msg):
        pass


class _RecordingStatusLogger:
    def __init__(self):
        self.messages = []

    def send_log(self, msg, dest=None):
        self.messages.append((msg, dest))

    def defer_status_texts(self, enable, *, flush=True):
        pass

    def close(self):
        pass


_OPEN_LOGGERS = []


def _cache_logger(logger=None, status_logger=None) -> CacheLogger:
    # The underlying stdlib logger is irrelevant -- we assert on the
    # module-level _diag_log for tracer output. Pass a no-op stub.
    result = CacheLogger(
        _LoggerArgsStub(),
        logger=logger or logging.getLogger("test.cl"),
        status_logger=status_logger,
    )
    _OPEN_LOGGERS.append(result)
    return result


@pytest.fixture(autouse=True)
def _close_loggers():
    yield
    while _OPEN_LOGGERS:
        _OPEN_LOGGERS.pop().close()


def _fresh_tracer(monkeypatch, max_traces: int = 5) -> None:
    monkeypatch.setattr(
        cl_module, "_trace_non_string_status",
        _make_non_string_status_tracer(max_traces=max_traces),
    )


# ---------------------------------------------------------------------------
# String status (happy path): no trace emitted
# ---------------------------------------------------------------------------

def test_string_status_does_not_trigger_tracer(caplog, monkeypatch):
    _fresh_tracer(monkeypatch)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        cl.info("some message", status="Operational")
        cl.warning("something bad", status="Warning text")
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert errors == []


# ---------------------------------------------------------------------------
# Non-string status triggers trace WITH live caller stack
# ---------------------------------------------------------------------------

def test_float_status_traces_with_caller_stack_on_info(caplog, monkeypatch):
    _fresh_tracer(monkeypatch)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        cl.info("navigation msg", status=15.0)
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    msg = errors[0].getMessage()
    assert "CacheLogger.info" in msg
    assert "type=float" in msg
    assert "15.0" in msg
    # Caller stack must reference THIS test frame (proves synchronous capture).
    assert "test_cache_logger_trace" in msg
    assert "test_float_status_traces_with_caller_stack_on_info" in msg


def test_float_status_traces_with_caller_stack_on_warning(caplog, monkeypatch):
    _fresh_tracer(monkeypatch)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        cl.warning("warning msg", status=2.5)
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    msg = errors[0].getMessage()
    assert "CacheLogger.warning" in msg
    assert "type=float" in msg
    assert "2.5" in msg
    assert "test_cache_logger_trace" in msg


def test_int_status_also_traces(caplog, monkeypatch):
    _fresh_tracer(monkeypatch)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        cl.info("m", status=7)
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    assert "type=int" in errors[0].getMessage()


def test_status_none_does_not_trace(caplog, monkeypatch):
    _fresh_tracer(monkeypatch)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        cl.info("m", status=None)
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert errors == []


# ---------------------------------------------------------------------------
# Suppression threshold
# ---------------------------------------------------------------------------

def test_suppression_kicks_in_after_max_traces(caplog, monkeypatch):
    _fresh_tracer(monkeypatch, max_traces=5)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        for i in range(10):
            # key must differ each time, or CacheLogger dedup skips the call
            cl.info(f"m{i}", key=f"k{i}", status=float(i))
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 6  # 5 full stacks + 1 suppression notice
    assert "suppressing" in errors[-1].getMessage()
    for rec in errors[:5]:
        assert "Caller stack:" in rec.getMessage()


def test_info_is_cached_by_key_and_message(monkeypatch):
    _fresh_tracer(monkeypatch)
    sink = _RecordingLogger()
    cl = _cache_logger(logger=sink)
    cl.info("PEER_GEO_ACQ: poi=1 reason=out_of_fov", key="peer_geo_acq")
    cl.info("PEER_GEO_ACQ: poi=1 reason=out_of_fov", key="peer_geo_acq")
    cl.close()
    _OPEN_LOGGERS.remove(cl)

    assert sink.info_messages.count(
        "PEER_GEO_ACQ: poi=1 reason=out_of_fov"
    ) == 1


# ---------------------------------------------------------------------------
# Bug NOT masked: the LogMessage is still enqueued with the raw status value
# ---------------------------------------------------------------------------

def test_non_string_status_still_enqueued(monkeypatch):
    """The tracer must not swallow the bad status: the log path still
    carries the raw float value downstream so existing crashes still fire.

    The recording status sink proves that the queued boundary receives the
    exact object rather than a coerced string."""
    _fresh_tracer(monkeypatch)
    status = _RecordingStatusLogger()
    cl = _cache_logger(status_logger=status)
    cl.info("m", status=9.9)
    cl.close()
    _OPEN_LOGGERS.remove(cl)

    assert status.messages[0][0] == 9.9
    assert isinstance(status.messages[0][0], float)


# ---------------------------------------------------------------------------
# dest promotion: info(msg, dest=...) promotes msg -> status internally
# ---------------------------------------------------------------------------

def test_non_string_msg_promoted_via_dest_also_traced(caplog, monkeypatch):
    """If a caller does logger.info(<float>, dest=LogStatusDest.DRONE) the
    CacheLogger's status = msg promotion puts the float into status, which
    must trigger the same tracer."""
    _fresh_tracer(monkeypatch)
    cl = _cache_logger()
    with caplog.at_level(logging.ERROR, logger=cl_module.__name__):
        cl.info(15.0, dest=LogStatusDest.DRONE)
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    msg = errors[0].getMessage()
    assert "CacheLogger.info" in msg
    assert "type=float" in msg
    assert "15.0" in msg
    assert "test_cache_logger_trace" in msg
