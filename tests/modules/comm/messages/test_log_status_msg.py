"""Tests for LogStatusMsg non-string-status diagnostic tracer.

The tracer is diagnostic-only: it MUST NOT mutate the stored status
(coercion would mask the real caller bug). It MUST emit a logger.error
with caller stack on the first few occurrences, then suppress further
stacks to avoid log flooding.
"""
from __future__ import annotations

import logging

import pytest

from navpy.modules.comm.messages import log_status_msg
from navpy.modules.comm.messages.log_status_msg import (
    LogStatusMsg,
    _make_non_string_status_tracer,
)


# ---------------------------------------------------------------------------
# String status (happy path): no tracing emitted; status preserved
# ---------------------------------------------------------------------------

def test_string_status_emits_no_error(caplog):
    with caplog.at_level(logging.ERROR, logger=log_status_msg.__name__):
        msg = LogStatusMsg(sender_id=1, status="Operational")
    assert msg.status == "Operational"
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert errors == [], "string status must not trigger the tracer"


# ---------------------------------------------------------------------------
# Non-string status (the bug): error emitted, bug NOT masked
# ---------------------------------------------------------------------------

def test_float_status_emits_error_without_masking_value(caplog, monkeypatch):
    # Fresh tracer per test to reset the suppression counter.
    monkeypatch.setattr(
        log_status_msg, "_trace_non_string_status",
        _make_non_string_status_tracer(),
    )
    with caplog.at_level(logging.ERROR, logger=log_status_msg.__name__):
        msg = LogStatusMsg(sender_id=42, status=3.14)
    # The bug is NOT masked: status remains the raw float so to_mavlink
    # will still crash and the caller visibly fails.
    assert msg.status == 3.14
    assert isinstance(msg.status, float)

    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    combined = errors[0].getMessage()
    assert "type=float" in combined
    assert "3.14" in combined
    assert "sender_id=42" in combined
    # Caller stack must be included (one of the traceback lines starts with
    # "File " and mentions this test file).
    assert "File " in combined
    assert "test_log_status_msg" in combined


def test_int_status_also_triggers_tracer(caplog, monkeypatch):
    monkeypatch.setattr(
        log_status_msg, "_trace_non_string_status",
        _make_non_string_status_tracer(),
    )
    with caplog.at_level(logging.ERROR, logger=log_status_msg.__name__):
        msg = LogStatusMsg(sender_id=1, status=7)
    assert msg.status == 7
    assert isinstance(msg.status, int)
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    assert "type=int" in errors[0].getMessage()


def test_none_status_triggers_tracer(caplog, monkeypatch):
    monkeypatch.setattr(
        log_status_msg, "_trace_non_string_status",
        _make_non_string_status_tracer(),
    )
    with caplog.at_level(logging.ERROR, logger=log_status_msg.__name__):
        msg = LogStatusMsg(sender_id=1, status=None)
    assert msg.status is None
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    assert "type=NoneType" in errors[0].getMessage()


# ---------------------------------------------------------------------------
# Suppression: first 5 full stacks, 6th emits a suppression notice, then silent
# ---------------------------------------------------------------------------

def test_suppression_kicks_in_after_max_traces(caplog, monkeypatch):
    monkeypatch.setattr(
        log_status_msg, "_trace_non_string_status",
        _make_non_string_status_tracer(max_traces=5),
    )
    with caplog.at_level(logging.ERROR, logger=log_status_msg.__name__):
        for i in range(10):
            msg = LogStatusMsg(sender_id=i, status=float(i))
            # bug not masked on any iteration
            assert msg.status == float(i)

    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    # 5 full stacks + 1 suppression notice = 6 error records total.
    assert len(errors) == 6
    # Last error must be the suppression notice (no caller stack).
    last = errors[-1].getMessage()
    assert "suppressing" in last
    assert "File " not in last
    # First 5 must all carry a caller stack.
    for rec in errors[:5]:
        assert "File " in rec.getMessage()
        assert "Caller stack:" in rec.getMessage()


def test_custom_max_traces_limit(caplog, monkeypatch):
    monkeypatch.setattr(
        log_status_msg, "_trace_non_string_status",
        _make_non_string_status_tracer(max_traces=2),
    )
    with caplog.at_level(logging.ERROR, logger=log_status_msg.__name__):
        for i in range(5):
            LogStatusMsg(sender_id=i, status=float(i))
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    # 2 full stacks + 1 suppression notice
    assert len(errors) == 3
    assert "suppressing" in errors[-1].getMessage()


# ---------------------------------------------------------------------------
# to_mavlink crash still surfaces (i.e. the bug is genuinely not masked)
# ---------------------------------------------------------------------------

def test_to_mavlink_still_raises_on_float_status(monkeypatch):
    monkeypatch.setattr(
        log_status_msg, "_trace_non_string_status",
        _make_non_string_status_tracer(),
    )
    msg = LogStatusMsg(sender_id=1, status=3.14)
    with pytest.raises(AttributeError, match="encode"):
        msg.to_mavlink()
