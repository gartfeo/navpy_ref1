"""Focused text/status output policy tests."""

from types import SimpleNamespace

import pytest

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_log_message import LogMessage
from navpy.logger.cache_log_output import CacheLogOutput, CacheStatusOutput


class _TextSink:
    def __init__(self):
        self.calls = []

    def debug(self, msg):
        self.calls.append(("debug", msg))

    def info(self, msg):
        self.calls.append(("info", msg))

    def warning(self, msg):
        self.calls.append(("warning", msg))

    def error(self, msg):
        self.calls.append(("error", msg))


class _StatusSink:
    def __init__(self):
        self.calls = []
        self.defer_calls = []
        self.close_calls = 0

    def send_log(self, msg, dest=None):
        self.calls.append((msg, dest))

    def defer_status_texts(self, enable, *, flush=True):
        self.defer_calls.append((enable, flush))

    def close(self):
        self.close_calls += 1


class _Clock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


def _policy(interval=1.0, level=CacheLogLevel.INFO):
    return SimpleNamespace(
        status_level=level,
        status_update_interval=interval,
    )


@pytest.mark.parametrize(
    ("level", "method"),
    [
        (CacheLogLevel.VERBOSE, "debug"),
        (CacheLogLevel.DEBUG, "debug"),
        (CacheLogLevel.INFO, "info"),
        (CacheLogLevel.WARNING, "warning"),
        (CacheLogLevel.ERROR, "error"),
    ],
)
def test_text_severity_dispatch(level, method):
    text = _TextSink()
    output = CacheLogOutput(text, CacheStatusOutput(_policy(), None))

    output.handle(LogMessage(level, "message"))

    assert text.calls == [(method, "message")]


def test_text_failure_prevents_same_message_status_dispatch():
    class BrokenText(_TextSink):
        def info(self, msg):
            raise RuntimeError("sink failed")

    status = _StatusSink()
    output = CacheLogOutput(
        BrokenText(),
        CacheStatusOutput(_policy(), status),
    )

    with pytest.raises(RuntimeError, match="sink failed"):
        output.handle(LogMessage(CacheLogLevel.INFO, "m", status="status"))

    assert status.calls == []


def test_status_cadence_dedupe_and_destination_are_independent_policies():
    clock = _Clock()
    sink = _StatusSink()
    output = CacheStatusOutput(_policy(interval=1.0), sink, clock=clock)

    output.emit(
        "early",
        CacheLogLevel.INFO,
        key="a",
        dest=LogStatusDest.DRONE,
        check_interval=True,
    )
    clock.value += 1.0
    output.emit(
        "accepted",
        CacheLogLevel.INFO,
        key="a",
        dest=LogStatusDest.DRONE,
        check_interval=True,
    )
    clock.value += 1.0
    output.emit("accepted", CacheLogLevel.INFO, key="a")
    output.emit("accepted", CacheLogLevel.INFO, key="b")

    assert sink.calls == [
        ("accepted", LogStatusDest.DRONE),
        ("accepted", None),
    ]


def test_status_severity_gate_and_none_sink_are_noops():
    sink = _StatusSink()
    output = CacheStatusOutput(_policy(level=CacheLogLevel.INFO), sink)

    output.emit("warning", CacheLogLevel.WARNING)
    CacheStatusOutput(_policy(), None).emit("info", CacheLogLevel.INFO)

    assert sink.calls == []


def test_status_defer_and_close_forward_once():
    sink = _StatusSink()
    output = CacheStatusOutput(_policy(), sink)

    output.defer(True, flush=False)
    output.close()
    output.close()

    assert sink.defer_calls == [(True, False)]
    assert sink.close_calls == 1
