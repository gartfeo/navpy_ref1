"""Lifecycle and isolation tests for the focused logging worker."""

import threading
import time
from unittest.mock import patch

import pytest

from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_log_message import LogMessage
from navpy.logger.cache_log_worker import CacheLogWorker


def _message(text: str) -> LogMessage:
    return LogMessage(CacheLogLevel.INFO, text)


def test_worker_drains_fifo_before_close_returns():
    seen = []
    worker = CacheLogWorker(lambda message: seen.append(message.msg))

    assert worker.submit(_message("one"))
    assert worker.submit(_message("two"))
    worker.close()

    assert seen == ["one", "two"]
    assert not worker.is_alive


def test_one_handler_failure_does_not_block_following_message():
    seen = []
    errors = []

    def handle(message):
        if message.msg == "bad":
            raise RuntimeError("poison")
        seen.append(message.msg)

    worker = CacheLogWorker(handle, on_error=errors.append)
    worker.submit(_message("bad"))
    worker.submit(_message("good"))
    worker.close()

    assert seen == ["good"]
    assert [str(error) for error in errors] == ["poison"]


def test_idle_barrier_keeps_worker_open_for_more_messages():
    seen = []
    worker = CacheLogWorker(lambda message: seen.append(message.msg))
    worker.submit(_message("before"))

    worker.wait_until_idle()

    assert seen == ["before"]
    assert worker.is_alive
    assert worker.submit(_message("after"))
    worker.close()
    assert seen == ["before", "after"]


def test_close_is_idempotent_and_rejects_post_close_messages():
    seen = []
    worker = CacheLogWorker(lambda message: seen.append(message.msg))

    worker.close()
    worker.close()

    assert not worker.submit(_message("late"))
    assert seen == []


def test_blocked_handler_close_is_bounded_retained_and_retryable():
    entered = threading.Event()
    release = threading.Event()

    def handle(_message):
        entered.set()
        assert release.wait(timeout=1.0)

    worker = CacheLogWorker(handle)
    assert worker.submit(_message("blocked"))
    assert entered.wait(timeout=1.0)

    with patch(
        "navpy.logger.cache_log_worker.CACHE_WORKER_JOIN_TIMEOUT_S",
        0.01,
    ):
        started_s = time.perf_counter()
        with pytest.raises(TimeoutError, match="output remains owned"):
            worker.close()
        assert time.perf_counter() - started_s < 0.2

    assert worker.is_alive
    release.set()
    worker.close()
    assert not worker.is_alive
