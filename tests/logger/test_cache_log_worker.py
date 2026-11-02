"""Lifecycle and isolation tests for the focused logging worker."""

import threading
import time
from unittest.mock import patch

import pytest

from navpy.core.kernel.logging.cache_log_level import CacheLogLevel
from navpy.core.kernel.logging.cache_log_message import LogMessage
from navpy.core.kernel.logging.cache_log_worker import CacheLogWorker


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


def test_a_handler_failure_is_latched():
    worker = CacheLogWorker(lambda _message: 1 / 0, on_error=lambda _exc: None)
    assert not worker.failed
    worker.submit(_message("bad"))
    worker.close()
    assert worker.failed


def _close_before_the_worker_runs(**close_kwargs) -> list:
    """Submit two messages and close twice while the worker has not yet entered."""
    seen = []
    release = threading.Event()
    original = CacheLogWorker._run_generation

    def late_start(self) -> None:
        assert release.wait(timeout=1.0)
        original(self)

    with patch.object(CacheLogWorker, "_run_generation", late_start):
        worker = CacheLogWorker(lambda message: seen.append(message.msg))
        worker.submit(_message("one"))
        worker.submit(_message("two"))
        worker.close(**close_kwargs)
        worker.close(**close_kwargs)  # idempotent: nothing more is handled
        release.set()
        worker._thread.join(timeout=1.0)  # the cancelled worker exits unrun
    assert not worker.is_alive and not worker.failed
    return seen


def test_default_close_before_the_worker_runs_drops_the_queue():
    """Wall close is unchanged: a cancelled worker's queued messages are not written."""
    assert _close_before_the_worker_runs() == []


def test_drain_unstarted_close_handles_every_queued_message():
    """The strict close drains a cancelled worker's FIFO in the caller."""
    assert _close_before_the_worker_runs(drain_unstarted=True) == ["one", "two"]


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
        "navpy.core.kernel.logging.cache_log_worker.CACHE_WORKER_JOIN_TIMEOUT_S",
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


def test_a_message_the_drain_produces_after_close_is_handled_in_order():
    """E.g. a status send that fails while the FIFO drains logs its drop warning."""
    seen = []
    started, release = threading.Event(), threading.Event()
    worker = None

    def handle(message):
        if message.msg == "first":
            started.set()
            release.wait(5.0)
        seen.append(message.msg)
        if message.msg == "send":
            assert worker.submit(_message("send failed"))

    worker = CacheLogWorker(handle)
    worker.submit(_message("first"))
    assert started.wait(5.0)
    worker.submit(_message("send"))
    worker.submit(_message("last"))
    closer = threading.Thread(target=worker.close)
    closer.start()
    while worker.submit(_message("probe")):  # wait until close stopped accepting
        time.sleep(0.001)
    release.set()
    closer.join(5.0)

    assert seen == ["first", "send", "send failed", "last"]
    assert not worker.submit(_message("late"))


def test_a_message_the_strict_drain_produces_is_handled_in_place():
    seen = []
    worker = None
    release = threading.Event()
    original = CacheLogWorker._run_generation

    def handle(message):
        seen.append(message.msg)
        if message.msg == "send":
            assert worker.submit(_message("send failed"))

    def late_start(self) -> None:
        assert release.wait(timeout=1.0)
        original(self)

    with patch.object(CacheLogWorker, "_run_generation", late_start):
        worker = CacheLogWorker(handle)
        worker.submit(_message("send"))
        worker.submit(_message("last"))
        late = []

        def close_then_log():
            # Drains in this thread, which holds the accept lock meanwhile.
            worker.close(drain_unstarted=True)
            late.append(worker.submit(_message("late")))

        closer = threading.Thread(target=close_then_log, daemon=True)
        closer.start()
        closer.join(5.0)
        assert not closer.is_alive(), "the nested submit deadlocked on the accept lock"
        release.set()
        worker._thread.join(timeout=1.0)

    assert seen == ["send", "send failed", "last"]
    assert late == [False]  # after the drain the closing thread is not special
    assert not worker.submit(_message("late"))


class _CloseWinsLock:
    """An accept lock that lets close win just before the submitter takes it."""

    def __init__(self, worker, lock):
        self._worker, self._lock = worker, lock

    def __enter__(self):
        self._worker._accepting = False
        return self._lock.__enter__()

    def __exit__(self, *exc):
        return self._lock.__exit__(*exc)


def test_a_drain_message_racing_close_is_still_handled():
    seen = []
    worker = None

    def handle(message):
        seen.append(message.msg)
        if message.msg == "send":
            worker._accept_lock = _CloseWinsLock(worker, worker._accept_lock)
            assert worker.submit(_message("send failed"))
            worker._accept_lock = worker._accept_lock._lock

    worker = CacheLogWorker(handle)
    worker.submit(_message("send"))
    worker.wait_until_idle()
    assert seen == ["send", "send failed"]
    worker._accepting = True  # undo the simulated close, then close for real
    worker.close()
