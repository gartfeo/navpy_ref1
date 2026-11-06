import threading

from navpy.core.approach.logs.navigation_stream_worker import (
    NavigationStreamBacklogError,
    NavigationStreamWorker,
)


def test_submit_returns_while_stream_sink_is_blocked():
    entered = threading.Event()
    release = threading.Event()
    handled: list[str] = []

    def write(line: str) -> None:
        entered.set()
        assert release.wait(timeout=2.0)
        handled.append(line)

    worker = NavigationStreamWorker(write)
    assert worker.submit("row")
    assert entered.wait(timeout=2.0)
    release.set()
    worker.wait_until_idle()
    worker.close()
    assert handled == ["row"]


def test_drain_fences_prior_stream_rows_in_fifo_order():
    entered = threading.Event()
    release = threading.Event()
    drained = threading.Event()
    handled: list[str] = []

    def write(line: str) -> None:
        if line == "first":
            entered.set()
            assert release.wait(timeout=2.0)
        handled.append(line)

    worker = NavigationStreamWorker(write)
    assert worker.submit("first")
    assert worker.submit("second")
    assert entered.wait(timeout=2.0)
    drain_thread = threading.Thread(
        target=lambda: (worker.wait_until_idle(), drained.set())
    )
    drain_thread.start()
    assert not drained.is_set()
    release.set()
    assert drained.wait(timeout=2.0)
    drain_thread.join(timeout=2.0)
    worker.close()
    assert handled == ["first", "second"]


def test_close_drains_fifo_and_rejects_later_rows():
    handled: list[str] = []
    worker = NavigationStreamWorker(handled.append)
    assert worker.submit("first")
    assert worker.submit("second")

    worker.close()

    assert handled == ["first", "second"]
    assert not worker.submit("late")


def test_sink_failure_is_retained_without_killing_stream_worker():
    handled: list[str] = []
    entered = threading.Event()
    release = threading.Event()

    def write(line: str) -> None:
        if line == "bad":
            entered.set()
            assert release.wait(timeout=2.0)
            raise OSError("disk unavailable")
        handled.append(line)

    reported: list[BaseException] = []
    worker = NavigationStreamWorker(write, on_error=reported.append)
    assert worker.submit("bad")
    assert entered.wait(timeout=2.0)
    assert worker.submit("good")
    release.set()
    worker.wait_until_idle()
    # Todo 290: the failed row is skipped and reported; rows submitted after
    # the failure are still written.
    assert worker.submit("after")
    worker.close()

    assert isinstance(worker.first_error, OSError)
    assert str(worker.first_error) == "disk unavailable"
    assert reported == [worker.first_error]
    assert handled == ["good", "after"]


def test_a_failing_error_reporter_does_not_stop_the_worker():
    handled: list[str] = []

    def write(line: str) -> None:
        if line == "bad":
            raise OSError("disk unavailable")
        handled.append(line)

    def report(_error: BaseException) -> None:
        raise RuntimeError("logger broke")

    worker = NavigationStreamWorker(write, on_error=report)
    assert worker.submit("bad")
    assert worker.submit("good")
    worker.close()

    assert handled == ["good"]
    assert isinstance(worker.first_error, OSError)


def test_backlog_overflow_drops_only_the_new_row_and_keeps_accepting():
    entered = threading.Event()
    release = threading.Event()
    handled: list[str] = []

    def write(line: str) -> None:
        entered.set()
        assert release.wait(timeout=2.0)
        handled.append(line)

    worker = NavigationStreamWorker(write, max_pending=1)
    assert worker.submit("owned")
    assert entered.wait(timeout=2.0)

    try:
        worker.submit("overflow")
    except NavigationStreamBacklogError as error:
        assert "capacity 1" in str(error)
    else:
        raise AssertionError("navigation backlog overflow was accepted")

    release.set()
    worker.wait_until_idle()
    # Todo 290: the overflow does not stop the worker.
    assert worker.submit("later")
    worker.close()
    assert handled == ["owned", "later"]
    assert isinstance(worker.first_error, NavigationStreamBacklogError)


def test_drain_times_out_instead_of_hanging_on_stalled_sink():
    entered = threading.Event()
    release = threading.Event()

    def write(_line: str) -> None:
        entered.set()
        assert release.wait(timeout=2.0)

    worker = NavigationStreamWorker(write)
    assert worker.submit("row")
    assert entered.wait(timeout=2.0)

    try:
        try:
            worker.wait_until_idle(timeout_s=0.01)
        except TimeoutError as exc:
            assert "did not drain" in str(exc)
        else:
            raise AssertionError("stalled navigation sink did not time out")
    finally:
        release.set()
        worker.wait_until_idle()
        worker.close()


def test_worker_start_failure_is_rethrown_on_every_later_submit(monkeypatch):
    worker = NavigationStreamWorker(lambda _row: None)
    failure = RuntimeError("thread unavailable")

    def fail_start(_thread) -> None:
        raise failure

    monkeypatch.setattr(threading.Thread, "start", fail_start)
    for row in ("first", "second"):
        try:
            worker.submit(row)
        except RuntimeError as exc:
            assert exc is failure
        else:
            raise AssertionError("worker start failure became silent")
