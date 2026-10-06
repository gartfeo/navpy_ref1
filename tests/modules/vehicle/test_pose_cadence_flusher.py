"""Tests for the PoseCadenceFlusher daemon lifecycle.

These target the class directly with a stub flush callback -- no module reload,
no CSV -- so the start/stop/join/idempotent/restart/startup-failure/timeout and
atexit-balance contracts are exercised in isolation from the recorder. The
recorder-side wiring (its own dump wired in, started at import under ENABLED) is
covered by test_pose_cadence_debug.py.

Every real daemon is created through the ``make_flusher`` factory so it is
stopped even if an assertion fails mid-test, and the two failure-path tests
patch this module's OWN ``threading`` / ``atexit`` bindings (module-local
SimpleNamespaces) rather than mutating the shared stdlib modules.
"""
import threading
import time
from types import SimpleNamespace

import pytest

import navpy.modules.vehicle.pose_cadence_flusher as flusher_mod
from navpy.modules.vehicle.pose_cadence_flusher import PoseCadenceFlusher


@pytest.fixture
def make_flusher():
    """Build flushers and guarantee each real daemon is stopped after the test.

    Teardown runs each flusher's OWN stop() (which joins and unregisters its
    atexit callback), attempts EVERY created flusher even if one fails, and
    re-raises the first failure afterwards. So an assertion failure mid-test can
    neither leak a live daemon, nor leak an accumulated atexit registration, nor
    silently swallow a wedged daemon -- and, since pytest reports a finalizer
    error alongside the test's own failure, it never masks that failure.
    """
    created: list[PoseCadenceFlusher] = []

    def _make(flush, interval_s):
        flusher = PoseCadenceFlusher(flush, interval_s)
        created.append(flusher)
        return flusher

    yield _make

    failures = []
    for flusher in created:
        try:
            flusher.stop()
        except Exception as exc:  # a wedged or failed stop is a real teardown leak
            failures.append(exc)
    if failures:
        raise failures[0]


def test_start_stop_is_idempotent_and_restartable(make_flusher):
    flusher = make_flusher(lambda: None, 0.25)
    flusher.start()
    thread = flusher._thread
    assert thread is not None and thread.is_alive()
    assert flusher._started is True

    # A second start while running is a no-op (same generation).
    flusher.start()
    assert flusher._thread is thread

    # stop joins the live generation and clears its handles.
    flusher.stop()
    assert not thread.is_alive()
    assert flusher._thread is None and flusher._stop is None
    assert flusher._started is False

    # Idempotent: a second stop on an already-stopped flusher is a quiet no-op.
    flusher.stop()

    # Restart starts a fresh, independent generation.
    flusher.start()
    restarted = flusher._thread
    assert restarted is not None and restarted.is_alive()
    assert restarted is not thread
    flusher.stop()
    assert not restarted.is_alive()


def test_loop_invokes_flush_until_stopped(make_flusher):
    # The daemon loop must actually call the flush callback on its cadence, not
    # merely spin -- proven by observing at least one real invocation.
    calls: list[int] = []
    flusher = make_flusher(lambda: calls.append(1), 0.01)
    flusher.start()
    deadline = time.monotonic() + 1.0
    while not calls and time.monotonic() < deadline:
        time.sleep(0.01)
    flusher.stop()
    assert calls, "flush loop never invoked the callback"


def test_start_failure_propagates_and_leaves_clean_state(make_flusher, monkeypatch):
    # A launch that raises must propagate (never be swallowed) AND leave a clean,
    # retryable state: no handle, no atexit registration, not "started" -- so
    # stop() never joins a never-started thread and a later start() can retry.
    # Patch this module's OWN threading/atexit bindings, not the shared stdlib.
    registered: list = []
    captured: dict = {}

    class _RaisingThread:
        def __init__(self, *args, **kwargs):
            captured["stop"] = kwargs.get("args", (None,))[0]

        def start(self):
            raise RuntimeError("cannot start thread")

        def is_alive(self):
            return False  # a real Thread whose start() raised pre-launch is not alive

    monkeypatch.setattr(
        flusher_mod, "threading",
        SimpleNamespace(Thread=_RaisingThread, Event=threading.Event),
    )
    monkeypatch.setattr(
        flusher_mod, "atexit",
        SimpleNamespace(register=lambda cb: registered.append(cb)),
    )

    flusher = make_flusher(lambda: None, 0.25)
    with pytest.raises(RuntimeError, match="cannot start thread"):
        flusher.start()

    assert flusher._started is False
    assert flusher._thread is None
    assert flusher._stop is None
    assert flusher._atexit is None
    assert registered == []  # atexit is registered only after a successful launch
    # The not-alive branch signals the generation's stop event, so a
    # delayed-bootstrap launch (if this had actually launched) would exit at its
    # first loop wait rather than run unowned.
    assert captured["stop"].is_set()
    flusher.stop()  # safe no-op on the clean state


def test_start_interrupted_after_launch_adopts_and_stops(make_flusher, monkeypatch):
    # An async interrupt that lands AFTER the daemon is already running must
    # propagate yet leave the live generation adopted, so stop() can still
    # terminate and join it rather than orphaning it.
    class _LaunchedThenRaise:
        """A Thread stand-in whose start() launches the real daemon and is then
        interrupted before returning."""

        def __init__(self, *args, **kwargs):
            self._target = kwargs.get("target")
            self._args = kwargs.get("args", ())
            self._real = None

        def start(self):
            self._real = threading.Thread(
                target=self._target, args=self._args, daemon=True
            )
            self._real.start()
            raise KeyboardInterrupt("interrupted after launch")

        def is_alive(self):
            return self._real is not None and self._real.is_alive()

        def join(self, timeout=None):
            if self._real is not None:
                self._real.join(timeout)

    monkeypatch.setattr(
        flusher_mod, "threading",
        SimpleNamespace(Thread=_LaunchedThenRaise, Event=threading.Event),
    )
    monkeypatch.setattr(
        flusher_mod, "atexit",
        SimpleNamespace(register=lambda cb: None, unregister=lambda cb: None),
    )

    flusher = make_flusher(lambda: None, 0.25)
    with pytest.raises(KeyboardInterrupt):
        flusher.start()

    adopted = flusher._thread
    assert adopted is not None and adopted.is_alive()  # launched + adopted, not dropped
    assert flusher._started is True
    real = adopted._real

    flusher.stop()  # must terminate and join the adopted daemon
    assert not real.is_alive()
    assert flusher._thread is None and flusher._started is False


def test_stop_timeout_retains_handle_and_raises(make_flusher, monkeypatch):
    # A flusher that will not stop must surface as a failure with its handle
    # retained for a retry -- never reported as a clean stop.
    flusher = make_flusher(lambda: None, 0.25)

    class _Unjoinable:
        def join(self, timeout=None):
            return None

        def is_alive(self):
            return True

    fake = _Unjoinable()
    flusher._thread = fake
    flusher._stop = threading.Event()
    flusher._atexit = None
    flusher._started = True
    monkeypatch.setattr(flusher_mod, "_FLUSH_JOIN_TIMEOUT_S", 0.01)

    with pytest.raises(RuntimeError, match="did not stop"):
        flusher.stop()

    assert flusher._thread is fake  # live handle retained for a retry
    assert flusher._started is True

    # Drop the synthetic handle so the factory teardown is a clean no-op.
    flusher._thread = None
    flusher._started = False


def test_atexit_registration_is_balanced(make_flusher, monkeypatch):
    # Each started generation registers exactly one atexit callback -- the flush
    # tail-drain -- and stop() unregisters that exact callback, so reloads never
    # accumulate registrations. Patch this module's OWN atexit binding.
    registered: list = []
    unregistered: list = []
    monkeypatch.setattr(
        flusher_mod, "atexit",
        SimpleNamespace(
            register=lambda cb: registered.append(cb),
            unregister=lambda cb: unregistered.append(cb),
        ),
    )

    def flush():
        return None

    flusher = make_flusher(flush, 0.25)
    flusher.start()
    assert registered == [flush]
    assert flusher._atexit is flush

    flusher.stop()
    assert unregistered == [flush]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
