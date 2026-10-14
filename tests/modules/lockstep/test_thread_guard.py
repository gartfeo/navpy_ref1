"""The thread guard's negative controls: an unmanaged thread is a fault at creation."""

from __future__ import annotations

import _thread
import threading

import pytest

from navpy.sim.lockstep.baton import BatonRuntime, LockstepFault, audit
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.thread_guard import install_thread_guard

TICK_US = 20_000


@pytest.fixture
def guarded():
    runtime = LockstepRuntime.create(BatonRuntime(), epoch_us=0, seed=1, ordinal=1)
    baseline = set(threading.enumerate())
    uninstall = install_thread_guard(runtime.baton)
    try:
        yield runtime, baseline
    finally:
        uninstall()


def _fault_from(runtime: LockstepRuntime, start) -> str:
    """Start an unmanaged thread from a managed one; return the latched fault."""
    runtime.baton.spawn(start, name="product")
    with pytest.raises(LockstepFault) as fault:
        runtime.baton.run_until_quiescent()
    with pytest.raises(LockstepFault):  # latched: the next tick refuses too
        runtime.baton.tick(TICK_US)
    return str(fault.value)


def test_a_thread_that_would_finish_within_the_tick_is_a_fault_at_creation(guarded):
    runtime, baseline = guarded
    ran = []
    stray = threading.Thread(target=ran.append, args=("ran",), name="quick")
    assert "unmanaged thread started: quick" in _fault_from(runtime, stray.start)
    assert ran == [] and set(threading.enumerate()) == baseline


def test_a_thread_that_would_outlive_the_tick_is_a_fault_at_creation(guarded):
    runtime, baseline = guarded
    release = threading.Event()
    stray = threading.Thread(target=release.wait, name="lingering", daemon=True)

    def start_and_swallow() -> None:
        try:
            stray.start()
        except LockstepFault:
            pass  # swallowed by the product: still latched

    assert "unmanaged thread started: lingering" in _fault_from(runtime, start_and_swallow)
    assert set(threading.enumerate()) == baseline


def test_a_raw_start_new_thread_is_a_fault(guarded):
    runtime, baseline = guarded
    ran = []
    fault = _fault_from(runtime, lambda: _thread.start_new_thread(ran.append, ("ran",)))
    assert "_thread.start_new_thread" in fault
    assert ran == [] and set(threading.enumerate()) == baseline


def test_the_start_alias_threading_cached_is_a_fault(guarded):
    runtime, baseline = guarded
    ran = []
    start = threading._start_new_thread  # the alias ``Thread.start`` itself calls
    fault = _fault_from(runtime, lambda: start(ran.append, ("ran",)))
    assert "threading._start_new_thread" in fault
    assert ran == [] and set(threading.enumerate()) == baseline


def test_a_look_alike_of_the_baton_os_thread_is_a_fault(guarded):
    """Same class and name as a spawned OS thread, but not one ``spawn`` is starting."""
    runtime, baseline = guarded
    ran = []
    fake = threading.Thread(target=ran.append, args=("ran",), name="lockstep-product", daemon=True)
    assert "unmanaged thread started: lockstep-product" in _fault_from(runtime, fake.start)
    assert ran == [] and set(threading.enumerate()) == baseline


def test_a_subclass_bypassing_thread_start_is_a_fault(guarded):
    """A start that skips ``Thread.start`` still meets the guarded alias."""
    runtime, baseline = guarded
    ran = []

    class Bypass(threading.Thread):
        def start(self) -> None:
            threading._start_new_thread(self._bootstrap, ())

    stray = Bypass(target=ran.append, args=("ran",), name="bypass", daemon=True)
    assert "threading._start_new_thread" in _fault_from(runtime, stray.start)
    assert ran == [] and set(threading.enumerate()) == baseline


def test_the_coordinator_cannot_start_one_either(guarded):
    runtime, _baseline = guarded
    with pytest.raises(LockstepFault, match="unmanaged thread started: host"):
        threading.Thread(target=lambda: None, name="host").start()


def test_managed_threads_and_timers_pass(guarded):
    runtime, baseline = guarded
    threads = runtime.threads
    ran = []

    def product() -> None:
        thread = threads.Thread(target=ran.append, args=("thread",))
        thread.start()
        threads.Timer(0.02, ran.append, args=("timer",)).start()
        thread.join()

    runtime.baton.spawn(product, name="product")
    runtime.baton.run_until_quiescent()
    audit(runtime.baton, baseline)
    runtime.baton.tick(TICK_US)
    audit(runtime.baton, baseline)
    assert ran == ["thread", "timer"]


def test_uninstall_restores_the_host_starts():
    runtime = LockstepRuntime.create(BatonRuntime(), epoch_us=0, seed=1, ordinal=1)
    original = threading.Thread.start, threading._start_new_thread, _thread.start_new_thread
    install_thread_guard(runtime.baton)()
    assert (threading.Thread.start, threading._start_new_thread, _thread.start_new_thread) == original
    thread = threading.Thread(target=lambda: None)
    thread.start()
    thread.join()
