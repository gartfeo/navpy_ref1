"""Baton scheduler: real threads, one runnable at a time, on a sim clock.

A lockstep prototype (step 6 of ``docs/design/deterministic_demo_eval.md``,
contract in ``docs/design/lockstep_contracts.md``). Managed threads are real OS
threads, but each runs only while it holds the baton. A thread gives the
baton back when it parks (lock contention, condition, event, queue, sleep,
join) or ends. The coordinator, the thread that created the runtime, owns
time: :meth:`BatonRuntime.tick` sets ``t_k``, delivers inputs, runs to
quiescence and then fires due timers.

Order is deterministic: runnable threads run first-in first-out, timers fire
in ``(due, park seq)`` order, and since only one thread runs at a time every
wake-up order follows. Primitive state needs no OS lock: nothing else runs
while the baton holder (or the coordinator, between handoffs) touches it.

Deadlines are the product's own float arithmetic: a wait of ``d`` seconds
started at ``t0`` is due at the first tick with ``t_k >= t0 + d`` in float
seconds, the comparison the product makes when it re-checks. So a plain sleep
never wakes early, and a product deadline that float drift puts just after
``t_k`` waits for the next tick instead of re-arming forever.
"""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable
from typing import NoReturn

from navpy.sim.lockstep.baton_parts import ManagedThread, ThreadTable, TimerHeap


#: Rounds of "fire due timers, run to quiescence" one tick may take. Only a
#: zero wait comes back due at the same time, so a thread re-arming one this
#: often is busy-waiting, which is a fault.
MAX_TIMER_ROUNDS_PER_TICK = 1000
#: Wall budget for one baton holder to give the baton back. It supervises the
#: host only: a breach means a managed thread blocked on a host primitive
#: (a real lock, socket or sleep), a host fault, never a different outcome.
BATON_WALL_BUDGET_S = 30.0


class LockstepFault(RuntimeError):
    """A contract violation: a host fault, never a different outcome."""


class LockstepAborted(BaseException):
    """Raised in parked threads when the run is aborted, so they unwind."""


class BatonRuntime:
    """The scheduler; primitives come from :mod:`navpy.sim.lockstep.baton_sync`."""

    def __init__(self, start_us: int = 0) -> None:
        self._coordinator = threading.get_ident()
        self._now_us = int(start_us)
        self._runnable: deque[ManagedThread] = deque()
        self._yielded = threading.Semaphore(0)
        self._current: ManagedThread | None = None
        self._scheduling = False
        self._aborted = False
        self._fault: LockstepFault | None = None
        self._in_timer_phase = True
        self.threads = ThreadTable()
        self.timers = TimerHeap()

    # -- time ---------------------------------------------------------------
    @property
    def now_us(self) -> int:
        return self._now_us

    def monotonic(self) -> float:
        return self._now_us / 1_000_000

    @property
    def in_timer_phase(self) -> bool:
        """False while a tick's inputs run; timers (asyncio's too) wait for it."""
        return self._in_timer_phase

    @property
    def aborted(self) -> bool:
        return self._aborted

    def tick(self, now_us: int, deliver: Callable[[], None] | None = None) -> None:
        """One tick in contract order: time, inputs to quiescence, then timers."""
        self.require_coordinator()
        if now_us < self._now_us:
            self.fail(f"time went back: {now_us} < {self._now_us}")
        self._now_us = int(now_us)
        self._in_timer_phase = False
        try:
            if deliver is not None:
                deliver()
            self.run_until_quiescent()
        finally:
            self._in_timer_phase = True
        self.settle_timers()

    def settle_timers(self) -> None:
        """Fire due timers and run to quiescence until nothing is due now."""
        for _round in range(MAX_TIMER_ROUNDS_PER_TICK):
            if not self.timers.any_due(self.monotonic()):
                return
            for thread in self.timers.pop_due(self.monotonic()):
                self.wake(thread)
            self.run_until_quiescent()
        self.fail(f"timers still due after {MAX_TIMER_ROUNDS_PER_TICK} rounds")

    def fail(self, message: str) -> NoReturn:
        """Latch a contract fault; scheduling raises it even if a caller swallows it."""
        if self._fault is None:
            self._fault = LockstepFault(message)
        raise LockstepFault(message)

    # -- threads ------------------------------------------------------------
    def spawn(self, target: Callable[[], object], *, name: str) -> ManagedThread:
        """Start a managed thread; it first runs at the next scheduling step."""
        if threading.get_ident() != self._coordinator:
            self.current()  # a managed thread may spawn only while it holds the baton
        thread = self.threads.new(name)

        def run() -> None:
            thread.go.acquire()
            try:
                if not self._aborted:
                    target()
            except LockstepAborted:
                pass
            except BaseException as error:  # reported by run_until_quiescent
                self.threads.errors.append(error)
            finally:
                thread.done = True
                for joiner in thread.joiners:
                    self.wake(joiner)
                self._current = None
                self._yielded.release()

        thread.os_thread = threading.Thread(target=run, name=f"lockstep-{name}", daemon=True)
        thread.os_thread.start()
        self.threads.add(thread)
        self._runnable.append(thread)
        return thread

    def run_until_quiescent(self) -> int:
        """Hand the baton around until every thread is parked or done."""
        self.require_coordinator()
        if self._scheduling:
            self.fail("scheduling re-entered")
        self._scheduling = True
        steps = 0
        while self._runnable:
            thread = self._runnable.popleft()
            self._current = thread
            self.threads.switches += 1
            steps += 1
            thread.go.release()
            if not self._yielded.acquire(timeout=BATON_WALL_BUDGET_S):
                # The holder still runs: the runtime stays poisoned (no more
                # ticks, handoffs or abort); only ownership teardown is left.
                self.fail(
                    f"{thread.name} kept the baton for {BATON_WALL_BUDGET_S} s "
                    "(blocked on a host primitive?)"
                )
            if thread.done:
                self.threads.reap(thread)
            if self._fault is not None or self.threads.errors:
                self._scheduling = False
                raise self._fault or self.threads.errors.pop(0)
        self._scheduling = False
        return steps

    def abort(self) -> None:
        """Unwind every parked thread (each gets ``LockstepAborted``)."""
        self.require_coordinator(after_fault=True)
        if self._scheduling:
            raise LockstepFault("abort while a baton holder still runs: stop by ownership")
        self._aborted = True
        self._fault = None
        self.threads.errors.clear()
        for thread in self.threads.alive():
            self.wake(thread)
        self.run_until_quiescent()

    # -- parking (called by the baton holder) --------------------------------
    def current(self) -> ManagedThread:
        thread = self.threads.lookup(threading.get_ident())
        if thread is None or thread is not self._current:
            self.fail("a lockstep primitive was used off the baton")
        return thread

    def park(self, reason: str, timeout_s: float | None = None) -> None:
        """Give the baton back until woken or until the timer is due."""
        thread = self.current()
        thread.park_token += 1
        thread.parked_on = reason
        thread.has_timer = timeout_s is not None
        if timeout_s is not None:
            self.timers.push(self.monotonic() + max(0.0, timeout_s), thread)
        self._current = None
        self._yielded.release()
        thread.go.acquire()
        if self._aborted:
            raise LockstepAborted()

    def wake(self, thread: ManagedThread) -> None:
        """Make a parked thread runnable; any pending timer is cancelled."""
        if thread.parked_on is not None:
            thread.parked_on = None
            thread.park_token += 1
            self._runnable.append(thread)

    def require_coordinator(self, *, after_fault: bool = False) -> None:
        if threading.get_ident() != self._coordinator:
            self.fail("only the coordinator advances time, schedules or delivers")
        if self._fault is not None and not after_fault:
            raise self._fault  # a latched fault ends the run; only abort remains


def audit(runtime: BatonRuntime, baseline: set[threading.Thread]) -> None:
    """Every live OS thread is either pre-existing or managed."""
    stray = runtime.threads.stray(baseline)
    if stray:
        runtime.fail(f"unmanaged threads: {stray}")


def check_no_deadlock(runtime: BatonRuntime) -> None:
    """At the end of a run, nothing may wait without a timer on a lock or join."""
    stuck = runtime.threads.deadlocked()
    if stuck:
        runtime.fail(f"deadlocked: {stuck}")


__all__ = [
    "BATON_WALL_BUDGET_S",
    "BatonRuntime",
    "LockstepAborted",
    "LockstepFault",
    "MAX_TIMER_ROUNDS_PER_TICK",
    "ManagedThread",
    "ThreadTable",
    "TimerHeap",
    "audit",
    "check_no_deadlock",
]
