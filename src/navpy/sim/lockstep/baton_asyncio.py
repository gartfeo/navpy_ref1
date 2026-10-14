"""An asyncio event loop under the baton (the ASGI call: :mod:`lockstep_asgi`).

The ASGI contract (``docs/design/lockstep_contracts.md``): the backend's loop
runs in one managed thread. ``loop.time()`` is the runtime clock; the idle
wait in ``select`` parks on the baton until the next asyncio timer or a
wake-up; ``call_soon_threadsafe`` (the self-pipe write) wakes it through the
runtime; the default executor runs each job in a managed thread. asyncio's
own order stays: ready callbacks first-in first-out, timers by ``when``.

asyncio timers follow the runtime's tick order: they become due only in the
timer phase, and exactly at ``when <= t_k`` (asyncio's clock-resolution
slack is replaced), so an input at a timer's tick is handled first.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import math
import selectors
from collections.abc import Callable
from typing import Any

from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton import MAX_TIMER_ROUNDS_PER_TICK, BatonRuntime, ManagedThread
from navpy.sim.lockstep.baton_jobs import JobFailures
from navpy.sim.lockstep.baton_sync import BatonEvent, join


class _BatonSelector(selectors.BaseSelector):
    """Never touches the host: ``select`` parks until a wake-up or timeout."""

    def __init__(self, runtime: BatonRuntime, wake: BatonEvent) -> None:
        self._runtime = runtime
        self._wake = wake
        self._map: dict[Any, selectors.SelectorKey] = {}
        self.loop: "BatonEventLoop | None" = None
        self._zero_rounds = (-1, 0)  # (tick time, zero-timeout timer rounds)

    def register(self, fileobj: Any, events: int, data: Any = None) -> selectors.SelectorKey:
        key = selectors.SelectorKey(fileobj, id(fileobj), events, data)
        self._map[fileobj] = key
        return key

    def unregister(self, fileobj: Any) -> selectors.SelectorKey:
        return self._map.pop(fileobj)

    def get_map(self) -> dict[Any, selectors.SelectorKey]:
        return self._map

    def select(self, timeout: float | None = None) -> list[tuple[selectors.SelectorKey, int]]:
        loop = self.loop
        has_ready = bool(loop._ready)
        timer_due = bool(loop._scheduled) and loop._scheduled[0]._when <= loop.time()
        if timer_due and self._runtime.in_timer_phase:
            self._count_zero_round()  # every due-timer round, ready callbacks or not
        if not has_ready and not self._wake.is_set():
            if not (timeout == 0 and self._runtime.in_timer_phase):
                # Idle, or a timer is due but this tick's inputs still run:
                # park; a zero wait is due now and fires in the timer phase.
                self._wake.wait(timeout)
        if self._wake.is_set():
            self._wake.clear()
        # asyncio promotes a timer when ``when < time() + _clock_resolution``.
        # One ulp makes that exactly ``when <= t_k``; minus infinity holds
        # every timer back while this tick's inputs run.
        now = loop.time()
        loop._clock_resolution = (
            math.nextafter(now, math.inf) - now if self._runtime.in_timer_phase else -math.inf
        )
        return []

    def _count_zero_round(self) -> None:
        """A due asyncio timer runs without parking; bound how often per tick."""
        now_us, rounds = self._zero_rounds
        rounds = rounds + 1 if now_us == self._runtime.now_us else 1
        self._zero_rounds = (self._runtime.now_us, rounds)
        if rounds > MAX_TIMER_ROUNDS_PER_TICK:
            self._runtime.fail(
                f"asyncio timers still due after {MAX_TIMER_ROUNDS_PER_TICK} rounds"
            )

    def close(self) -> None:
        self._map.clear()


def _cancel_running(future: concurrent.futures.Future) -> None:
    """End a running future as cancelled (``Future.cancel`` refuses a running one)."""
    with future._condition:
        future._state = concurrent.futures._base.CANCELLED_AND_NOTIFIED
        future._condition.notify_all()
    future._invoke_callbacks()


class BatonExecutor(concurrent.futures.ThreadPoolExecutor):
    """Each job runs in its own managed thread; the pool's own workers never start.

    A job runs in a copy of its submitter's context (the runtime's spawn), so
    it shares the submitting task's cancellation scope; a ``Cancelled`` of
    that scope ends its future cancelled.

    A ``ThreadPoolExecutor`` subclass only because asyncio's
    ``set_default_executor`` requires one.
    """

    def __init__(self, runtime: BatonRuntime) -> None:
        super().__init__(max_workers=1)
        self._runtime = runtime
        self._count = 0
        self._jobs: list[tuple[concurrent.futures.Future, ManagedThread]] = []
        self._closed = False
        #: Called before each submission; raises to refuse it (a shutdown
        #: admits only its own owner's jobs).
        self.gate: Callable[[], None] | None = None

    def submit(self, fn: Callable[..., Any], /, *args: Any, **kwargs: Any) -> concurrent.futures.Future:
        if self._closed:
            raise RuntimeError("cannot schedule new futures after shutdown")
        if self.gate is not None:
            self.gate()
        self._jobs = [(f, t) for f, t in self._jobs if not t.done]  # retire finished jobs
        future: concurrent.futures.Future = concurrent.futures.Future()
        self._count += 1

        def run() -> None:
            if not future.set_running_or_notify_cancel():
                return
            try:
                future.set_result(fn(*args, **kwargs))
            except BaseException as error:
                if cancel_scope.consumed(error):
                    _cancel_running(future)  # its scope was cancelled: cancelled, not failed
                else:
                    future.set_exception(error)

        thread = self._runtime.spawn(run, name=f"executor-{self._count}")
        self._jobs.append((future, thread))
        return future

    def job_threads(self) -> list[ManagedThread]:
        """The managed threads of the jobs not yet retired."""
        return [thread for _future, thread in self._jobs]

    def pending(self) -> list[concurrent.futures.Future]:
        return [future for future, _thread in self._jobs if not future.done()]

    def shutdown(self, wait: bool = True, *, cancel_futures: bool = False) -> None:
        """Refuse new jobs; with ``wait``, join every job (on the baton)."""
        self._closed = True
        if cancel_futures:
            for future in self.pending():
                future.cancel()
        if wait:
            for _future, thread in self._jobs:
                join(self._runtime, thread)


class BatonEventLoop(asyncio.SelectorEventLoop):
    @property
    def baton_executor(self) -> BatonExecutor:
        return self._baton_executor

    @property
    def baton(self) -> BatonRuntime:
        return self._baton_runtime

    def __init__(self, runtime: BatonRuntime) -> None:
        self._baton_runtime = runtime
        self._baton_wake = BatonEvent(runtime)
        selector = _BatonSelector(runtime, self._baton_wake)
        super().__init__(selector)
        selector.loop = self
        self._baton_executor = BatonExecutor(runtime)
        self.set_default_executor(self._baton_executor)
        #: Each failed job's error and whether its awaiter took it
        #: (:mod:`baton_jobs`); the owner sets ``job_failures.report``.
        self.job_failures = JobFailures(self)

    def run_in_executor(self, executor: Any, func: Callable[..., Any], *args: Any) -> asyncio.Future:
        """asyncio's, on the managed executor, with each job's failure observed."""
        if executor is not None and executor is not self._baton_executor:
            return super().run_in_executor(executor, func, *args)
        self._check_closed()
        return self.job_failures.wrap(self._baton_executor.submit(func, *args))

    def time(self) -> float:
        return self._baton_runtime.monotonic()

    # No self-pipe: wake-ups go through the runtime, never a host socket.
    def _make_self_pipe(self) -> None:
        self._ssock = self._csock = None
        self._internal_fds += 1

    def _close_self_pipe(self) -> None:
        self._internal_fds -= 1

    def _write_to_self(self) -> None:  # call_soon_threadsafe's wake-up
        self._baton_wake.set()

    async def shutdown_default_executor(self, timeout: float | None = None) -> None:
        """Wait for the managed jobs on the loop; no helper thread."""
        self._baton_executor.shutdown(wait=False)  # refuse new jobs first
        pending = [asyncio.wrap_future(future) for future in self._baton_executor.pending()]
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)

    def deliver(self, callback: Callable[[], Any]) -> None:
        """The coordinator's input path: run ``callback`` on the loop."""
        self._baton_runtime.require_coordinator()
        self._call_soon(callback, (), None)
        self._baton_wake.deliver_set()


__all__ = ["BatonEventLoop", "BatonExecutor"]
