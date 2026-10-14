"""Ownership inventory of everything in the backend that can make work or events.

Each producer registers what it starts here: the asyncio tasks it creates,
the threads it starts, the message subscriptions it holds and the broadcast
futures it schedules. On the wall nothing reads the inventory, so behavior is
unchanged. Under lockstep the strict shutdown's fence
(``gcs.backend.lockstep.fence``, ``lockstep_slice.md`` "Shutdown in drain")
closes admission and stops every producer by name; work admitted before the
fence runs to completion or its cancellation.

:data:`PRODUCERS` names every producer the fence stops, and
``tests/gcs/backend/test_producer_inventory.py`` compares it with the sites
in the code that start work, so a new producer fails until it is fenced.
"""
from __future__ import annotations

import asyncio
import contextvars
from collections.abc import Callable, Coroutine
from contextlib import AbstractContextManager
from typing import Any, Protocol

from navpy.core.kernel import runtime_time as rt

#: Producer -> what it is; the fence stops each (``gcs.backend.lockstep.fence``).
PRODUCERS = {
    "http_routes": "new requests (answered 503 once fenced)",
    "telemetry_loop": "the telemetry broadcast loop and its executor work",
    "autoconnect": "the startup auto-connect task",
    "launch_tasks": "the launch controller's per-vehicle tasks",
    "mission_probe": "the connect-time mission probe threads",
    "task_confirm_listener": "task confirmation message subscriptions and broadcasts",
    "task_assign_listener": "task assignment message subscriptions and broadcasts",
    "compass_cal_listener": "compass calibration message subscriptions and broadcasts",
    "vehicle_entry_messages": "each vehicle entry's message subscriptions (status, gimbal, RC, swarm)",
    "full_param_observer": "the full-parameter store's PARAM_VALUE observer per vehicle",
    "route_broadcasts": "broadcasts a route schedules from its executor job",
    "manual_control": "the WebSocket manual-control repeater task",
    "esp32_simulator": "the ESP32 simulator's server and trigger threads (an in-process port in lockstep)",
    "companion_processes": "NavPy companion output readers (the launcher port in lockstep)",
}

class ProducerScopes(Protocol):
    """Per-producer cancellation scopes (the lockstep backend participant's)."""

    def context(self, name: str) -> contextvars.Context:
        """A context whose cancellation scope is a new one for ``name``."""

    def entered(self, name: str) -> AbstractContextManager[Any]:
        """Run a block in a new cancellation scope for ``name``."""

    def observe(self, producer: str, task: asyncio.Task | ScheduledFuture) -> None:
        """A done callback: a task's or broadcast's unexpected end is a fault."""


class ProducerFenced(RuntimeError):
    """A producer tried to start work after the fence closed admission."""


class ScheduledFuture(Protocol):
    """What ``run_coroutine_threadsafe`` returns, as far as the inventory reads it."""

    def done(self) -> bool: ...


class ProducerInventory:
    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._admitting = True
        self._tasks: list[tuple[str, asyncio.Task]] = []
        self._threads: list[tuple[str, Any]] = []
        self._subscriptions: list[tuple[str, Any]] = []
        self._futures: list[tuple[str, ScheduledFuture]] = []
        #: Under lockstep each tracked task and thread runs in its own
        #: cancellation scope (set by the backend participant); None on the wall.
        self.scopes: ProducerScopes | None = None

    @property
    def admitting(self) -> bool:
        return self._admitting

    def close_admission(self) -> None:
        with self._lock:
            self._admitting = False

    def _admit(self, producer: str) -> None:
        if producer not in PRODUCERS:
            raise KeyError(f"unknown producer {producer!r}: add it to PRODUCERS and the fence")
        if not self._admitting:
            raise ProducerFenced(f"{producer} started work after the fence")

    def create_task(self, producer: str, coroutine: Coroutine[Any, Any, Any]) -> asyncio.Task:
        """``asyncio.create_task``, tracked, in the producer's own scope."""
        loop = asyncio.get_running_loop()
        context = None if self.scopes is None else self.scopes.context(producer)
        with self._lock:
            try:
                self._admit(producer)
            except BaseException:
                coroutine.close()
                raise
            # On the wall no context is passed (``create_task(context=)`` is 3.11+).
            task = (loop.create_task(coroutine) if context is None
                    else context.run(loop.create_task, coroutine))
            scopes = self.scopes
            if scopes is not None:
                task.add_done_callback(lambda done: scopes.observe(producer, done))
            self._tasks = [(p, t) for p, t in self._tasks if not t.done()]
            self._tasks.append((producer, task))
        return task

    def start_thread(self, producer: str, target: Callable[[], object], *, name: str,
                     daemon: bool = True) -> Any:
        """Start ``target`` on a tracked thread, in the producer's own scope."""
        scopes = self.scopes

        def body() -> None:
            with scopes.entered(name):
                target()

        with self._lock:
            self._admit(producer)
            # On the wall the target runs as before; under lockstep in its scope.
            thread = rt.Thread(target=target if scopes is None else body, name=name, daemon=daemon)
            self._threads = [(p, t) for p, t in self._threads if t.is_alive()]
            self._threads.append((producer, thread))
        with rt.detached():  # its own producer scope, never its creator's (wall: no-op)
            thread.start()
        return thread

    def keep_subscription(self, producer: str, subscription: Any) -> Any:
        """Hold ``subscription`` for the fence; on the wall it is not held (no fence)."""
        with self._lock:
            self._admit(producer)
            if self.scopes is not None:
                self._subscriptions.append((producer, subscription))
        return subscription

    def broadcast(self, producer: str, coroutine: Coroutine[Any, Any, Any],
                  loop: asyncio.AbstractEventLoop) -> ScheduledFuture:
        """``run_coroutine_threadsafe`` with the future tracked until it is done."""
        with self._lock:
            try:
                self._admit(producer)
            except BaseException:
                coroutine.close()
                raise
            future = asyncio.run_coroutine_threadsafe(coroutine, loop)
            scopes = self.scopes
            if scopes is not None:  # nobody awaits a broadcast: its failure is a fault at once
                future.add_done_callback(lambda done: scopes.observe(producer, done))
            self._futures = [(p, f) for p, f in self._futures if not f.done()]
            self._futures.append((producer, future))
        return future

    def tasks(self, producer: str | None = None) -> list[asyncio.Task]:
        return [t for p, t in self._tasks if producer in (None, p)]

    def threads(self, producer: str | None = None) -> list[Any]:
        return [t for p, t in self._threads if producer in (None, p)]

    def futures(self) -> list[ScheduledFuture]:
        return [f for _p, f in self._futures if not f.done()]

    def unsubscribe(self, producer: str) -> int:
        """Cancel every subscription ``producer`` holds; returns how many."""
        with self._lock:
            mine = [s for p, s in self._subscriptions if p == producer]
            self._subscriptions = [(p, s) for p, s in self._subscriptions if p != producer]
        for subscription in mine:
            subscription.cancel()
        return len(mine)


producers = ProducerInventory()


__all__ = ["PRODUCERS", "ProducerFenced", "ProducerInventory", "ProducerScopes", "producers"]
