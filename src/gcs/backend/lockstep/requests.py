"""Operator requests on the backend loop, each a task in its own cancellation scope.

A request's payload names its kind:

- ``{"kind": "http", "method", "target", "json"?, "deadline_s"?}``: one
  ``asgi_request`` through the app. Its answer is ``{"status", "body"}`` (the
  body as text); 503 once the fence closed admission.
- ``{"kind": "subscribe"}``: install a WS subscriber (the subscription id is
  the request id), send its initial snapshot, answer ``{"subscribed": id}``.
- ``{"kind": "unsubscribe", "subscription": id}``: remove it.

Each runs as a task created in a fresh :class:`CancelScope` keyed by its wire
request id, so its executor jobs and their child threads share the scope. A
``cancels[]`` entry cancels exactly that scope (waking its parked waits) and
the task; the request then answers ``{"cancelled": true}``. A request past
its sim-time deadline is cancelled the same way and answers
``{"deadline": true}``. A response stands: a cancel after it does nothing.
Any other end (an unexpected exception, a ``Cancelled`` of a scope nobody
cancelled) is unanswered and latches a fault (:mod:`.faults`); so does a
thread other than its executor jobs still alive in its scope when it ends.
"""

from __future__ import annotations

import asyncio
import contextvars
from typing import Any

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.lockstep.event_sink import EventSink
from gcs.backend.lockstep.faults import FaultLatch
from gcs.backend.producer_inventory import ProducerInventory
from gcs.backend.routes.telemetry import send_initial_state
from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton import is_abort
from navpy.sim.lockstep.baton_asyncio import BatonEventLoop
from navpy.sim.lockstep.cancel_scope import CancelScope
from navpy.sim.lockstep.lockstep_asgi import AsgiRequest, asgi_request
from navpy.sim.lockstep.part_wire import Request, RequestId, Response
from navpy.sim.lockstep.participant_hooks import OutboxBuffer

#: The answer to a request after the fence closed admission.
FENCED = {"status": 503, "body": '{"detail":"backend shutting down"}'}


class RequestRunner:
    def __init__(self, loop: BatonEventLoop, app: Any, sink: EventSink, out: OutboxBuffer,
                 inventory: ProducerInventory) -> None:
        self._loop = loop
        self._app = app
        self._sink = sink
        self._out = out
        self._inventory = inventory
        self._faults = FaultLatch(out)
        self._pending: dict[RequestId, tuple[asyncio.Task, CancelScope]] = {}

    def start(self, request: Request) -> None:
        """On the loop: start the request's task in its own scope."""
        scope = CancelScope(f"request {request.id[0]}.{request.id[1]}")
        context = cancel_scope.bind(contextvars.copy_context(), scope)
        task = context.run(self._loop.create_task, self._serve(request, scope))
        self._pending[request.id] = (task, scope)

    def cancel(self, request_id: RequestId) -> None:
        """Coordinator, input phase: cancel exactly this request's scope and task."""
        entry = self._pending.get(request_id)
        if entry is not None and entry[1].cancel():
            self._loop.deliver(entry[0].cancel)

    def cancel_all(self) -> list[asyncio.Task]:
        """On the loop (shutdown): cancel every pending request; return their tasks."""
        tasks = []
        for task, scope in list(self._pending.values()):
            if scope.cancel():
                task.cancel()
            tasks.append(task)
        return tasks

    async def _serve(self, request: Request, scope: CancelScope) -> None:
        try:
            payload = await self._outcome(request, scope)
        except BaseException as error:
            if is_abort(error):
                raise
            # Unanswered: a fault, never an outcome. The latch observes the
            # error, so the task ends without an unretrieved exception.
            self._faults.latch(f"request {request.id[0]}.{request.id[1]}", error)
            return
        finally:
            del self._pending[request.id]
            scope.close()
            self._check_leaks(request.id, scope)
        self._out.responses.append(Response(request.id, payload))

    def _check_leaks(self, request_id: RequestId, scope: CancelScope) -> None:
        """No thread outlives its request in the request's scope, except the
        request's own executor jobs: a long-lived worker belongs to its
        component (``rt.detached``), or a request's cancel would stop it."""
        jobs = self._loop.baton_executor.job_threads()
        for thread in self._loop.baton.threads.alive():
            if thread.scope is scope and thread not in jobs:
                self._faults.latch(f"request {request_id[0]}.{request_id[1]} thread {thread.name}",
                                   "ScopeLeak")

    async def _outcome(self, request: Request, scope: CancelScope) -> dict[str, Any]:
        try:
            return await self._handle(request)
        except asyncio.CancelledError:
            if not scope.cancelled:
                raise
            return {"cancelled": True}
        except TimeoutError:
            scope.cancel()  # the deadline: its jobs stop at their next wait
            return {"deadline": True}
        except cancel_scope.Cancelled as error:
            if not cancel_scope.consumed(error):
                raise
            return {"cancelled": True}

    async def _handle(self, request: Request) -> dict[str, Any]:
        payload = request.payload
        kind = payload.get("kind")
        if kind == "http":
            return await self._http(request.id, payload)
        if kind == "subscribe":
            subscriber = self._sink.subscriber(request.id)
            await ws_manager.connect(subscriber)
            await send_initial_state(subscriber)
            return {"subscribed": list(request.id)}
        if kind == "unsubscribe":
            subscription = tuple(payload["subscription"])
            subscriber = self._sink.subscribers.pop(subscription, None)
            if subscriber is not None:
                await ws_manager.disconnect(subscriber)
            return {"unsubscribed": subscriber is not None}
        raise ValueError(f"unknown request kind {kind!r}")

    async def _http(self, request_id: RequestId, payload: dict[str, Any]) -> dict[str, Any]:
        if not self._inventory.admitting:
            return dict(FENCED)
        options: dict[str, Any] = {"deadline_s": payload.get("deadline_s"), "request_id": request_id}
        if "json" in payload:
            options["json"] = payload["json"]
        response = await asgi_request(
            self._app, AsgiRequest(payload["method"], payload["target"], **options),
        )
        return {"status": response.status, "body": response.body.decode("utf-8")}


__all__ = ["FENCED", "RequestRunner"]
