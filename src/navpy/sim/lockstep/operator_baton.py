"""The operator on its own baton, and its lockstep I/O ports (tick step 5).

:class:`OperatorBaton` is an ``OperatorHost``: it runs an operator script on
a baton in the coordinator's process. Each tick it delivers what the tick
brought (lifecycle records, responses, subscribed events), runs the baton to
quiescence at ``t_k``, and returns the requests the script made; they travel
in ``TICK k+1``.

Ports (``lockstep_slice.md``, "Operator"):

- :class:`LockstepApi` (``JsonApiPort``): a request is queued and the script
  parks until its response arrives in a ``DONE``. Its timeout is the
  request's sim-time deadline, enforced by the backend.
- :class:`LockstepEventStream` (``EventStreamPort``): ``__enter__`` sends a
  subscribe and parks until its acknowledgement, as a WebSocket connect
  returns before the caller's next step; ``receive`` returns subscribed
  deliveries in order (each receive's tick is journaled); ``__exit__``
  queues an unsubscribe and drops the deliveries not yet received, and
  those that arrive before the backend applies it; each drop is counted.
  Receipts and drops are the operator's tick observations, which the
  coordinator records in the run journal (and so in its digest).
- :class:`BatonClock` (``ClockPort``): the baton's clock.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import Any

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import BatonRuntime, ManagedThread
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import BatonEvent, sleep
from navpy.sim.lockstep.coordinator_types import OperatorCancel, OperatorInbox, OperatorRequest
from navpy.sim.lockstep.part_wire import RequestId
from navpy.sim.lockstep.participant_hooks import RequestIds

#: The backend's ``DONE`` event kind of one subscriber delivery.
DELIVERY_EVENT = "ws"


class ApiError(RuntimeError):
    """A lockstep API call failed: an unexpected status, a deadline or a cancel."""


@dataclass
class _Pending:
    done: BatonEvent
    answer: Any = None


@dataclass
class OperatorJournal:
    """What the operator saw: deliveries' receive ticks and the drops."""

    receipts: list[tuple[int, list[int]]] = field(default_factory=list)
    dropped: int = 0
    unclaimed: list[tuple[int, list[int]]] = field(default_factory=list)
    #: This tick's receipts and drops, until the coordinator takes them.
    tick: dict[str, Any] = field(default_factory=dict)
    #: Unsubscribe request id -> its subscription, until the backend answers:
    #: a delivery for it meanwhile is a late drop.
    closing: dict[RequestId, RequestId] = field(default_factory=dict)
    #: Each participant's ``DONE.log_lines`` so far, by ordinal.
    log_lines: dict[int, list[str]] = field(default_factory=dict)

    def receipt(self, k: int, subscription: RequestId) -> None:
        self.receipts.append((k, list(subscription)))
        self.tick.setdefault("receipts", []).append(list(subscription))

    def drop(self, subscription: RequestId, count: int) -> None:
        if count:
            self.dropped += count
            drops = self.tick.setdefault("dropped", {})
            key = ".".join(map(str, subscription))
            drops[key] = drops.get(key, 0) + count


class OperatorBaton:
    def __init__(self, ids: RequestIds, script: Callable[["OperatorBaton"], object]) -> None:
        self._ids = ids
        self._script = script
        self._baton: BatonRuntime | None = None
        self._thread: ManagedThread | None = None
        self._outbox: list[OperatorRequest | OperatorCancel] = []
        self._pending: dict[RequestId, _Pending] = {}
        self._ready: dict[int, BatonEvent] = {}
        self.streams: dict[RequestId, "LockstepEventStream"] = {}
        self.journal = OperatorJournal()
        self.tick = 0
        self.result: object = None
        self.done = False

    @property
    def baton(self) -> BatonRuntime:
        assert self._baton is not None
        return self._baton

    # -- the coordinator side -------------------------------------------------------
    def run_tick(self, inbox: OperatorInbox) -> tuple[OperatorRequest | OperatorCancel, ...]:
        self.tick = inbox.k
        if self._baton is None:
            self._baton = BatonRuntime(start_us=inbox.t_us)
            self._thread = self._baton.spawn(self._run, name="operator-script")
        self._baton.tick(inbox.t_us, lambda: self._deliver(inbox))
        self.done = self._thread.done
        out, self._outbox = self._outbox, []
        return tuple(out)

    def deliver_drain(self, inbox: OperatorInbox) -> None:
        """Drain: the script has returned; its ports still take the deliveries."""
        self.tick = inbox.k
        self._baton.tick(inbox.t_us, lambda: self._deliver(inbox))

    def observations(self) -> dict[str, Any]:
        observed, self.journal.tick = self.journal.tick, {}
        return observed

    def _run(self) -> None:
        self.result = self._script(self)

    def _deliver(self, inbox: OperatorInbox) -> None:
        for ordinal, line in inbox.log_lines:
            self.journal.log_lines.setdefault(ordinal, []).append(line)
        for entry in inbox.lifecycle:
            if entry.kind == "ready":
                self._ready_event(entry.ordinal).deliver_set()
        for _ordinal, response in inbox.responses:
            pending = self._pending.pop(response.id, None)
            if pending is None:  # an answer nobody waits for (an unsubscribe)
                self.journal.unclaimed.append((inbox.k, list(response.id)))
                continue
            pending.answer = response.payload
            pending.done.deliver_set()
        for _ordinal, event in inbox.events:
            if event.kind == DELIVERY_EVENT:
                subscription = tuple(event.payload["subscription"])
                stream = self.streams.get(subscription)
                if stream is not None:
                    stream.queue.deliver(event.payload["data"])
                elif subscription in self.journal.closing.values():  # sent before the unsubscribe applied
                    self.journal.drop(subscription, 1)
        for _ordinal, response in inbox.responses:  # applied: no delivery follows its answer
            self.journal.closing.pop(response.id, None)

    def _ready_event(self, ordinal: int) -> BatonEvent:
        if ordinal not in self._ready:
            self._ready[ordinal] = BatonEvent(self.baton)
        return self._ready[ordinal]

    # -- the script side (on the baton) ----------------------------------------------
    def wait_ready(self, ordinal: int) -> None:
        self._ready_event(ordinal).wait()

    def send(self, target: int, payload: object, request_id: RequestId | None = None) -> RequestId:
        """Queue a request for ``TICK k+1`` without waiting for its answer."""
        request_id = request_id or self._ids.next()
        self._outbox.append(OperatorRequest(target, request_id, payload))
        return request_id

    def call(self, target: int, payload: object, request_id: RequestId | None = None) -> Any:
        """Queue a request and park until its response."""
        request_id = request_id or self._ids.next()
        pending = self.expect(request_id)
        self.send(target, payload, request_id)
        return self.await_answer(pending)

    def expect(self, request_id: RequestId) -> _Pending:
        """Register for ``request_id``'s answer (before the request leaves)."""
        pending = _Pending(BatonEvent(self.baton))
        self._pending[request_id] = pending
        return pending

    def await_answer(self, pending: _Pending) -> Any:
        pending.done.wait()
        return pending.answer

    def cancel(self, target: int, request_id: RequestId) -> None:
        self._outbox.append(OperatorCancel(target, request_id))

    def next_id(self) -> RequestId:
        return self._ids.next()


class LockstepApi:
    """``JsonApiPort`` over operator requests to one backend participant."""

    websocket_url = "lockstep:events"

    def __init__(self, operator: OperatorBaton, target: int, *,
                 error: type[Exception] = ApiError) -> None:
        self._operator = operator
        self._target = target
        self._error = error

    def close(self) -> None:
        """Nothing to close: requests ride the participant wire."""

    def request(self, method: str, path: str, payload: dict | None = None, *,
                expected: Iterable[int] = (200,), timeout: float = 30.0) -> tuple[int, Any]:
        body: dict[str, Any] = {"kind": "http", "method": method, "target": path,
                                "deadline_s": float(timeout)}
        if payload is not None:
            body["json"] = payload
        answer = self._operator.call(self._target, body)
        if "status" not in answer:
            raise self._error(f"{method} {path} failed: {answer}")
        if answer["status"] not in set(expected):
            raise self._error(f"{method} {path} returned {answer['status']}: {answer['body'][:1000]}")
        return answer["status"], json.loads(answer["body"])

    def get_json(self, path: str, *, timeout: float = 30.0) -> Any:
        return self.request("GET", path, timeout=timeout)[1]

    def post_json(self, path: str, payload: dict | None = None, *,
                  expected: Iterable[int] = (200,), timeout: float = 30.0) -> tuple[int, Any]:
        return self.request("POST", path, payload, expected=expected, timeout=timeout)

    def put_json(self, path: str, payload: dict, *, timeout: float = 30.0) -> Any:
        return self.request("PUT", path, payload, timeout=timeout)[1]


class LockstepEventStream:
    """``EventStreamPort`` over the backend's event sink (subscribed deliveries)."""

    def __init__(self, operator: OperatorBaton, target: int) -> None:
        self._operator = operator
        self._target = target
        self.queue = BatonQueue(operator.baton)
        self.subscription: RequestId | None = None
        self._pending: _Pending | None = None

    def __enter__(self) -> "LockstepEventStream":
        self.start()
        self.wait_subscribed()
        return self

    def start(self) -> None:
        """Queue the subscribe request (``__enter__`` without the wait)."""
        # Registered under its id before the request leaves: deliveries in the
        # acknowledgement's own tick (the initial snapshot) are not lost.
        self.subscription = self._operator.next_id()
        self._operator.streams[self.subscription] = self
        self._pending = self._operator.expect(self.subscription)
        self._operator.send(self._target, {"kind": "subscribe"}, self.subscription)

    def wait_subscribed(self) -> None:
        answer = self._operator.await_answer(self._pending)
        if answer.get("subscribed") != list(self.subscription):
            raise ApiError(f"subscribe failed: {answer}")

    def __exit__(self, *_exc: object) -> None:
        self._operator.streams.pop(self.subscription, None)
        dropped = 0
        while not self.queue.empty():
            self.queue.get_nowait()
            dropped += 1
        self._operator.journal.drop(self.subscription, dropped)
        request_id = self._operator.send(self._target, {"kind": "unsubscribe",
                                                        "subscription": list(self.subscription)})
        self._operator.journal.closing[request_id] = self.subscription

    def receive(self, timeout_s: float) -> str:
        try:
            data = self.queue.get(timeout=timeout_s)
        except rt.Empty as error:
            raise TimeoutError(f"no event within {timeout_s} s") from error
        self._operator.journal.receipt(self._operator.tick, self.subscription)
        return data


class BatonClock:
    """``ClockPort`` on the operator's baton."""

    def __init__(self, operator: OperatorBaton, epoch_us: int = 0) -> None:
        self._operator = operator
        self._epoch_us = epoch_us

    def monotonic(self) -> float:
        return self._operator.baton.monotonic()

    def unix(self) -> float:
        return (self._epoch_us + self._operator.baton.now_us) / 1_000_000

    def sleep(self, seconds: float) -> None:
        sleep(self._operator.baton, seconds)


__all__ = [
    "ApiError", "BatonClock", "DELIVERY_EVENT", "LockstepApi", "LockstepEventStream",
    "OperatorBaton", "OperatorJournal",
]
