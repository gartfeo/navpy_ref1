"""The operator on its own baton (tick step 5).

:class:`OperatorBaton` is an ``OperatorHost``: it runs an operator script on
a baton in the coordinator's process. Each tick it delivers what the tick
brought (lifecycle records, responses, subscribed events, log lines), runs
the baton to quiescence at ``t_k``, and returns the requests the script
made; they travel in ``TICK k+1``. The script reaches it through the ports
in :mod:`navpy.sim.lockstep.operator_ports`.

:class:`OperatorJournal` keeps what the operator saw. Each tick's part of it
(:meth:`OperatorBaton.observations`) goes into the run journal, and so into
its digest:

- ``receipts``: the subscription of each delivery a receive returned;
- ``received``: per receive, the tick the delivery arrived and its event
  ``type`` (the confirmation timing reads e2 and e3 from it);
- ``sent``: each request the script queued (id, kind or method, target);
- ``answered``: each HTTP response the script got (id, status);
- ``dropped``: deliveries an unsubscribed stream never received.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from navpy.sim.lockstep.baton import BatonRuntime, ManagedThread
from navpy.sim.lockstep.baton_sync import BatonEvent
from navpy.sim.lockstep.coordinator_types import OperatorCancel, OperatorInbox, OperatorRequest
from navpy.sim.lockstep.part_wire import RequestId
from navpy.sim.lockstep.participant_hooks import RequestIds

if TYPE_CHECKING:
    from navpy.sim.lockstep.operator_ports import LockstepEventStream

#: The backend's ``DONE`` event kind of one subscriber delivery.
DELIVERY_EVENT = "ws"


@dataclass
class PendingAnswer:
    done: BatonEvent
    answer: Any = None


def event_type(data: object) -> str | None:
    """The ``type`` of a delivered event (a JSON object as text), if any."""
    try:
        value = json.loads(data) if isinstance(data, (str, bytes)) else None
    except ValueError:
        return None
    kind = value.get("type") if isinstance(value, dict) else None
    return kind if isinstance(kind, str) else None


@dataclass
class OperatorJournal:
    """What the operator saw: requests, answers, receipts with their ticks, drops."""

    receipts: list[tuple[int, list[int]]] = field(default_factory=list)
    dropped: int = 0
    unclaimed: list[tuple[int, list[int]]] = field(default_factory=list)
    #: This tick's observations, until the coordinator takes them.
    tick: dict[str, Any] = field(default_factory=dict)
    #: Unsubscribe request id -> its subscription, until the backend answers:
    #: a delivery for it meanwhile is a late drop.
    closing: dict[RequestId, RequestId] = field(default_factory=dict)
    #: Each participant's ``DONE.log_lines`` so far, by ordinal, and the tick of each.
    log_lines: dict[int, list[str]] = field(default_factory=dict)
    log_ticks: dict[int, list[int]] = field(default_factory=dict)

    def _add(self, key: str, value: object) -> None:
        self.tick.setdefault(key, []).append(value)

    def receipt(self, k: int, subscription: RequestId, delivered_k: int, data: object) -> None:
        self.receipts.append((k, list(subscription)))
        self._add("receipts", list(subscription))
        self._add("received", [delivered_k, event_type(data)])

    def sent(self, request_id: RequestId, payload: object) -> None:
        body = payload if isinstance(payload, dict) else {}
        self._add("sent", [*request_id, body.get("method") or body.get("kind"), body.get("target")])

    def answered(self, request_id: RequestId, answer: object) -> None:
        if isinstance(answer, dict) and "status" in answer:
            self._add("answered", [*request_id, answer["status"]])

    def log_line(self, k: int, ordinal: int, line: str) -> None:
        self.log_lines.setdefault(ordinal, []).append(line)
        self.log_ticks.setdefault(ordinal, []).append(k)

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
        self._pending: dict[RequestId, PendingAnswer] = {}
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
            self.journal.log_line(inbox.k, ordinal, line)
        for entry in inbox.lifecycle:
            if entry.kind == "ready":
                self._ready_event(entry.ordinal).deliver_set()
        for _ordinal, response in inbox.responses:
            pending = self._pending.pop(response.id, None)
            if pending is None:  # an answer nobody waits for (an unsubscribe)
                self.journal.unclaimed.append((inbox.k, list(response.id)))
                continue
            self.journal.answered(response.id, response.payload)
            pending.answer = response.payload
            pending.done.deliver_set()
        for _ordinal, event in inbox.events:
            if event.kind == DELIVERY_EVENT:
                subscription = tuple(event.payload["subscription"])
                stream = self.streams.get(subscription)
                if stream is not None:
                    stream.queue.deliver((inbox.k, event.payload["data"]))
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
        self.journal.sent(request_id, payload)
        self._outbox.append(OperatorRequest(target, request_id, payload))
        return request_id

    def call(self, target: int, payload: object, request_id: RequestId | None = None) -> Any:
        """Queue a request and park until its response."""
        request_id = request_id or self._ids.next()
        pending = self.expect(request_id)
        self.send(target, payload, request_id)
        return self.await_answer(pending)

    def expect(self, request_id: RequestId) -> PendingAnswer:
        """Register for ``request_id``'s answer (before the request leaves)."""
        pending = PendingAnswer(BatonEvent(self.baton))
        self._pending[request_id] = pending
        return pending

    def await_answer(self, pending: PendingAnswer) -> Any:
        pending.done.wait()
        return pending.answer

    def cancel(self, target: int, request_id: RequestId) -> None:
        self._outbox.append(OperatorCancel(target, request_id))

    def next_id(self) -> RequestId:
        return self._ids.next()


__all__ = ["DELIVERY_EVENT", "OperatorBaton", "OperatorJournal", "PendingAnswer", "event_type"]
