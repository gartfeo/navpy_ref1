"""Operator admission: the scripted operator and the app relay in one order.

``deterministic_dev_sim.md`` section 5, "How an action is fixed in time".
In tick k's operator phase the coordinator takes, in one step, the
scripted operator's requests and cancels, then everything the relay has
received so far, in arrival order (:meth:`Admission.admit`). Each relay
request gets its id from the counter the scripted operator and the
coordinator's spawns share, so ids follow admission order. A cancel keeps
the id of the request it cancels:

- for a request taken in the same step, the coordinator resolves it: the
  request is never delivered, its handler never starts, and the coordinator
  answers ``{"cancelled": true}``;
- for one taken earlier, the cancel is delivered (``TICK k+1``); if the
  response already came, it does nothing (a response stands).

A request the run does not support is refused here, ``refuse`` naming its
answer (a ``501``): it never enters the run, and the refusal is journaled
like any answer. Relay items wait in the relay until its target is ready,
so none reaches a participant that cannot take it.

A tick that took relay items journals the whole admission (``admission``:
every item in order, each payload as its hash, any answer the coordinator
gave), so the digest covers suppressed and refused requests too. A tick
without relay items journals nothing new: a scripted run's digest is
unchanged. :attr:`Admission.actions` is the action file: every admitted
request and cancel, scripted and relay alike, with its tick, id and payload.

:meth:`Admission.route` sends the relay its responses (matched by id, in
any completion order) and the backend's journaled broadcasts; it returns
the responses that are the scripted operator's.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from navpy.sim.lockstep.coordinator_journal import sha256
from navpy.sim.lockstep.coordinator_types import OperatorCancel, OperatorRequest
from navpy.sim.lockstep.part_wire import Event, RequestId, Response
from navpy.sim.lockstep.part_wire_codec import dumps
from navpy.sim.lockstep.participant_hooks import RequestIds

#: The backend's ``DONE`` event kind of one journaled broadcast (``event_sink``).
BROADCAST_EVENT = "broadcast"
#: The coordinator's answer to a request cancelled in the step that took it.
CANCELLED = {"cancelled": True}
#: The action file's sources.
SCRIPT, RELAY = "script", "relay"

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RelayRequest:
    #: The relay's own handle (host side: it never enters the journal).
    ticket: int
    payload: dict[str, Any]


@dataclass(frozen=True)
class RelayCancel:
    ticket: int


RelayItem = RelayRequest | RelayCancel


class RelayPort(Protocol):
    """The app relay as the coordinator sees it. ``take`` and the
    deliveries run on the coordinator's thread; they must not block."""

    #: The participant every relay request goes to (the backend).
    target: int

    def take(self) -> Sequence[RelayItem]:
        """Everything received since the last take, in arrival order."""

    def answer(self, ticket: int, answer: object) -> None: ...

    def broadcast(self, data: str) -> None: ...


class RelayInbox:
    """The relay's thread-safe queue of items for the next admission."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: list[RelayItem] = []

    def submit(self, item: RelayItem) -> None:
        with self._lock:
            self._items.append(item)

    def take(self) -> tuple[RelayItem, ...]:
        with self._lock:
            items, self._items = tuple(self._items), []
        return items


def payload_hash(payload: object) -> str:
    return sha256(dumps(payload))


class Admission:
    def __init__(self, ids: RequestIds, relay: RelayPort | None = None,
                 refuse: Callable[[dict[str, Any]], object | None] = lambda payload: None) -> None:
        self._ids = ids
        self._relay = relay
        self._refuse = refuse
        #: Relay ticket -> its request id, for every relay request admitted.
        self._ids_by_ticket: dict[int, RequestId] = {}
        #: Delivered relay requests still waiting for their response.
        self._waiting: dict[RequestId, int] = {}
        self.actions: list[dict[str, Any]] = []

    def route(self, responses: Iterable[tuple[int, Response]],
              events: Iterable[tuple[int, Event]]) -> tuple[tuple[int, Response], ...]:
        """Relay responses and broadcasts to the relay; return the operator's responses."""
        kept = []
        for ordinal, response in responses:
            ticket = self._waiting.pop(response.id, None)
            if ticket is None:
                kept.append((ordinal, response))
            else:
                self._relay.answer(ticket, response.payload)
        if self._relay is not None:
            for ordinal, event in events:
                if ordinal == self._relay.target and event.kind == BROADCAST_EVENT:
                    self._relay.broadcast(event.payload["data"])
        return tuple(kept)

    def admit(self, k: int, scripted: Sequence[OperatorRequest | OperatorCancel],
              ready: Callable[[int], bool]) -> tuple[list[OperatorRequest | OperatorCancel],
                                                     dict[str, Any] | None]:
        """Tick k's one admission step: the deliveries for ``TICK k+1`` and
        the journal note (None when no relay item was taken)."""
        for item in scripted:
            self._action(k, SCRIPT, item)
        relay = self._relay
        items = relay.take() if relay is not None and ready(relay.target) else ()
        if not items:
            return list(scripted), None
        deliveries: list[OperatorRequest | OperatorCancel | None] = list(scripted)
        taken: dict[int, int] = {}  # ticket -> its delivery index, this step
        entries = []
        for item in items:
            if isinstance(item, RelayRequest):
                entries.append(self._request(k, item, deliveries, taken))
            else:
                entry = self._cancel(k, item, deliveries, taken)
                if entry is not None:
                    entries.append(entry)
        note = {"script": [_scripted_entry(item) for item in scripted], "relay": entries}
        return [item for item in deliveries if item is not None], note

    def _request(self, k: int, item: RelayRequest, deliveries: list, taken: dict[int, int]) -> dict:
        request_id = self._ids.next()
        self._ids_by_ticket[item.ticket] = request_id
        request = OperatorRequest(self._relay.target, request_id, item.payload)
        self._action(k, RELAY, request)
        entry = {"kind": "request", "id": list(request_id), "target": request.target,
                 "payload": payload_hash(item.payload)}
        refusal = self._refuse(item.payload)
        if refusal is not None:
            entry["answer"] = refusal
            self._relay.answer(item.ticket, refusal)
            return entry
        taken[item.ticket] = len(deliveries)
        deliveries.append(request)
        self._waiting[request_id] = item.ticket
        return entry

    def _cancel(self, k: int, item: RelayCancel, deliveries: list, taken: dict[int, int]) -> dict | None:
        request_id = self._ids_by_ticket.get(item.ticket)
        if request_id is None:  # the relay only cancels what it sent: nothing to journal
            _log.warning("relay cancel for unknown ticket %s ignored", item.ticket)
            return None
        cancel = OperatorCancel(self._relay.target, request_id)
        self._action(k, RELAY, cancel)
        entry = {"kind": "cancel", "id": list(request_id), "target": cancel.target}
        index = taken.pop(item.ticket, None)
        if index is not None:  # taken in this step: never delivered
            deliveries[index] = None
            del self._waiting[request_id]
            entry["answer"] = dict(CANCELLED)
            self._relay.answer(item.ticket, dict(CANCELLED))
        elif request_id in self._waiting:
            deliveries.append(cancel)
        else:  # answered already, or refused: a response stands
            entry["answer"] = None
        return entry

    def _action(self, k: int, source: str, item: OperatorRequest | OperatorCancel) -> None:
        action = {"k": k, "source": source, "kind": "request" if isinstance(item, OperatorRequest)
                  else "cancel", "target": item.target, "id": list(item.id)}
        if isinstance(item, OperatorRequest):
            action["payload"] = item.payload
        self.actions.append(action)


def _scripted_entry(item: OperatorRequest | OperatorCancel) -> dict[str, Any]:
    if isinstance(item, OperatorRequest):
        return {"kind": "request", "id": list(item.id), "target": item.target,
                "payload": payload_hash(item.payload)}
    return {"kind": "cancel", "id": list(item.id), "target": item.target}


__all__ = ["Admission", "BROADCAST_EVENT", "CANCELLED", "RELAY", "RelayCancel", "RelayInbox",
           "RelayItem", "RelayPort", "RelayRequest", "SCRIPT", "payload_hash"]
