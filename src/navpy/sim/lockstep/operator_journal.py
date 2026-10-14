"""What the operator saw, for the run journal (``OperatorBaton.observations``).

Each tick's part (``tick``) goes into that tick's journal record, and so
into the digest:

- ``receipts``: the subscription of each delivery a receive returned;
- ``received``: per receive, the tick the delivery arrived and its event
  ``type`` (the confirmation timing reads e2 and e3 from it);
- ``sent``: each request the script queued (id, kind or method, target);
- ``answered``: each HTTP response the script got (id, status);
- ``dropped``: deliveries an unsubscribed stream never received.

With ``evidence`` (the fleet's approval replay, ``lockstep_fleet.md``,
"Operator approvals" and "Operator polling schedule"), more is journaled;
the slice leaves it off, since its approval gate (one approval, its
confirmation timing) reads none of it, and step 10g's replay gate turns it on:

- ``sent`` and ``received`` carry the confirmation identity
  ``(sys_id, local_task_id, round_uid)`` and the request id: the POST's own
  id for ``sent``, the delivering subscription's for ``received``; global
  assignment ids are never confused with a peer's local task id;
- ``delivered``: each subscribed delivery as it is queued, separately from
  its consumption (``received``): ``[*subscription, type]``;
- ``receive``: each receive's ``[start tick, return tick, ended by]``, where
  ``ended by`` is ``event`` or ``timeout``;
- ``due_check``: each pass's due check, ``[tick, now_us, due]``, the due
  list as confirmation identities in pending order;
- ``sleep``: each sleep's ``[start tick, wake tick]``.

A POST's send tick is its ``sent`` record's and its answer tick its
``answered`` record's.
"""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any

from navpy.sim.lockstep.part_wire import RequestId

#: A confirmation identity's fields, as the approval POST and the
#: ``task_confirm_request`` event name them (``local_task_id`` is ``task_id``).
IDENTITY_FIELDS = ("sys_id", "task_id", "round_uid")


def _json_object(data: object) -> dict | None:
    try:
        value = json.loads(data) if isinstance(data, (str, bytes)) else data
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


def event_type(data: object) -> str | None:
    """The ``type`` of a delivered event (a JSON object as text), if any."""
    value = _json_object(data) if isinstance(data, (str, bytes)) else None
    kind = value.get("type") if value is not None else None
    return kind if isinstance(kind, str) else None


def confirmation_identity(body: object) -> list[Any]:
    """``[sys_id, local_task_id, round_uid]`` of an approval body or a request
    event; None for each field it lacks."""
    value = _json_object(body) or {}
    return [value.get(name) for name in IDENTITY_FIELDS]


@dataclass
class OperatorJournal:
    """What the operator saw: requests, answers, receipts with their ticks, drops."""

    evidence: bool = False
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
        received = [delivered_k, event_type(data)]
        if self.evidence:
            received += [*confirmation_identity(data), *subscription]
        self._add("received", received)

    def sent(self, request_id: RequestId, payload: object) -> None:
        body = payload if isinstance(payload, dict) else {}
        sent = [*request_id, body.get("method") or body.get("kind"), body.get("target")]
        if self.evidence:
            sent += confirmation_identity(body.get("json"))
        self._add("sent", sent)

    def answered(self, request_id: RequestId, answer: object) -> None:
        if isinstance(answer, dict) and "status" in answer:
            self._add("answered", [*request_id, answer["status"]])

    def delivered(self, subscription: RequestId, data: object) -> None:
        if self.evidence:
            self._add("delivered", [*subscription, event_type(data)])

    def receive(self, start_k: int, return_k: int, event: bool) -> None:
        if self.evidence:
            self._add("receive", [start_k, return_k, "event" if event else "timeout"])

    def due_check(self, k: int, now_s: float, due: Iterable[Iterable[object]]) -> None:
        """``due``: each due request's ``(sys_id, local_task_id, round_uid)``."""
        if self.evidence:
            self._add("due_check", [k, round(now_s * 1_000_000), [list(item) for item in due]])

    def sleep(self, start_k: int, wake_k: int) -> None:
        if self.evidence:
            self._add("sleep", [start_k, wake_k])

    def log_line(self, k: int, ordinal: int, line: str) -> None:
        self.log_lines.setdefault(ordinal, []).append(line)
        self.log_ticks.setdefault(ordinal, []).append(k)

    def drop(self, subscription: RequestId, count: int) -> None:
        if count:
            self.dropped += count
            drops = self.tick.setdefault("dropped", {})
            key = ".".join(map(str, subscription))
            drops[key] = drops.get(key, 0) + count


__all__ = ["IDENTITY_FIELDS", "OperatorJournal", "confirmation_identity", "event_type"]
