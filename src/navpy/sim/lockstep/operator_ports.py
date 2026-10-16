"""The operator's lockstep I/O ports (``lockstep_slice.md``, "Operator").

The script runs on :class:`~navpy.sim.lockstep.operator_baton.OperatorBaton`
and reaches the stack only through these:

- :class:`LockstepApi` (``JsonApiPort``): a request is queued and the script
  parks until its response arrives in a ``DONE``. Its timeout is the
  request's sim-time deadline, enforced by the backend.
- :class:`LockstepEventStream` (``EventStreamPort``): ``__enter__`` sends a
  subscribe and parks until its acknowledgement, as a WebSocket connect
  returns before the caller's next step; ``receive`` returns subscribed
  deliveries in order (each receive's tick and its delivery's tick are
  journaled); ``__exit__`` queues an unsubscribe and drops the deliveries
  not yet received, and those that arrive before the backend applies it;
  each drop is counted.
- :class:`BatonClock` (``ClockPort``): the baton's clock; ``unix`` is the
  run's epoch plus ``t_k``.

With the operator's ``evidence`` on, each receive's start and return ticks
and each sleep's start and wake ticks are journaled too, and
:func:`polling_start_hook` and :func:`due_check_hook` journal the
confirmation loop's start and due checks (``operator_journal``).

The ``TailPort`` is :class:`~navpy.sim.lockstep.operator_tail.LockstepLogTail`.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable
from typing import Any

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import sleep
from navpy.sim.lockstep.operator_baton import OperatorBaton, PendingAnswer
from navpy.sim.lockstep.part_wire import RequestId


class ApiError(RuntimeError):
    """A lockstep API call failed: an unexpected status, a deadline or a cancel."""


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
        #: ``(delivery tick, data)`` per delivery, in delivery order.
        self.queue = BatonQueue(operator.baton)
        self.subscription: RequestId | None = None
        self._pending: PendingAnswer | None = None

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
        journal, start = self._operator.journal, self._operator.tick
        try:
            delivered_k, data = self.queue.get(timeout=timeout_s)
        except rt.Empty as error:
            journal.receive(start, self._operator.tick, False, self.subscription)
            raise TimeoutError(f"no event within {timeout_s} s") from error
        journal.receive(start, self._operator.tick, True, self.subscription)
        journal.receipt(self._operator.tick, self.subscription, delivered_k, data)
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
        start = self._operator.tick
        sleep(self._operator.baton, seconds)
        self._operator.journal.sleep(start, self._operator.tick)


def polling_start_hook(operator: OperatorBaton) -> Callable[[float], None]:
    """The confirmation loop's ``on_polling_start``: journals the loop's
    start tick and its deadline on the operator's journal."""

    def record(deadline_s: float) -> None:
        operator.journal.polling(operator.tick, deadline_s)

    return record


def due_check_hook(operator: OperatorBaton) -> Callable[[float, Iterable[Any]], None]:
    """The confirmation loop's ``on_due_check``: journals each pass's due check
    (its tick, ``now`` and the due requests' identities) on the operator's journal."""

    def record(now_s: float, due: Iterable[Any]) -> None:
        operator.journal.due_check(operator.tick, now_s, [
            (request.sys_id, request.local_task_id, request.round_uid) for request in due])

    return record


__all__ = ["ApiError", "BatonClock", "LockstepApi", "LockstepEventStream", "due_check_hook",
           "polling_start_hook"]
