"""The backend's per-tick WebSocket event sink (``lockstep_slice.md``, "WS").

It keeps two things apart:

- **The event journal:** every broadcast, subscribed or not, becomes a
  ``broadcast`` event in the tick's ``DONE`` (``ConnectionManager.observer``).
- **Subscriber delivery** follows WebSocket semantics. A subscribe request's
  handler installs a :class:`Subscriber` as a connection-manager client and
  sends it the initial snapshot first (``routes/telemetry.py``), then answers
  the subscribe. A broadcast reaches a subscriber if its membership exists
  when the broadcast executes (``broadcast.py``), so delivery starts exactly
  after the installation; nothing earlier is replayed. Each delivery is a
  ``ws`` event naming the subscription.

After the shutdown's work is retired the sink closes: a later broadcast is a
fault.
"""

from __future__ import annotations

from typing import Any

import orjson

from navpy.sim.lockstep.part_wire import Event, RequestId
from navpy.sim.lockstep.participant_hooks import OutboxBuffer

#: ``DONE`` event kinds: a journaled broadcast, and a delivery to a subscriber.
BROADCAST_EVENT = "broadcast"
DELIVERY_EVENT = "ws"
#: The fault kind a broadcast after the sink closed reports.
CLOSED_FAULT = "EventSinkClosed:broadcast"


class EventSinkClosed(RuntimeError):
    """A broadcast after the shutdown closed the event sink."""


class Subscriber:
    """A connection-manager client standing in for one operator WebSocket."""

    def __init__(self, sink: "EventSink", subscription: RequestId) -> None:
        self._sink = sink
        self.subscription = subscription

    async def accept(self) -> None:
        """The WS handshake: nothing to do in process."""

    async def send_bytes(self, data: bytes) -> None:
        self._sink.deliver(self.subscription, data.decode("utf-8"))

    async def send_text(self, data: str) -> None:
        self._sink.deliver(self.subscription, data)


class EventSink:
    def __init__(self, out: OutboxBuffer) -> None:
        self._out = out
        self._open = True
        self._sequence = 0
        self.subscribers: dict[RequestId, Subscriber] = {}

    def observe(self, data: Any) -> None:
        """``ConnectionManager.observer``: journal one broadcast."""
        if not self._open:
            self._out.faults.append(CLOSED_FAULT)
            raise EventSinkClosed("a broadcast after the event sink closed")
        self._sequence += 1
        self._out.events.append(Event(BROADCAST_EVENT, {
            "seq": self._sequence, "data": orjson.dumps(data).decode("utf-8"),
        }))

    def subscriber(self, subscription: RequestId) -> Subscriber:
        subscriber = Subscriber(self, subscription)
        self.subscribers[subscription] = subscriber
        return subscriber

    def deliver(self, subscription: RequestId, data: str) -> None:
        self._out.events.append(Event(DELIVERY_EVENT, {
            "subscription": list(subscription), "data": data,
        }))

    def close(self) -> None:
        self._open = False


__all__ = [
    "BROADCAST_EVENT", "CLOSED_FAULT", "DELIVERY_EVENT", "EventSink", "EventSinkClosed", "Subscriber",
]
