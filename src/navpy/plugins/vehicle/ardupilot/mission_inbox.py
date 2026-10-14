"""Generation-tagged mission-protocol inbox."""
from __future__ import annotations

import collections
from collections.abc import Callable
from dataclasses import dataclass

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.message_store import MessageHistoryOverrun


MISSION_PROTOCOL_TYPES = frozenset({
    "MISSION_REQUEST_INT",
    "MISSION_REQUEST",
    "MISSION_ACK",
})
_MISSION_PROTOCOL_TYPE_NAMES = tuple(sorted(MISSION_PROTOCOL_TYPES))


@dataclass(frozen=True)
class MissionEnvelope:
    message: MAVLink_message
    receipt_time_s: float
    generation: int


class MissionInbox:
    """Bounded mission-protocol replies; an evicted waiter's cursor fails closed.

    As ``MessageStore``: once a reply newer than a waiter's cursor has been
    evicted, ``wait_after`` raises ``MessageHistoryOverrun`` instead of
    timing out on an answer it can no longer see.
    """

    def __init__(self, max_messages: int = 256) -> None:
        self._condition = rt.Condition()
        self._messages: collections.deque[MissionEnvelope] = collections.deque(
            maxlen=max_messages,
        )
        self._evicted_through = 0
        self._generation = 0

    def publish(
        self,
        message: MAVLink_message,
        receipt_time_s: float,
    ) -> None:
        if message.get_type() not in MISSION_PROTOCOL_TYPES:
            return
        with self._condition:
            self._generation += 1
            if len(self._messages) == self._messages.maxlen:
                self._evicted_through = self._messages[0].generation
            self._messages.append(MissionEnvelope(
                message=message,
                receipt_time_s=receipt_time_s,
                generation=self._generation,
            ))
            self._condition.notify_all()

    def cursor(self) -> int:
        with self._condition:
            return self._generation

    def wait_after(
        self,
        cursor: int,
        predicate: Callable[[MAVLink_message], bool],
        *,
        deadline: float,
    ) -> MissionEnvelope | None:
        with self._condition:
            while True:
                for envelope in self._messages:
                    if envelope.generation > cursor and predicate(envelope.message):
                        return envelope
                if self._evicted_through > cursor:
                    raise MessageHistoryOverrun(_MISSION_PROTOCOL_TYPE_NAMES, cursor)
                remaining = deadline - rt.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
