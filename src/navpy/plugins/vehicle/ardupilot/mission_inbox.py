"""Generation-tagged mission-protocol inbox."""
from __future__ import annotations

import collections
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message


MISSION_PROTOCOL_TYPES = frozenset({
    "MISSION_REQUEST_INT",
    "MISSION_REQUEST",
    "MISSION_ACK",
})


@dataclass(frozen=True)
class MissionEnvelope:
    message: MAVLink_message
    receipt_time_s: float
    generation: int


class MissionInbox:
    def __init__(self, max_messages: int = 256) -> None:
        self._condition = threading.Condition()
        self._messages = collections.deque(maxlen=max_messages)
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
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
