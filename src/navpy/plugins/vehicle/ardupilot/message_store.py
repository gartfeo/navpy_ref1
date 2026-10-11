"""Thread-safe immutable MAVLink message samples and fresh-response waits."""
from __future__ import annotations

import threading
import time
from collections import defaultdict, deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Iterable

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message


MESSAGE_HISTORY_LIMIT = 1024


class MessageHistoryOverrun(RuntimeError):
    """A wait cursor fell behind the bounded per-type response history."""

    def __init__(self, message_types: tuple[str, ...], cursor: int) -> None:
        self.message_types = message_types
        self.cursor = int(cursor)
        super().__init__(
            "MAVLink response history overrun after cursor "
            f"{self.cursor} for {', '.join(message_types)}"
        )


@dataclass(frozen=True)
class MessageSample:
    message: MAVLink_message
    receipt_time_s: float
    boot_time_ms: int | None
    generation: int


class MessageStore:
    """Publishes atomic samples and retains recent generations per type."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._samples: dict[str, MessageSample] = {}
        self._history: dict[str, deque[MessageSample]] = defaultdict(
            lambda: deque(maxlen=MESSAGE_HISTORY_LIMIT)
        )
        self._evicted_through: dict[str, int] = {}
        self._generation = 0

    def publish(
        self,
        message_type: str,
        message: MAVLink_message,
        *,
        receipt_time_s: float,
        boot_time_ms: int | None = None,
    ) -> MessageSample:
        with self._condition:
            self._generation += 1
            sample = MessageSample(
                message=message,
                receipt_time_s=float(receipt_time_s),
                boot_time_ms=boot_time_ms,
                generation=self._generation,
            )
            self._samples[message_type] = sample
            history = self._history[message_type]
            if len(history) == MESSAGE_HISTORY_LIMIT:
                self._evicted_through[message_type] = history[0].generation
            history.append(sample)
            self._condition.notify_all()
            return sample

    def latest(self, message_type: str) -> MessageSample | None:
        with self._condition:
            return self._samples.get(message_type)

    def recent(
        self, message_type: str, count: int
    ) -> tuple[MessageSample, ...]:
        """Newest-last immutable view of retained samples, copied under lock.

        Returning a tuple built while holding the condition is what makes the
        view safe against a concurrent `publish()`; handing out the deque or
        an unlocked iterator would race with it.
        """
        if count <= 0:
            return ()
        with self._condition:
            history = self._history.get(message_type)
            if not history:
                return ()
            items = tuple(history)
        return items[-count:] if count < len(items) else items

    def message(self, message_type: str) -> MAVLink_message | None:
        sample = self.latest(message_type)
        return sample.message if sample is not None else None

    def cursor(self, message_type: str | None = None) -> int:
        with self._condition:
            if message_type is None:
                return self._generation
            sample = self._samples.get(message_type)
            return sample.generation if sample is not None else self._generation

    def wait_after(
        self,
        message_types: str | Iterable[str],
        cursor: int,
        predicate: Callable[[MAVLink_message], bool],
        *,
        deadline: float,
    ) -> MessageSample | None:
        types = (
            (message_types,)
            if isinstance(message_types, str)
            else tuple(message_types)
        )
        with self._condition:
            while True:
                candidates = sorted(
                    (
                        sample
                        for name in types
                        for sample in self._history.get(name, ())
                        if sample.generation > cursor
                    ),
                    key=lambda sample: sample.generation,
                )
                for sample in candidates:
                    if predicate(sample.message):
                        return sample
                if self._cursor_was_overrun(types, cursor):
                    raise MessageHistoryOverrun(types, cursor)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

    def _cursor_was_overrun(
        self,
        message_types: tuple[str, ...],
        cursor: int,
    ) -> bool:
        return any(
            self._evicted_through.get(name, -1) > cursor
            for name in message_types
        )
