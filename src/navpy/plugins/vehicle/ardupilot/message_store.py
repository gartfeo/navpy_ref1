"""Thread-safe immutable MAVLink message samples and fresh-response waits."""
from __future__ import annotations

from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Iterable

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt


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
    """Publishes atomic samples; keeps history only for declared types.

    ``history_policy`` maps a message type to how many samples it keeps for
    `wait_after`/`recent`. Every other type keeps only its latest sample, so
    high-rate telemetry does not grow a live heap the garbage collector must
    walk (docs/design/message_history.md). The policy is copied here and
    never changes afterwards.
    """

    def __init__(self, history_policy: Mapping[str, int]) -> None:
        policy = dict(history_policy)
        for message_type, depth in policy.items():
            if not isinstance(message_type, str) or not message_type:
                raise ValueError(
                    f"message history type must be a name: {message_type!r}"
                )
            if isinstance(depth, bool) or not isinstance(depth, int) or depth <= 0:
                raise ValueError(
                    f"message history depth for {message_type} must be a "
                    f"positive int: {depth!r}"
                )
        self._condition = rt.Condition()
        self._samples: dict[str, MessageSample] = {}
        self._history: dict[str, deque[MessageSample]] = {
            message_type: deque(maxlen=depth)
            for message_type, depth in policy.items()
        }
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
            history = self._history.get(message_type)
            if history is not None:
                if len(history) == history.maxlen:
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
        an unlocked iterator would race with it. Raises ValueError when
        ``message_type`` has no declared history.
        """
        with self._condition:
            self._require_history((message_type,))
            if count <= 0:
                return ()
            items = tuple(self._history[message_type])
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
            self._require_history(types)
            while True:
                candidates = sorted(
                    (
                        sample
                        for name in types
                        for sample in self._history[name]
                        if sample.generation > cursor
                    ),
                    key=lambda sample: sample.generation,
                )
                for sample in candidates:
                    if predicate(sample.message):
                        return sample
                if self._cursor_was_overrun(types, cursor):
                    raise MessageHistoryOverrun(types, cursor)
                remaining = deadline - rt.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)

    def _require_history(self, message_types: tuple[str, ...]) -> None:
        undeclared = [
            name for name in message_types if name not in self._history
        ]
        if undeclared:
            raise ValueError(
                "no message history declared for "
                f"{', '.join(map(str, undeclared))}; declare a depth in the "
                "vehicle's history policy"
            )

    def _cursor_was_overrun(
        self,
        message_types: tuple[str, ...],
        cursor: int,
    ) -> bool:
        return any(
            self._evicted_through.get(name, -1) > cursor
            for name in message_types
        )
