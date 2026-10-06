"""Synchronized MAVLink callback registration and ordered dispatch."""
from __future__ import annotations

import collections
import threading
from collections.abc import Callable

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.vehicle.logger_ref import LoggerRef

MessageCallback = Callable[[MAVLink_message], None]


class Subscription:
    def __init__(self, cancel: Callable[[], None]) -> None:
        self._cancel_action = cancel
        self._lock = threading.Lock()
        self._cancelled = False

    def cancel(self) -> None:
        with self._lock:
            if self._cancelled:
                return
            self._cancelled = True
        self._cancel_action()

    def __call__(self) -> None:
        self.cancel()


class CallbackRegistry:
    """Snapshots under lock and always invokes callbacks after releasing it."""

    def __init__(self, logger_ref: LoggerRef | None = None) -> None:
        self._lock = threading.Lock()
        self._callbacks: dict[str, list[MessageCallback]] = (
            collections.defaultdict(list)
        )
        self._logger_ref = logger_ref

    def subscribe(
        self,
        message_name: str,
        callback: MessageCallback,
    ) -> Subscription:
        with self._lock:
            self._callbacks[message_name].append(callback)
        return Subscription(lambda: self.unsubscribe(message_name, callback))

    def unsubscribe(
        self,
        message_name: str,
        callback: MessageCallback,
    ) -> None:
        with self._lock:
            callbacks = self._callbacks.get(message_name)
            if not callbacks:
                return
            try:
                callbacks.remove(callback)
            except ValueError:
                return
            if not callbacks:
                self._callbacks.pop(message_name, None)

    def snapshot(
        self,
        message_type: str,
        *,
        is_navlink: bool,
    ) -> tuple[MessageCallback, ...]:
        with self._lock:
            ordered = list(self._callbacks.get(message_type, ()))
            ordered.extend(self._callbacks.get("*", ()))
            if is_navlink:
                ordered.extend(self._callbacks.get("NAVLINK", ()))
            return tuple(ordered)

    def dispatch(
        self,
        message_type: str,
        message: MAVLink_message,
        *,
        is_navlink: bool,
    ) -> None:
        for callback in self.snapshot(message_type, is_navlink=is_navlink):
            try:
                callback(message)
            except Exception as exc:  # callbacks must not kill the bus reader
                if self._logger_ref is not None:
                    self._logger_ref.value.error(
                        f"callback {callback} raised {exc}"
                    )
