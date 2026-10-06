"""Text and status sinks used by the asynchronous cache logger."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Protocol

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_log_message import LogMessage
from navpy.logger.status_logger import IStatusLogger


class TextLogSink(Protocol):
    def debug(self, msg: object) -> None: ...

    def info(self, msg: object) -> None: ...

    def warning(self, msg: object) -> None: ...

    def error(self, msg: object) -> None: ...


class StatusPolicyPort(Protocol):
    status_level: CacheLogLevel
    status_update_interval: float


class CacheStatusOutput:
    """Apply status severity, cadence, and duplicate policies."""

    def __init__(
        self,
        policy: StatusPolicyPort,
        sink: IStatusLogger | None,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._policy = policy
        self._sink = sink
        self._clock = clock
        self._last_by_key: dict[str, object] = {}
        self._last_update_s = float(clock())
        self._lock = threading.RLock()

    @property
    def sink(self) -> IStatusLogger | None:
        with self._lock:
            return self._sink

    def emit(
        self,
        msg: object,
        level: CacheLogLevel,
        *,
        key: str = "",
        dest: LogStatusDest | None = None,
        check_interval: bool = False,
    ) -> None:
        with self._lock:
            sink = self._sink
            if level > self._policy.status_level or sink is None:
                return
            now_s = float(self._clock())
            if (
                check_interval
                and now_s - self._last_update_s
                < self._policy.status_update_interval
            ):
                return
            if self._last_by_key.get(key) == msg:
                return
            self._last_by_key[key] = msg
            sink.send_log(msg, dest)
            self._last_update_s = now_s

    def defer(self, enable: bool, *, flush: bool) -> None:
        with self._lock:
            if self._sink is not None:
                self._sink.defer_status_texts(enable, flush=flush)

    def close(self) -> None:
        with self._lock:
            sink = self._sink
            self._sink = None
        if sink is not None:
            sink.close()


class CacheLogOutput:
    """Dispatch one queued message to text first, then optional status."""

    def __init__(
        self,
        text: TextLogSink,
        status: CacheStatusOutput,
    ) -> None:
        self._text = text
        self._status = status

    def handle(self, message: LogMessage) -> None:
        dispatch = {
            CacheLogLevel.VERBOSE: self._text.debug,
            CacheLogLevel.DEBUG: self._text.debug,
            CacheLogLevel.INFO: self._text.info,
            CacheLogLevel.WARNING: self._text.warning,
            CacheLogLevel.ERROR: self._text.error,
        }
        dispatch[message.level](message.msg)
        if message.status is not None:
            self._status.emit(
                message.status,
                message.level,
                key=message.key,
                dest=message.dest,
                check_interval=message.check_interval,
            )


__all__ = [
    "CacheLogOutput",
    "CacheStatusOutput",
    "StatusPolicyPort",
    "TextLogSink",
]
