"""Thread-safe ownership and construction of navigation log streams."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import os
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import NoReturn, Optional, Protocol, TextIO

from navpy.logger.cache_logger import ILogger
from navpy.logger.log_events import LogEvent, STANDARD_EVENT_HEADER
from navpy.logger.log_schema import PRIMARY_SCHEMA
from navpy.logger.navigation_stream_worker import NavigationStreamWorker


_COMPACT = "compact"
_DEBUG = "debug"


@dataclass(frozen=True)
class _StreamWrite:
    target: str
    line: str | None
    flush: bool


class NavigationTextStream(Protocol):
    """Text-stream operations required by navigation logging."""

    def write(self, text: str) -> object: ...

    def flush(self) -> object: ...

    def close(self) -> object: ...


class NavigationLogStreams:
    """Own the compact/debug handles, write lock, and flush counters."""

    def __init__(
        self,
        compact: Optional[NavigationTextStream],
        debug: Optional[NavigationTextStream],
    ) -> None:
        self._compact = compact
        self._compact_output = compact
        self._debug = debug
        self._debug_output = debug
        self._worker = NavigationStreamWorker(self._apply_write)
        self._condition = threading.Condition(threading.RLock())
        self._debug_event_count = 0
        self._phase = "active"
        self._close_owner: int | None = None

    def has_compact(self) -> bool:
        with self._condition:
            return self._phase == "active" and self._compact is not None

    def write_primary(self, line: str) -> None:
        with self._condition:
            if self._phase != "active" or self._compact is None:
                return
            self._worker.submit(_StreamWrite(_COMPACT, line, True))

    def write_event(
        self,
        event: LogEvent,
        line: str,
        *,
        flush_interval: int,
    ) -> None:
        with self._condition:
            if self._phase != "active":
                return
            if event == LogEvent.SNAP:
                self._write_snap(line)
                return
            if self._debug is None:
                return
            self._debug_event_count += 1
            flush = (
                event == LogEvent.SNAP_COMPONENTS
                or self._debug_event_count % flush_interval == 0
            )
            self._worker.submit(_StreamWrite(_DEBUG, line, flush))
            if event == LogEvent.SNAP_COMPONENTS:
                self.drain()

    def drain(self) -> None:
        """Fence every navigation row submitted before this call."""
        self._worker.wait_until_idle()
        error = self._worker.first_error
        if error is not None:
            raise error

    def close(self) -> None:
        owner = threading.get_ident()
        with self._condition:
            while self._phase == "closing":
                if self._close_owner == owner:
                    return
                self._condition.wait()
            if self._phase == "closed":
                return
            self._phase = "closing"
            self._close_owner = owner
            compact, self._compact = self._compact, None
            debug, self._debug = self._debug, None

        worker_errors: list[BaseException] = []
        try:
            self._worker.close()
        except BaseException as error:
            with self._condition:
                self._compact = compact
                self._debug = debug
                self._phase = "close_failed"
                self._close_owner = None
                self._condition.notify_all()
            raise error
        worker_error = self._worker.first_error
        if worker_error is not None:
            worker_errors.append(worker_error)

        compact_errors, compact_failed = _close_stream(compact)
        debug_errors, debug_failed = _close_stream(debug)
        errors = [*worker_errors, *compact_errors, *debug_errors]

        with self._condition:
            self._compact = compact if compact_failed else None
            self._debug = debug if debug_failed else None
            self._phase = (
                "close_failed"
                if compact_failed or debug_failed
                else "closed"
            )
            self._close_owner = None
            self._condition.notify_all()
        if errors:
            _raise_stream_close_errors(errors)

    def _write_snap(self, line: str) -> None:
        if self._compact is None:
            return
        if self._debug is not None:
            self._worker.submit(_StreamWrite(_DEBUG, None, True))
        self._worker.submit(_StreamWrite(_COMPACT, line, True))
        self.drain()

    def _apply_write(self, operation: _StreamWrite) -> None:
        stream = (
            self._compact_output
            if operation.target == _COMPACT
            else self._debug_output
        )
        if stream is None:
            return
        if operation.line is not None:
            stream.write(operation.line)
        if operation.flush:
            stream.flush()


def open_navigation_log_streams(
    sys_id: int,
    logger: ILogger,
) -> NavigationLogStreams:
    """Open and initialise both navigation CSV streams independently."""
    log_path = _create_log_directory(logger)
    if log_path is None:
        return NavigationLogStreams(None, None)
    compact = _open_stream(
        os.path.join(log_path, f"uav_{sys_id}_navigation_compact.csv"),
        PRIMARY_SCHEMA.header_line(),
        "compact CSV",
        logger,
    )
    debug = _open_stream(
        os.path.join(log_path, f"uav_{sys_id}_navigation_debug.csv"),
        STANDARD_EVENT_HEADER,
        "navigation debug CSV",
        logger,
    )
    return NavigationLogStreams(compact, debug)


def _create_log_directory(logger: ILogger) -> Optional[str]:
    try:
        log_path = getattr(logger, "log_path", None)
        if not log_path:
            today = datetime.today()
            log_path = f".logs/{today.date()}/{today.strftime('%H%M%S')}/"
        os.makedirs(log_path, exist_ok=True)
        return log_path
    except OSError as exc:
        logger.warning(f"Failed to create navigation log directory: {exc}")
        return None


def _open_stream(
    path: str,
    header: str,
    label: str,
    logger: ILogger,
) -> Optional[TextIO]:
    stream: Optional[TextIO] = None
    try:
        stream = open(path, "w", encoding="utf-8")
        stream.write(header + "\n")
        stream.flush()
        return stream
    except OSError as exc:
        logger.warning(f"Failed to create {label}: {exc}")
        cleanup_errors, _close_failed = _close_stream(stream)
        for cleanup_error in cleanup_errors:
            if not isinstance(cleanup_error, OSError):
                raise cleanup_error
            logger.warning(f"Failed to discard incomplete {label}: {cleanup_error}")
        return None


def _close_stream(
    stream: Optional[NavigationTextStream],
) -> tuple[list[BaseException], bool]:
    if stream is None:
        return [], False
    errors: list[BaseException] = []
    try:
        stream.flush()
    except BaseException as error:
        errors.append(error)
    close_failed = False
    try:
        stream.close()
    except BaseException as error:
        errors.append(error)
        close_failed = True
    return errors, close_failed


def _raise_stream_close_errors(errors: list[BaseException]) -> NoReturn:
    if len(errors) == 1:
        raise errors[0]
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup("navigation stream cleanup failed", errors)
    raise BaseExceptionGroup("navigation stream cleanup failed", errors)
