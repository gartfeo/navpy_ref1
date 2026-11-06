"""Thread-safe ownership and construction of navigation log streams."""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.exception_groups import BaseExceptionGroup, ExceptionGroup

import os
from dataclasses import dataclass
from typing import Callable, NoReturn, Optional, Protocol, TextIO

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.approach.logs.evidence_failure import report_evidence_failure
from navpy.core.approach.logs.log_events import LogEvent, STANDARD_EVENT_HEADER
from navpy.core.approach.logs.log_schema import PRIMARY_SCHEMA
from navpy.core.approach.logs.navigation_stream_worker import (
    NavigationStreamBacklogError,
    NavigationStreamWorker,
)


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
        *,
        on_error: Callable[[BaseException], None] | None = None,
        setup_error: BaseException | None = None,
    ) -> None:
        self._compact = compact
        self._compact_output = compact
        self._debug = debug
        self._debug_output = debug
        # A failed write is reported and skipped; writing goes on (todo 290).
        # Failures outside the worker (setup, a full queue, close) are kept
        # here, so raise_if_failed still sees them.
        self._on_error = on_error
        self._failure = setup_error
        self._worker = NavigationStreamWorker(self._apply_write, on_error=on_error)
        self._condition = rt.Condition(rt.RLock())
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
            self._submit(_StreamWrite(_COMPACT, line, True))

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
            self._submit(_StreamWrite(_DEBUG, line, flush))
            if event == LogEvent.SNAP_COMPONENTS:
                self.drain()

    def drain(self) -> None:
        """Fence every navigation row submitted before this call."""
        self._worker.wait_until_idle()

    def raise_if_failed(self) -> None:
        """Raise the first failed write, for callers that stop on it."""
        error = self._worker.first_error or self._failure
        if error is not None:
            raise RuntimeError("navigation log stream write failed") from error

    def close(self) -> None:
        owner = rt.get_ident()
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

        compact_errors, compact_failed = _close_stream(compact)
        debug_errors, debug_failed = _close_stream(debug)
        errors = [*compact_errors, *debug_errors]

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
        for error in errors:
            self._report(error)
        if errors:
            _raise_stream_close_errors(errors)

    def _write_snap(self, line: str) -> None:
        if self._compact is None:
            return
        if self._debug is not None:
            self._submit(_StreamWrite(_DEBUG, None, True))
        self._submit(_StreamWrite(_COMPACT, line, True))
        self.drain()

    def _submit(self, operation: _StreamWrite) -> None:
        """A full queue drops only this row; NAV writes it synchronously."""
        try:
            self._worker.submit(operation)
        except NavigationStreamBacklogError as error:
            self._report(error)

    def _report(self, error: BaseException) -> None:
        with self._condition:
            if self._failure is None:
                self._failure = error
        if self._on_error is None:
            return
        try:
            self._on_error(error)
        except Exception:  # noqa: BLE001 - the record stays; keep writing
            pass

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
    """Open and initialise both navigation CSV streams independently.

    A setup failure is logged with the evidence marker (todo 290) and kept as
    the streams' failure; NavPy runs on without that CSV.
    """
    def on_error(error: BaseException) -> None:
        report_evidence_failure(logger, "navigation log write", error)

    try:
        log_path = _create_log_directory(logger)
    except OSError as error:
        report_evidence_failure(logger, "navigation log directory", error)
        return NavigationLogStreams(None, None, on_error=on_error, setup_error=error)
    setup_errors: list[OSError] = []
    compact = _open_stream(
        os.path.join(log_path, f"uav_{sys_id}_navigation_compact.csv"),
        PRIMARY_SCHEMA.header_line(),
        "compact CSV",
        logger,
        setup_errors,
    )
    debug = _open_stream(
        os.path.join(log_path, f"uav_{sys_id}_navigation_debug.csv"),
        STANDARD_EVENT_HEADER,
        "navigation debug CSV",
        logger,
        setup_errors,
    )
    return NavigationLogStreams(
        compact,
        debug,
        on_error=on_error,
        setup_error=setup_errors[0] if setup_errors else None,
    )


def _create_log_directory(logger: ILogger) -> str:
    log_path = getattr(logger, "log_path", None)
    if not log_path:
        today = rt.now()
        log_path = f".logs/{today.date()}/{today.strftime('%H%M%S')}/"
    os.makedirs(log_path, exist_ok=True)
    return log_path


def _open_stream(
    path: str,
    header: str,
    label: str,
    logger: ILogger,
    setup_errors: list[OSError],
) -> Optional[TextIO]:
    stream: Optional[TextIO] = None
    try:
        stream = open(path, "w", encoding="utf-8")
        stream.write(header + "\n")
        stream.flush()
        return stream
    except OSError as exc:
        setup_errors.append(exc)
        report_evidence_failure(logger, f"creating {label}", exc)
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
