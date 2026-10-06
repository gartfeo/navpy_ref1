"""Asynchronous compatibility logger composed from focused policies."""

from __future__ import annotations

import logging
import traceback
from typing import Optional

from navpy.args.logger_args import LoggerArgs, LoggerArgsStub, LogStatusDest
from navpy.logger.cache_log_dedupe import CacheLogDeduplicator
from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_log_message import LogMessage
from navpy.logger.cache_log_output import (
    CacheLogOutput,
    CacheStatusOutput,
    TextLogSink,
)
from navpy.logger.cache_log_worker import CacheLogWorker
from navpy.logger.logger_api import ConsoleLogger, ILogger
from navpy.logger.status_logger import IStatusLogger

_diag_log = logging.getLogger(__name__)


def _make_non_string_status_tracer(max_traces: int = 5):
    """Capture the caller before a malformed status crosses the worker."""
    state = {"count": 0}

    def trace(level: str, status: object) -> None:
        state["count"] += 1
        count = state["count"]
        if count <= max_traces:
            _diag_log.error(
                "CacheLogger.%s called with non-string status "
                "(count=%d, type=%s, repr=%r). Caller stack:\n%s",
                level,
                count,
                type(status).__name__,
                status,
                "".join(traceback.format_stack()[:-2]),
            )
        elif count == max_traces + 1:
            _diag_log.error(
                "CacheLogger non-string status repeated; suppressing "
                "further stacks (latest level=%s, type=%s)",
                level,
                type(status).__name__,
            )

    return trace


_trace_non_string_status = _make_non_string_status_tracer()


class CacheLogger(ILogger):
    """Filter and enqueue logs while collaborators own output behavior."""

    def __init__(
        self,
        logger_args: LoggerArgs,
        logger: TextLogSink,
        status_logger: IStatusLogger | None = None,
    ) -> None:
        self._args = logger_args
        self._level = logger_args.log_level
        self._prefix: object = None
        self._dedupe = CacheLogDeduplicator()
        self._status_output = CacheStatusOutput(logger_args, status_logger)
        self._output = CacheLogOutput(logger, self._status_output)
        self._worker = CacheLogWorker(self._output.handle)
        self.log_path: str | None = None

    @property
    def status_logger(self) -> IStatusLogger | None:
        return self._status_output.sink

    def with_prefix(self, prefix: object) -> "CacheLogger":
        self._prefix = prefix
        return self

    def is_enabled_for(self, level: CacheLogLevel) -> bool:
        return self._level <= level

    def verbose(self, msg: object) -> None:
        if self._level <= CacheLogLevel.VERBOSE:
            self._submit(CacheLogLevel.VERBOSE, self._prefixed(msg))

    def debug(self, msg: object) -> None:
        if self._level <= CacheLogLevel.DEBUG:
            self._submit(CacheLogLevel.DEBUG, self._prefixed(msg))

    def info(
        self,
        msg: object,
        key: str = "",
        status: object = None,
        dest: Optional[LogStatusDest] = None,
        check_interval: bool = False,
    ) -> None:
        if self._level > CacheLogLevel.INFO:
            return
        if not self._dedupe.accept_info(key, msg):
            return
        rendered = self._prefixed(msg)
        if status is None and dest is not None:
            status = rendered
        if status is not None and not isinstance(status, str):
            _trace_non_string_status("info", status)
        self._submit(
            CacheLogLevel.INFO,
            rendered,
            key=key,
            status=status,
            dest=dest,
            check_interval=check_interval,
        )

    def warning(
        self,
        msg: object,
        key: str = "",
        status: object = None,
        dest: Optional[LogStatusDest] = None,
    ) -> None:
        if self._level > CacheLogLevel.WARNING:
            return
        if not self._dedupe.accept_warning(msg):
            return
        if status is not None and not isinstance(status, str):
            _trace_non_string_status("warning", status)
        self._submit(
            CacheLogLevel.WARNING,
            self._prefixed(msg),
            key=key,
            status=status,
            dest=dest,
        )

    def single_warning(self, msg: object, key: str) -> None:
        if (
            self._level <= CacheLogLevel.WARNING
            and self._dedupe.accept_single_warning(key)
        ):
            self._submit(
                CacheLogLevel.WARNING,
                self._prefixed(msg),
                key=key,
            )

    def error(self, msg: object, ex: BaseException | None = None) -> None:
        if self._level > CacheLogLevel.ERROR:
            return
        rendered = str(msg)
        if ex:
            rendered += "\n" + "".join(
                traceback.format_exception(type(ex), ex, ex.__traceback__)
            )
        self._submit(CacheLogLevel.ERROR, self._prefixed(rendered))

    def defer_status_texts(
        self,
        enable: bool,
        *,
        flush: bool = True,
    ) -> None:
        self._status_output.defer(enable, flush=flush)

    def refresh(self) -> None:
        self._args.refresh()

    def close(self) -> None:
        self.info("Logger closed", dest=None)
        self._worker.close()
        self._status_output.close()

    def _prefixed(self, msg: object) -> object:
        return f"{self._prefix}: {msg}" if self._prefix else msg

    def _submit(
        self,
        level: CacheLogLevel,
        msg: object,
        *,
        key: str = "",
        status: object = None,
        dest: LogStatusDest | None = None,
        check_interval: bool = False,
    ) -> bool:
        return self._worker.submit(LogMessage(
            level,
            msg,
            key,
            status,
            dest,
            check_interval,
        ))


class ConsoleCacheLogger(CacheLogger):
    def __init__(self, level: CacheLogLevel = CacheLogLevel.DEBUG) -> None:
        super().__init__(LoggerArgsStub(), ConsoleLogger(level))


__all__ = [
    "CacheLogger",
    "ConsoleCacheLogger",
    "ConsoleLogger",
    "ILogger",
    "IStatusLogger",
    "LogMessage",
    "_make_non_string_status_tracer",
]
