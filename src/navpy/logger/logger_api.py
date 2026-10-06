"""Stable logger API and the synchronous console implementation."""

from __future__ import annotations

import traceback
from abc import ABC, abstractmethod
from typing import Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_log_level import CacheLogLevel


class ILogger(ABC):
    @abstractmethod
    def is_enabled_for(self, level: CacheLogLevel) -> bool:
        raise NotImplementedError

    @abstractmethod
    def with_prefix(self, prefix: object) -> "ILogger":
        raise NotImplementedError

    @abstractmethod
    def verbose(self, msg: object) -> None:
        raise NotImplementedError

    @abstractmethod
    def debug(self, msg: object) -> None:
        raise NotImplementedError

    @abstractmethod
    def info(
        self,
        msg: object,
        key: str = "",
        status: object = None,
        dest: Optional[LogStatusDest] = None,
        check_interval: bool = False,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def warning(
        self,
        msg: object,
        key: str = "",
        status: object = None,
        dest: Optional[LogStatusDest] = None,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def single_warning(self, msg: object, key: str) -> None:
        raise NotImplementedError

    @abstractmethod
    def error(self, msg: object, ex: BaseException | None = None) -> None:
        raise NotImplementedError

    @abstractmethod
    def defer_status_texts(
        self,
        enable: bool,
        *,
        flush: bool = True,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def refresh(self) -> None:
        raise NotImplementedError


class ConsoleLogger(ILogger):
    def __init__(self, level: CacheLogLevel = CacheLogLevel.DEBUG) -> None:
        self._level = level
        self._prefix = ""

    def with_prefix(self, prefix: object) -> "ConsoleLogger":
        self._prefix = str(prefix)
        return self

    def is_enabled_for(self, level: CacheLogLevel) -> bool:
        return self._level <= level

    def verbose(self, msg: object) -> None:
        if self._level >= CacheLogLevel.VERBOSE:
            print(msg)

    def debug(self, msg: object) -> None:
        if self._level <= CacheLogLevel.DEBUG:
            print(msg)

    def info(
        self,
        msg: object,
        key: str = "",
        status: object = None,
        dest: Optional[LogStatusDest] = None,
        check_interval: bool = False,
    ) -> None:
        del key, dest, check_interval
        if self._level <= CacheLogLevel.INFO:
            print(msg if status is None else status)

    def warning(
        self,
        msg: object,
        key: str = "",
        status: object = None,
        dest: Optional[LogStatusDest] = None,
    ) -> None:
        del key, status, dest
        if self._level <= CacheLogLevel.WARNING:
            print(msg)

    def single_warning(self, msg: object, key: str) -> None:
        del key
        if self._level <= CacheLogLevel.WARNING:
            print(msg)

    def error(self, msg: object, ex: BaseException | None = None) -> None:
        if self._level > CacheLogLevel.ERROR:
            return
        rendered = str(msg)
        if ex:
            rendered += "\n" + "".join(
                traceback.format_exception(type(ex), ex, ex.__traceback__)
            )
        print(rendered)

    def defer_status_texts(
        self,
        enable: bool,
        *,
        flush: bool = True,
    ) -> None:
        del enable, flush
        print("Defer status texts is not supported in ConsoleLogger")

    def refresh(self) -> None:
        print("Refreshing logger")

    def close(self) -> None:
        print("Closing logger")


__all__ = ["ConsoleLogger", "ILogger"]
