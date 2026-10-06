"""Message value passed from logger callers to the output worker."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_log_level import CacheLogLevel


@dataclass
class LogMessage:
    level: CacheLogLevel
    msg: object
    key: str = ""
    status: object = None
    dest: LogStatusDest | None = None
    check_interval: bool = False


__all__ = ["LogMessage"]
