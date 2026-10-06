"""Shared logger reference for collaborators that support runtime replacement."""

from navpy.logger.cache_logger import ILogger


class LoggerRef:
    def __init__(self, logger: ILogger) -> None:
        self._logger = logger

    @property
    def value(self) -> ILogger:
        return self._logger

    def replace(self, logger: ILogger) -> None:
        self._logger = logger
