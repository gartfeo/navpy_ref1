"""Navigation status text reporting."""

from __future__ import annotations

from typing import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger


class NavigationStatusReporter:
    def __init__(
        self,
        logger: ILogger,
        args: NavArgs,
        home_location: Callable[[], object],
    ) -> None:
        self._logger = logger
        self._args = args
        self._home_location = home_location
        self.last_ignore_code = 0

    def ignore(self, code: int, message: str) -> None:
        if code == self.last_ignore_code:
            return
        self.last_ignore_code = code
        self._logger.info(
            message,
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def initial(self) -> None:
        home = self._home_location()
        self._logger.info(
            f"N: {self._args.confirm_wait_time_sec}s, "
            f"min_wp: {self._args.min_wp}; "
            f"min_alt: {self._args.min_alt} m; "
            f"home: {home if home else 'N/A'}"
        )


__all__ = ["NavigationStatusReporter"]
