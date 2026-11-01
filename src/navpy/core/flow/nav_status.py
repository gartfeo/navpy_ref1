"""Navigation status text reporting."""

from __future__ import annotations

from typing import Callable

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger


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
        """Log the effective settings that shape the POI review and its exits."""
        home = self._home_location()
        args = self._args
        self._logger.info(
            f"N: {args.confirm_wait_time_sec}s, "
            f"min_wp: {args.min_wp}; "
            f"min_alt: {args.min_alt} m; "
            f"auto_confirm: {bool(args.is_auto_confirm)}; "
            f"confirm_on_fail: {bool(args.is_confirm_on_fail)}; "
            f"oneshot: {args.is_oneshot}; "
            f"gate: {args.confirm_gate_timeout_sec}s; "
            f"home: {home if home else 'N/A'}"
        )


__all__ = ["NavigationStatusReporter"]
