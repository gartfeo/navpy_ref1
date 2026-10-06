"""Operator-visible confirmation status reporters."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger


class ConfirmDebugReporter:
    """Rate-limit diagnostic waits without storing state on a controller."""

    def __init__(
        self,
        logger: ILogger,
        clock_s: Callable[[], float],
    ) -> None:
        self._logger = logger
        self._clock_s = clock_s
        self.last_emit_s = 0.0

    def log(self, reason: str) -> None:
        now_s = self._clock_s()
        if now_s - self.last_emit_s < 1.0:
            return
        self.last_emit_s = now_s
        self._logger.info(f"CONFIRM_WAIT: {reason}", key="nav")


class ConfirmBlockedReporter:
    """Own the live CONFIRM_BLOCKED status and its link-rate limit."""

    def __init__(
        self,
        logger: ILogger,
        clock_s: Callable[[], float],
    ) -> None:
        self._logger = logger
        self._clock_s = clock_s
        self.reason: Optional[str] = None
        self.target_id: Optional[int] = None
        self.last_emit_s = 0.0

    def emit(
        self,
        target_id: int,
        reason: str,
        detail: str = "",
    ) -> None:
        target_changed = target_id != self.target_id
        reason_changed = reason != self.reason
        self.target_id = target_id
        self.reason = reason
        now_s = self._clock_s()
        if (
            not target_changed
            and not reason_changed
            and now_s - self.last_emit_s < 1.0
        ):
            return
        self.last_emit_s = now_s
        self._logger.info(
            f"CONFIRM_BLOCKED:{reason}|{detail}|{target_id}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def clear(self) -> None:
        if self.reason is None:
            return
        self.reason = None
        self.target_id = None
        self.last_emit_s = 0.0
        self._logger.info(
            "CONFIRM_BLOCKED:clear",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["ConfirmBlockedReporter", "ConfirmDebugReporter"]
