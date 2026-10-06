"""Deadline handling for terminal confirmation record commits."""

from __future__ import annotations

from typing import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_reporting import ConfirmDebugReporter
from navpy.modules.nav.nav_constants import TERMINAL_RECORD_TIMEOUT_S
from navpy.modules.nav.nav_state import TerminalNavState
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id


class TerminalRecordDeadline:
    """Bound manual-confirm wait for a recordable terminal observation."""

    def __init__(
        self,
        state: TerminalNavState,
        mark_failed: Callable[[], None],
        clock_s: Callable[[], float],
        logger: ILogger,
        debug: ConfirmDebugReporter,
    ) -> None:
        self._state = state
        self._mark_failed = mark_failed
        self._clock_s = clock_s
        self._logger = logger
        self._debug = debug

    def defer_or_fail(self, target: DetectedObject) -> None:
        now_s = self._clock_s()
        if self._state.deferred_record_started_at is None:
            self._state.deferred_record_started_at = now_s
        elif (
            now_s - self._state.deferred_record_started_at
            > TERMINAL_RECORD_TIMEOUT_S
        ):
            self._mark_failed()
            try:
                self._logger.info(
                    "NAV_FAIL: terminal_record_unavailable after "
                    f"{TERMINAL_RECORD_TIMEOUT_S:g}s "
                    f"task={get_target_task_id(target)} "
                    f"obj={target.identity.obj_id}",
                    key="nav",
                    dest=LogStatusDest.DRONE,
                )
            except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
                pass
            return
        self._debug.log(
            "terminal_record_deferred_unavailable "
            f"obj={target.identity.obj_id}"
        )


__all__ = ["TerminalRecordDeadline"]
