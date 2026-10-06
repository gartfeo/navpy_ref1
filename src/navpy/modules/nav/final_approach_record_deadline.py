"""Deadline handling for final-approach confirmation record commits."""

from __future__ import annotations

from typing import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_reporting import ConfirmDebugReporter
from navpy.modules.nav.nav_constants import FINAL_APPROACH_RECORD_TIMEOUT_S
from navpy.modules.nav.nav_state import FinalApproachNavState
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


class FinalApproachRecordDeadline:
    """Bound manual-confirm wait for a recordable final-approach observation."""

    def __init__(
        self,
        state: FinalApproachNavState,
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

    def defer_or_fail(self, poi: DetectedObject) -> None:
        now_s = self._clock_s()
        if self._state.deferred_record_started_at is None:
            self._state.deferred_record_started_at = now_s
        elif (
            now_s - self._state.deferred_record_started_at
            > FINAL_APPROACH_RECORD_TIMEOUT_S
        ):
            self._mark_failed()
            try:
                self._logger.info(
                    "NAV_FAIL: final_approach_record_unavailable after "
                    f"{FINAL_APPROACH_RECORD_TIMEOUT_S:g}s "
                    f"task={get_poi_task_id(poi)} "
                    f"obj={poi.identity.obj_id}",
                    key="nav",
                    dest=LogStatusDest.DRONE,
                )
            except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
                pass
            return
        self._debug.log(
            "final_approach_record_deferred_unavailable "
            f"obj={poi.identity.obj_id}"
        )


__all__ = ["FinalApproachRecordDeadline"]
