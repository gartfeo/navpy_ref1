"""Deadline handling for guidance confirmation record commits."""

from __future__ import annotations

from typing import Callable

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.confirm.confirmation_reporting import ConfirmDebugReporter
from navpy.core.flow.nav_constants import GUIDANCE_RECORD_TIMEOUT_S
from navpy.core.flow.nav_state import GuidanceNavState
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.approach.vision_nav import nav_fail_reason as fail_reason


class GuidanceRecordDeadline:
    """Bound manual-confirm wait for a recordable guidance observation."""

    def __init__(
        self,
        state: GuidanceNavState,
        mark_failed: Callable[[str], object],
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
            > GUIDANCE_RECORD_TIMEOUT_S
        ):
            self._mark_failed(fail_reason.GUIDANCE_RECORD_UNAVAILABLE)
            try:
                self._logger.info(
                    "NAV_FAIL: guidance_record_unavailable after "
                    f"{GUIDANCE_RECORD_TIMEOUT_S:g}s "
                    f"task={get_poi_task_id(poi)} "
                    f"obj={poi.identity.obj_id}",
                    key="nav",
                    dest=LogStatusDest.DRONE,
                )
            except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
                pass
            return
        self._debug.log(
            "guidance_record_deferred_unavailable "
            f"obj={poi.identity.obj_id}"
        )


__all__ = ["GuidanceRecordDeadline"]
