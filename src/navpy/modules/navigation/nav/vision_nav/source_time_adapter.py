"""Failure-isolated adapter from final-approach timing ports to debug CSV rows."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable

from navpy.modules.navigation.nav.vision_nav.source_time_ports import (
    CommandOutcome,
    ObservationOutcome,
    SourceNow,
)
import navpy.modules.vehicle.pose_cadence_debug as pose_cadence_debug


_LOG = logging.getLogger(__name__)


class PoseCadenceFinalApproachSourceTimeObserver:
    """Record diagnostics without retaining a vehicle or affecting control."""

    def __init__(
        self,
        sys_id: int,
        wall_time_s: Callable[[], float] = time.time,
    ) -> None:
        if type(sys_id) is not int:
            raise TypeError("source-time observer sys_id must be an integer")
        self._sys_id = sys_id
        self._wall_time_s = wall_time_s

    def record_observation(
        self,
        *,
        source_timestamp_s: float | None,
        source_now_s: SourceNow | None,
        outcome: ObservationOutcome,
    ) -> None:
        if not pose_cadence_debug.ENABLED:
            return
        self._isolate(
            "observation",
            lambda: pose_cadence_debug.record_observation(
                self._sys_id,
                self._wall_time_s(),
                source_timestamp_s,
                _read_source_now(source_now_s),
                outcome,
            ),
        )

    def record_command(
        self,
        *,
        wall_start_s: float,
        source_timestamp_s: float,
        source_now_s: SourceNow | None,
        execution_ms: float,
        outcome: CommandOutcome = "fresh",
    ) -> None:
        if not pose_cadence_debug.ENABLED:
            return
        self._isolate(
            "command",
            lambda: pose_cadence_debug.record_worker(
                self._sys_id,
                wall_start_s,
                execution_ms,
                source_timestamp_s,
                _read_source_now(source_now_s),
                "measured",
                outcome,
            ),
        )

    @staticmethod
    def _isolate(stage: str, operation: Callable[[], None]) -> None:
        """Explicit logged boundary: diagnostics must never affect flight."""
        try:
            operation()
        except Exception as error:  # noqa: BLE001 - logged diagnostic isolation
            _LOG.warning(
                "final-approach source-time %s observer failed: %s: %s",
                stage,
                type(error).__name__,
                error,
                exc_info=True,
            )


def _read_source_now(provider: SourceNow | None) -> float | None:
    return None if provider is None else provider()


__all__ = ["PoseCadenceFinalApproachSourceTimeObserver"]
