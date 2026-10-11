"""Legacy geo-assisted navigation runtime stream."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Iterable, Optional

from navpy.core.kernel.attitude import Attitude
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_postprocess_dispatcher import PostprocessJob
from navpy.core.ports.detect_data import DetectedObject
from navpy.modules.navigation.navigation_runtime import NavigationRuntime

if TYPE_CHECKING:
    import numpy as np

    from navpy.core.command.navigation_command_slot import NavigationCommandSlot


class LegacyNavigationRuntime(NavigationRuntime):
    """Legacy geo-assisted stream built from three narrow dependencies."""

    def __init__(
            self,
            *,
            command_slot: NavigationCommandSlot,
            poi_resolver: Callable[
                [DetectedObject | None],
                tuple[np.ndarray | None, Attitude | None],
            ],
            command_executor: Callable[
                [np.ndarray, DetectedObject],
                CalcData | None,
            ],
            reset_law: Callable[[], None],
    ) -> None:
        self._command_slot = command_slot
        self._poi_resolver = poi_resolver
        self._command_executor = command_executor
        self._reset_law = reset_law

    def invalidate_commands(self) -> None:
        self._command_slot.invalidate()

    def reset_phase(self) -> None:
        self._command_slot.invalidate()
        self._reset_law()

    def clear_source_discontinuity_state(
        self,
        source_names: str | Iterable[str],
    ) -> None:
        del source_names
        self.invalidate_commands()

    def consume_command_liveness_failure(self) -> bool:
        return False

    def has_command_pending_or_in_flight(self) -> bool:
        return self._command_slot.peek() is not None

    def nav(self, detect_data: DetectedObject) -> bool:
        d_poi_ned, _ = self._poi_resolver(detect_data)
        if d_poi_ned is None:
            return False

        self._command_slot.replace((d_poi_ned, detect_data))
        self._command_slot.signal_pending()
        return True

    def take_work(self) -> Optional[Any]:
        # Peek, do not consume: the legacy PN/PID final-approach loop streams
        # commands continuously from the freshest detection between frames
        # (detections can arrive slower than the command loop). Consuming
        # here turns the loop one-shot and starves the autopilot of attitude
        # targets. Pausing/clearing happens via invalidate_commands.
        self._command_slot.clear_signal()
        return self._command_slot.peek()

    def execute_work(self, work: Any) -> Optional[CalcData]:
        return self._command_executor(*work)

    def finish_work(self, work: Any) -> None:
        del work
        return None

    def postprocess_job(self, work: Any) -> PostprocessJob | None:
        del work
        return None
