"""Runtime strategies used by ``Navigation``.

The selected navigation algorithm owns the runtime behavior needed by the
termination worker. Legacy geo-assisted laws enqueue target NED work, while
vision-nav laws provide their own observation pipeline in the
vision-nav package.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Iterable, Optional, Protocol

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_postprocess_dispatcher import PostprocessJob
from navpy.modules.vision.models.detect_data import DetectedObject

if TYPE_CHECKING:
    import numpy as np

    from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot


class NavigationIngress(Protocol):
    """Detection-admission capability used by :class:`Navigation`."""

    def nav(self, detect_data: DetectedObject) -> bool:
        ...


class CommandWorkRuntime(Protocol):
    """Minimal queued-command capability consumed by the worker thread."""

    def has_command_pending_or_in_flight(self) -> bool:
        ...

    def take_work(self) -> Optional[Any]:
        ...

    def execute_work(self, work: Any) -> Optional[CalcData]:
        ...

    def finish_work(self, work: Any) -> None:
        ...

    def postprocess_job(self, work: Any) -> PostprocessJob | None:
        ...


class RuntimeLifecycle(Protocol):
    """Phase/source reset capability shared by navigation runtimes."""

    def invalidate_commands(self) -> None:
        ...

    def reset_phase(self) -> None:
        ...

    def clear_source_discontinuity_state(
        self,
        source_names: str | Iterable[str],
    ) -> None:
        ...

    def consume_command_liveness_failure(self) -> bool:
        ...


class NavigationRuntime(
    NavigationIngress,
    CommandWorkRuntime,
    RuntimeLifecycle,
    Protocol,
):
    """Structural contract implemented by each algorithm runtime."""

class LegacyNavigationRuntime(NavigationRuntime):
    """Legacy geo-assisted stream built from three narrow dependencies."""

    def __init__(
            self,
            *,
            command_slot: NavigationCommandSlot,
            target_resolver: Callable[
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
        self._target_resolver = target_resolver
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
        d_target_ned, _ = self._target_resolver(detect_data)
        if d_target_ned is None:
            return False

        self._command_slot.replace((d_target_ned, detect_data))
        self._command_slot.signal_pending()
        return True

    def take_work(self) -> Optional[Any]:
        # Peek, do not consume: the legacy PN/PID terminate loop streams
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


__all__ = [
    "CommandWorkRuntime",
    "NavigationIngress",
    "NavigationRuntime",
    "LegacyNavigationRuntime",
    "RuntimeLifecycle",
]
