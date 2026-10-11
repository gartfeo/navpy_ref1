"""Runtime strategies used by ``Navigation``.

The selected navigation algorithm owns the runtime behavior needed by the
final-approach worker. Legacy geo-assisted laws enqueue POI NED work, while
vision-nav laws provide their own observation pipeline in the
vision-nav package.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Protocol

from navpy.core.approach.calc_data import CalcData
from navpy.core.approach.navigation_postprocess_dispatcher import PostprocessJob
from navpy.core.ports.detect_data import DetectedObject


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


__all__ = [
    "CommandWorkRuntime",
    "NavigationIngress",
    "NavigationRuntime",
    "RuntimeLifecycle",
]
