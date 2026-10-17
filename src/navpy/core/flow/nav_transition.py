"""NAV transition facade over focused entry and exit owners."""

from __future__ import annotations

from navpy.core.kernel.exception_groups import ExceptionGroup

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.approach.nav_entry import NavEntry
from navpy.core.approach.nav_exit_cleanup import NavCleanupResult, NavExitCleanup
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.flow.nav_oneshot_completion import OneShotCompletion
from navpy.core.approach.nav_snap_reporter import NavSnapReporter
from navpy.core.flow.nav_state import NavigationTaskState, NavState


@dataclass(frozen=True)
class NavExitOutcome:
    redirect: NavState | None = None
    oneshot_completed: bool = False


class NavTransition:
    """Coordinate NAV entry, fenced cleanup, diagnostics, and completion."""

    def __init__(
        self,
        entry: NavEntry,
        cleanup: NavExitCleanup,
        snap_reporter: NavSnapReporter,
        oneshot: OneShotCompletion,
        navigation_task: NavigationTaskState,
        is_oneshot: Callable[[], bool],
        *,
        logger: Optional[ILogger] = None,
    ) -> None:
        self._entry = entry
        self._cleanup = cleanup
        self._snap_reporter = snap_reporter
        self._oneshot = oneshot
        self._navigation_task = navigation_task
        self._is_oneshot = is_oneshot
        self._logger = logger

    def enter(self) -> None:
        self._entry.run()

    def exit(self, requested: NavState | None = None) -> NavExitOutcome:
        navigation_active = self._navigation_task.final_approach_navigation_active
        completed = self._navigation_task.final_approach_nav_completed
        cleanup = self._cleanup.run()
        errors = list(cleanup.errors)

        if navigation_active and cleanup.snap is not None:
            try:
                self._snap_reporter.report(cleanup.snap)
            except Exception as error:  # noqa: BLE001 - teardown diagnostics
                errors.append(error)

        outcome = NavExitOutcome()
        docked = (
            requested is NavState.RESET
            and navigation_active
            and not errors
            and cleanup.own_dock_contact is not None
        )
        if docked:
            # One-shot is how a real dock ends: disarm off the net, final.
            oneshot = self._is_oneshot()
            if oneshot:
                errors.extend(self._oneshot.run())
            outcome = NavExitOutcome(
                redirect=NavState.DOCKED,
                oneshot_completed=oneshot,
            )
        elif self._is_oneshot() and completed:
            errors.extend(self._oneshot.run())
            outcome = NavExitOutcome(
                redirect=NavState.ONHOLD,
                oneshot_completed=True,
            )

        if errors:
            raise ExceptionGroup("NAV exit failed", errors)
        if docked:
            self._report_docked(cleanup)
        return outcome

    def _report_docked(self, cleanup: NavCleanupResult) -> None:
        """Operator status line, once the exit succeeded."""
        if self._logger is None:
            return
        event = cleanup.own_dock_contact
        identity = event.identity
        self._logger.info(
            f"STATE DOCKED: contact 3d={event.closest_3d_m:.2f}m "
            f"dock={identity.dock_uid} task={identity.task_id}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["NavExitOutcome", "NavTransition"]
