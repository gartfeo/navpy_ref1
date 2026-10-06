"""NAV transition facade over focused entry and exit owners."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.nav.nav_entry import NavEntry
from navpy.modules.nav.nav_exit_cleanup import NavExitCleanup
from navpy.modules.nav.nav_oneshot_completion import OneShotCompletion
from navpy.modules.nav.nav_snap_reporter import NavSnapReporter
from navpy.modules.nav.nav_state import NavigationTaskState, NavState


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
    ) -> None:
        self._entry = entry
        self._cleanup = cleanup
        self._snap_reporter = snap_reporter
        self._oneshot = oneshot
        self._navigation_task = navigation_task
        self._is_oneshot = is_oneshot

    def enter(self) -> None:
        self._entry.run()

    def exit(self) -> NavExitOutcome:
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
        if self._is_oneshot() and completed:
            errors.extend(self._oneshot.run())
            outcome = NavExitOutcome(
                redirect=NavState.ONHOLD,
                oneshot_completed=True,
            )

        if errors:
            raise ExceptionGroup("NAV exit failed", errors)
        return outcome


__all__ = ["NavExitOutcome", "NavTransition"]
