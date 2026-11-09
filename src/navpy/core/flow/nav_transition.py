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
from navpy.core.approach.nav_snap_reporter import NavSnapReporter
from navpy.core.approach.vision_nav.episode_latch import (
    UNNAMED_FAILURE_REASON,
    GuidanceEpisodeResult,
    GuidanceResult,
)
from navpy.core.flow.nav_display import NavOutcomeLatch
from navpy.core.flow.nav_held_approach import HeldApproach
from navpy.core.flow.nav_state import NavigationTaskState, NavState


@dataclass(frozen=True)
class NavExitOutcome:
    redirect: NavState | None = None
    oneshot_completed: bool = False


class NavTransition:
    """Coordinate NAV entry, fenced cleanup, diagnostics, and completion.

    ``finish_sortie`` ends with the task teardown;
    ``NavigationTransitionHandler._enter_docked`` relies on it for a one-shot
    DOCKED entry, contact or dock failed.
    """

    def __init__(
        self,
        entry: NavEntry,
        cleanup: NavExitCleanup,
        snap_reporter: NavSnapReporter,
        finish_sortie: Callable[[], tuple[Exception, ...]],
        navigation_task: NavigationTaskState,
        is_oneshot: Callable[[], bool],
        *,
        logger: Optional[ILogger] = None,
        outcome: Optional[NavOutcomeLatch] = None,
        held: Optional[HeldApproach] = None,
    ) -> None:
        self._entry = entry
        self._cleanup = cleanup
        self._snap_reporter = snap_reporter
        self._finish_sortie = finish_sortie
        self._navigation_task = navigation_task
        self._is_oneshot = is_oneshot
        self._logger = logger
        self._outcome = outcome
        self._held = held
        if held is not None:
            held.bind(self._end_held_episode, self.enter)

    def enter(self) -> None:
        self._entry.run()
        if self._held is not None:
            self._held.clear()       # only once the entry succeeded (nav_held_approach.py)

    def _end_held_episode(self) -> None:
        """Recovery's hold: the in-flight episode ends without failure (owner ruling 2026-11-09)."""
        self._cleanup.end_episode()
        self._navigation_task.guidance_navigation_active = False
        if self._logger is not None:
            self._logger.info("NAV approach held: episode ended, a fresh one follows the resume", key="nav")

    def exit(self, requested: NavState | None = None) -> NavExitOutcome:
        navigation_active = self._navigation_task.guidance_navigation_active
        completed = self._navigation_task.guidance_nav_completed
        cleanup = self._cleanup.run()
        errors = list(cleanup.errors)

        if navigation_active and cleanup.snap is not None:
            try:
                self._snap_reporter.report(cleanup.snap)
            except Exception as error:  # noqa: BLE001 - teardown diagnostics
                errors.append(error)

        outcome = NavExitOutcome()
        dock_ending = (
            requested is NavState.RESET and navigation_active and not errors
        )
        docked = dock_ending and cleanup.own_dock_contact is not None
        # A started approach that ended by NAV_FAIL without contact is a
        # failed dock: it ends like DOCKED, never back to search
        # (missed_dock.md). The result names an armed (sim) episode.
        failed = (
            dock_ending
            and not docked
            and _failed_result(cleanup.dock_result)
        )
        if docked or failed:
            # One-shot is how a real dock ends: the sortie finishes, final.
            oneshot = self._is_oneshot()
            if oneshot:
                errors.extend(self._finish_sortie())
            outcome = NavExitOutcome(
                redirect=NavState.DOCKED,
                oneshot_completed=oneshot,
            )
        elif self._is_oneshot() and completed:
            errors.extend(self._finish_sortie())
            outcome = NavExitOutcome(
                redirect=NavState.ONHOLD,
                oneshot_completed=True,
            )

        if errors:
            raise ExceptionGroup("NAV exit failed", errors)
        if self._held is not None:
            self._held.clear()       # NAV left: an abandoned held approach is over only now
        if docked:
            self._report_docked(cleanup)
        elif failed:
            self._report_dock_failed(cleanup.dock_result)
        return outcome

    def _report_dock_failed(
        self,
        result: GuidanceEpisodeResult | None,
    ) -> None:
        """Evidence row and operator status line, once the exit succeeded."""
        if result is None:
            return
        reason = result.reason or UNNAMED_FAILURE_REASON
        self._cleanup.record_dock_failed(result, reason)
        if self._outcome is not None:
            self._outcome.record_failed(reason)
        if self._logger is None:
            return
        identity = result.identity
        self._logger.info(
            f"STATE DOCK_FAILED (sim): reason={reason} "
            f"dock={identity.dock_uid} task={identity.task_id}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def _report_docked(self, cleanup: NavCleanupResult) -> None:
        """Operator status line, once the exit succeeded."""
        if self._outcome is not None:
            self._outcome.record_docked()
        if self._logger is None:
            return
        event = cleanup.own_dock_contact
        identity = event.identity
        self._logger.info(
            f"STATE DOCKED (sim): 3d={event.closest_3d_m:.2f}m "
            f"dock={identity.dock_uid} task={identity.task_id}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


def _failed_result(result: GuidanceEpisodeResult | None) -> bool:
    return result is not None and result.outcome in (
        GuidanceResult.FAILED,
        GuidanceResult.MISSED,
    )


__all__ = ["NavExitOutcome", "NavTransition"]
