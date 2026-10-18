"""Atomic dispatch of navigation state transition hooks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.flow.navigation_task_reset import NavigationTaskResetTransaction
from navpy.core.flow.nav_transition import NavExitOutcome, NavTransition
from navpy.core.flow.nav_state import ConfirmGateState, NavState, PostDockClimb


@dataclass(frozen=True)
class TransitionOutcome:
    effective_state: NavState
    oneshot_completed: bool = False


@dataclass(frozen=True)
class TransitionPorts:
    decision_clock_s: Callable[[], float]
    return_to_search: Callable[[], None]
    enter_recovery: Callable[[], None]
    start_task_actor: Callable[[], None]
    is_oneshot: Callable[[], bool] = lambda: False


class NavigationTransitionHandler:
    """Apply enter/exit hooks exactly once per state change."""

    def __init__(
        self,
        ports: TransitionPorts,
        confirm: ConfirmGateState,
        nav: NavTransition,
        reset: NavigationTaskResetTransaction,
        logger: ILogger,
        post_dock_climb: PostDockClimb | None = None,
    ) -> None:
        self._ports = ports
        self._confirm = confirm
        self._nav = nav
        self._reset = reset
        self._logger = logger
        self._post_dock_climb = post_dock_climb or PostDockClimb()
        # Each exit hook gets the requested state; only NAV uses it.
        self._exit_hooks: dict[NavState, Callable[[NavState], NavExitOutcome]] = {
            NavState.ONHOLD: self._no_exit,
            NavState.DETECT: self._no_exit,
            NavState.CONFIRM: self._exit_confirmation,
            NavState.NAV: self._nav.exit,
            NavState.RESET: self._no_exit,
            NavState.RECOVERY: self._exit_recovery,
            NavState.DOCKED: self._no_exit,
        }
        self._entry_hooks: dict[NavState, Callable[[], None]] = {
            NavState.ONHOLD: self._no_entry,
            NavState.DETECT: self._ports.start_task_actor,
            NavState.CONFIRM: self._enter_confirmation,
            NavState.NAV: self._enter_nav,
            NavState.RESET: self._enter_reset_active,
            NavState.RECOVERY: self._enter_recovery,
            NavState.DOCKED: self._enter_docked,
        }
        self._pair_hooks: dict[tuple[NavState, NavState], Callable[[], None]] = {
            (NavState.RESET, NavState.DETECT): self._ports.return_to_search,
            (NavState.RECOVERY, NavState.DETECT): self._ports.return_to_search,
            (NavState.DOCKED, NavState.DETECT): self._ports.return_to_search,
            (NavState.DOCKED, NavState.RECOVERY): self._start_post_dock_climb,
        }
        expected = set(NavState)
        if set(self._exit_hooks) != expected or set(self._entry_hooks) != expected:
            raise ValueError("transition maps must cover every NavState")

    def on_change(
        self,
        previous: NavState,
        requested: NavState,
    ) -> TransitionOutcome:
        exit_outcome = self._exit_hooks[previous](requested)
        effective = exit_outcome.redirect or requested
        self._pair_hooks.get((previous, effective), self._no_entry)()
        self._entry_hooks[effective]()
        return TransitionOutcome(
            effective_state=effective,
            oneshot_completed=exit_outcome.oneshot_completed,
        )

    def enter_reset(self) -> None:
        self._reset.clear()
        self._logger.refresh()

    @staticmethod
    def _no_exit(_requested: NavState) -> NavExitOutcome:
        return NavExitOutcome()

    @staticmethod
    def _no_entry() -> None:
        return None

    def _exit_confirmation(self, _requested: NavState) -> NavExitOutcome:
        self._reset_confirmation(None)
        return NavExitOutcome()

    def _enter_confirmation(self) -> None:
        self._reset_confirmation(self._ports.decision_clock_s())
        self._ports.start_task_actor()

    def _enter_nav(self) -> None:
        self._nav.enter()
        self._ports.start_task_actor()

    def _enter_reset_active(self) -> None:
        self.enter_reset()
        self._ports.start_task_actor()

    def _enter_recovery(self) -> None:
        self._ports.enter_recovery()
        self._ports.start_task_actor()

    def _exit_recovery(self, _requested: NavState) -> NavExitOutcome:
        self._post_dock_climb.active = False
        return NavExitOutcome()

    def _start_post_dock_climb(self) -> None:
        self._post_dock_climb.active = True

    def _enter_docked(self) -> None:
        """DOCKED replaces a RESET entry and takes over its cleanup owner.

        One-shot: OneShotCompletion already ran the reset transaction at NAV
        exit, so it is not run twice.
        """
        if not self._ports.is_oneshot():
            self.enter_reset()
        self._ports.start_task_actor()

    def _reset_confirmation(self, entered_at: float | None) -> None:
        self._confirm.entered_at = entered_at
        self._confirm.zoom_had_poi = False
        self._confirm.zoom_seen_poi = False
        self._confirm.resets_used = 0
        self._confirm.loss_started_at = None
        self._confirm.review_started_at = None


__all__ = [
    "NavigationTransitionHandler",
    "TransitionOutcome",
    "TransitionPorts",
]
