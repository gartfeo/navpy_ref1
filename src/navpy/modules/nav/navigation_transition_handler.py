"""Atomic dispatch of navigation state transition hooks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.navigation_task_reset import NavigationTaskResetTransaction
from navpy.modules.nav.nav_transition import NavExitOutcome, NavTransition
from navpy.modules.nav.nav_state import ConfirmGateState, NavState


@dataclass(frozen=True)
class TransitionOutcome:
    effective_state: NavState
    oneshot_completed: bool = False


@dataclass(frozen=True)
class TransitionPorts:
    decision_clock_s: Callable[[], float]
    restart_auto_mission: Callable[[], None]
    enter_recovery: Callable[[], None]
    start_task_actor: Callable[[], None]


class NavigationTransitionHandler:
    """Apply enter/exit hooks exactly once per state change."""

    def __init__(
        self,
        ports: TransitionPorts,
        confirm: ConfirmGateState,
        nav: NavTransition,
        reset: NavigationTaskResetTransaction,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._confirm = confirm
        self._nav = nav
        self._reset = reset
        self._logger = logger
        self._exit_hooks: dict[NavState, Callable[[], NavExitOutcome]] = {
            NavState.ONHOLD: self._no_exit,
            NavState.DETECT: self._no_exit,
            NavState.CONFIRM: self._exit_confirmation,
            NavState.NAV: self._nav.exit,
            NavState.RESET: self._no_exit,
            NavState.RECOVERY: self._no_exit,
        }
        self._entry_hooks: dict[NavState, Callable[[], None]] = {
            NavState.ONHOLD: self._no_entry,
            NavState.DETECT: self._ports.start_task_actor,
            NavState.CONFIRM: self._enter_confirmation,
            NavState.NAV: self._enter_nav,
            NavState.RESET: self._enter_reset_active,
            NavState.RECOVERY: self._enter_recovery,
        }
        self._pair_hooks: dict[tuple[NavState, NavState], Callable[[], None]] = {
            (NavState.RESET, NavState.DETECT): self._ports.restart_auto_mission,
            (NavState.RECOVERY, NavState.DETECT): self._ports.restart_auto_mission,
        }
        expected = set(NavState)
        if set(self._exit_hooks) != expected or set(self._entry_hooks) != expected:
            raise ValueError("transition maps must cover every NavState")

    def on_change(
        self,
        previous: NavState,
        requested: NavState,
    ) -> TransitionOutcome:
        exit_outcome = self._exit_hooks[previous]()
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
    def _no_exit() -> NavExitOutcome:
        return NavExitOutcome()

    @staticmethod
    def _no_entry() -> None:
        return None

    def _exit_confirmation(self) -> NavExitOutcome:
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
