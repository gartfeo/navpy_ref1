"""Small authoritative state owners for the navigation application."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum, unique
from typing import Optional

from navpy.modules.common.models.location import Location


@unique
class NavState(Enum):
    ONHOLD = 0
    DETECT = 1
    CONFIRM = 2
    NAV = 3
    RESET = 4
    RECOVERY = 5


@dataclass(frozen=True)
class NavPhaseSnapshot:
    current: NavState
    previous: NavState
    oneshot_completed: bool
    last_parameter_refresh_s: float


class NavPhaseState:
    """Current state-machine phase and episode-level completion latch."""

    def __init__(
        self,
        current: NavState = NavState.ONHOLD,
        previous: NavState = NavState.ONHOLD,
        oneshot_completed: bool = False,
        last_parameter_refresh_s: float = 0.0,
    ) -> None:
        self._current = current
        self._previous = previous
        self._oneshot_completed = bool(oneshot_completed)
        self._last_parameter_refresh_s = float(last_parameter_refresh_s)
        self._lock = threading.RLock()

    @property
    def current(self) -> NavState:
        with self._lock:
            return self._current

    @property
    def previous(self) -> NavState:
        with self._lock:
            return self._previous

    @property
    def oneshot_completed(self) -> bool:
        with self._lock:
            return self._oneshot_completed

    @oneshot_completed.setter
    def oneshot_completed(self, completed: bool) -> None:
        with self._lock:
            self._oneshot_completed = bool(completed)

    @property
    def last_parameter_refresh_s(self) -> float:
        with self._lock:
            return self._last_parameter_refresh_s

    @last_parameter_refresh_s.setter
    def last_parameter_refresh_s(self, timestamp_s: float) -> None:
        with self._lock:
            self._last_parameter_refresh_s = float(timestamp_s)

    def request(self, state: NavState) -> None:
        if not isinstance(state, NavState):
            raise TypeError("navigation phase must be a NavState")
        with self._lock:
            self._current = state

    def is_current(self, state: NavState) -> bool:
        with self._lock:
            return self._current is state

    def snapshot(self) -> NavPhaseSnapshot:
        with self._lock:
            return NavPhaseSnapshot(
                self._current,
                self._previous,
                self._oneshot_completed,
                self._last_parameter_refresh_s,
            )

    def commit_transition(
        self,
        *,
        expected_previous: NavState,
        requested: NavState,
        effective: NavState,
        oneshot_completed: bool,
    ) -> None:
        """Commit one successfully applied transition as one state update."""
        with self._lock:
            if (
                self._previous is not expected_previous
                or self._current is not requested
            ):
                raise RuntimeError("navigation phase changed during transition")
            self._current = effective
            self._previous = effective
            self._oneshot_completed = (
                self._oneshot_completed or oneshot_completed
            )


@dataclass
class NavigationTaskState:
    """Mutable state for one navigation navigation_task."""

    peer_navigation: bool = False
    peer_approach_distance_m: float = 500.0
    orbit_radius_m: float = 0.0
    orbit_approach_alt_rel_m: Optional[float] = None
    peer_target_location: Optional[Location] = None
    navigation_target_location: Optional[Location] = None
    nav_mode_observed: bool = False
    guided_last_attempt_at: Optional[float] = None
    guided_request_started_at: Optional[float] = None
    terminal_navigation_active: bool = False
    terminal_nav_completed: bool = False


class NavigationFailureLatch:
    """Thread-safe one-shot failure signal, separate from nav-thread state."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._failed = False

    @property
    def failed(self) -> bool:
        with self._lock:
            return self._failed

    def mark_failed(self) -> None:
        with self._lock:
            self._failed = True

    def consume(self) -> bool:
        with self._lock:
            failed = self._failed
            self._failed = False
            return failed

    def reset(self) -> None:
        with self._lock:
            self._failed = False


@dataclass
class TerminalNavState:
    """Vision-nav record state for the current NAV episode."""

    confirmed_recorded: bool = False
    deferred_record_started_at: Optional[float] = None

    def mark_recorded(self) -> None:
        self.confirmed_recorded = True
        self.deferred_record_started_at = None


@dataclass
class GeoHoldState:
    """Geo-follow recovery state, isolated from pure-terminal commands."""

    active: bool = False
    target_location: Optional[Location] = None
    last_own_target_geo: Optional[Location] = None
    acquisition_log_bucket: Optional[tuple[str, ...]] = None


@dataclass
class ConfirmGateState:
    """Recognition and review timing state for one CONFIRM episode."""

    entered_at: Optional[float] = None
    zoom_had_target: bool = False
    zoom_seen_target: bool = False
    resets_used: int = 0
    loss_started_at: Optional[float] = None
    review_started_at: Optional[float] = None
    hold_active: bool = False


class ConfirmOverrideInbox:
    """Thread-safe one-shot task-id overrides from the network listener."""

    def __init__(self) -> None:
        self._task_ids: set[int] = set()
        self._lock = threading.RLock()

    def request(self, task_id: int) -> None:
        with self._lock:
            self._task_ids.add(task_id)

    def consume(self, task_id: int) -> bool:
        with self._lock:
            if task_id not in self._task_ids:
                return False
            self._task_ids.remove(task_id)
            return True

    def contains(self, task_id: int) -> bool:
        with self._lock:
            return task_id in self._task_ids


__all__ = [
    "ConfirmGateState",
    "ConfirmOverrideInbox",
    "NavigationTaskState",
    "GeoHoldState",
    "NavigationFailureLatch",
    "NavPhaseState",
    "NavPhaseSnapshot",
    "NavState",
    "TerminalNavState",
]
