from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger


@dataclass(frozen=True)
class SimSpeedupParameterPort:
    read: Callable[[], object]
    write: Callable[[float], bool]


class NavigationSpeedupState(Enum):
    """Observable ownership state for one navigation task speedup episode."""

    NO_LEASE = auto()
    VERIFIED_ACTIVE = auto()
    ROLLBACK_REQUIRED = auto()
    REQUEST_SUPPRESSED = auto()


class NavigationSpeedupLease:
    """Keep ``-gsu`` as a simulator resource, never a timestamp transform."""

    def __init__(
        self,
        *,
        parameter: SimSpeedupParameterPort,
        requested_speedup: Callable[[], float],
        sync_scheduler_cadence: Callable[[], None],
        logger: ILogger,
    ) -> None:
        self._parameter = parameter
        self._requested_speedup = requested_speedup
        self._sync_scheduler_cadence = sync_scheduler_cadence
        self._logger = logger
        self.original: Optional[float] = None
        self._state = NavigationSpeedupState.NO_LEASE

    @property
    def state(self) -> NavigationSpeedupState:
        return self._state

    def apply(self) -> bool:
        if self._state is NavigationSpeedupState.ROLLBACK_REQUIRED:
            return self._restore_baseline(
                next_state=NavigationSpeedupState.REQUEST_SUPPRESSED,
                log=False,
            )
        if self._state in {
            NavigationSpeedupState.VERIFIED_ACTIVE,
            NavigationSpeedupState.REQUEST_SUPPRESSED,
        }:
            return True
        requested = self._requested_speedup()
        if requested <= 0:
            return True
        if not self._capture_baseline():
            return False
        # PARAM_SET may take effect even when its PARAM_VALUE echo is lost.
        # Own the rollback before sending so every uncertain outcome is safe.
        self._state = NavigationSpeedupState.ROLLBACK_REQUIRED
        if self._parameter.write(requested) is not True:
            ready = self._restore_baseline(
                next_state=NavigationSpeedupState.REQUEST_SUPPRESSED,
                log=False,
            )
            self._logger.warning(
                f"SIM_SPEEDUP={requested} was not verified; "
                + (
                    "baseline restored and request suppressed for this navigation task"
                    if ready
                    else "baseline restore remains unverified; navigation task deferred"
                ),
                key="nav",
            )
            return ready
        self._state = NavigationSpeedupState.VERIFIED_ACTIVE
        self._sync_scheduler_cadence()
        self._logger.info(f"SIM_SPEEDUP={requested}", key="nav")
        return True

    def restore(self, *, log: bool) -> bool:
        if self._state is NavigationSpeedupState.NO_LEASE:
            return True
        if self._state is NavigationSpeedupState.REQUEST_SUPPRESSED:
            self._close_episode()
            return True
        return self._restore_baseline(
            next_state=NavigationSpeedupState.NO_LEASE,
            log=log,
        )

    def _restore_baseline(
        self,
        *,
        next_state: NavigationSpeedupState,
        log: bool,
    ) -> bool:
        initial = self.original
        if initial is None:
            return False
        if self._parameter.write(initial) is not True:
            self._state = NavigationSpeedupState.ROLLBACK_REQUIRED
            self._logger.warning(
                f"SIM_SPEEDUP={initial} restore was not verified; retry retained",
                key="nav",
            )
            return False
        self._state = next_state
        self._sync_scheduler_cadence()
        if log:
            self._logger.info(f"SIM_SPEEDUP={initial}", key="nav")
        if next_state is NavigationSpeedupState.NO_LEASE:
            self._close_episode()
        return True

    def _close_episode(self) -> None:
        self.original = None
        self._state = NavigationSpeedupState.NO_LEASE

    def _capture_baseline(self) -> bool:
        value = self._parameter.read()
        try:
            baseline = float(value)
        except (TypeError, ValueError):
            baseline = math.nan
        if not math.isfinite(baseline) or baseline <= 0:
            self._logger.warning(
                "SIM_SPEEDUP baseline unavailable; NAV speedup deferred",
                key="nav",
            )
            return False
        self.original = baseline
        return True


__all__ = [
    "NavigationSpeedupLease",
    "NavigationSpeedupState",
    "SimSpeedupParameterPort",
]
