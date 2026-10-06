"""Altitude recovery and return to the search mission."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.nav.nav_constants import ALT_HYST
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.nav_status import NavigationStatusReporter
from navpy.modules.vehicle.flight_mode import FlightMode


@dataclass(frozen=True)
class RecoveryPorts:
    final_approach_active: Callable[[], bool]
    detector_stop: Callable[[], None]
    current_relative: Callable[[], Optional[Location]]
    pause_navigation: Callable[[], None]
    set_mode: Callable[[FlightMode], None]
    restart_mission: Callable[[int], None]
    set_attitude: Callable[[float, float, float, float], None]
    max_pitch_deg: Callable[[], float]


class RecoveryAction:
    """Advance and command the non-blocking altitude recovery phase."""

    def __init__(
        self,
        ports: RecoveryPorts,
        phase: NavPhaseState,
        args: NavArgs,
        status: NavigationStatusReporter,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._phase = phase
        self._args = args
        self._status = status
        self._logger = logger
        self._altitude_available: Optional[bool] = None

    def advance(self) -> bool:
        if self._ports.final_approach_active():
            return self._advance_final_approach()
        if self._phase.current == NavState.RESET:
            self._ports.detector_stop()
            altitude = self._altitude()
            if altitude is None:
                return True
            self._phase.request(
                NavState.RECOVERY
                if altitude < self._args.min_alt - ALT_HYST
                else NavState.DETECT
            )
            return True
        altitude = self._altitude()
        if altitude is None:
            return True
        if altitude < self._args.min_alt:
            if self._phase.current != NavState.RECOVERY:
                self._status.ignore(2, f"RECOVERY: MIN ALT {altitude}")
            self._phase.request(NavState.RECOVERY)
            return True
        if self._phase.current == NavState.RECOVERY:
            if altitude >= self._args.min_alt + ALT_HYST:
                self._phase.request(
                    NavState.ONHOLD
                    if self._args.is_oneshot and self._phase.oneshot_completed
                    else NavState.DETECT
                )
            return True
        return self._hold_completed_oneshot()

    def enter(self) -> None:
        self._ports.pause_navigation()
        self._logger.info(
            "RECOVERY: climbing to min altitude",
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def restart_auto_mission(self) -> None:
        self._ports.set_mode(FlightMode.AUTO)
        self._ports.restart_mission(1)

    def act(self) -> None:
        self._ports.pause_navigation()
        if self._altitude_available is False:
            return
        pitch = math.radians(2 * self._ports.max_pitch_deg())
        self._ports.set_attitude(0, pitch, 0, 1.0)

    def _advance_final_approach(self) -> bool:
        if self._phase.current == NavState.RESET:
            self._ports.detector_stop()
            self._phase.request(NavState.DETECT)
            return True
        if self._phase.current == NavState.RECOVERY:
            self._phase.request(NavState.DETECT)
            return True
        return self._hold_completed_oneshot()

    def _hold_completed_oneshot(self) -> bool:
        if not (self._args.is_oneshot and self._phase.oneshot_completed):
            return False
        self._status.ignore(4, "ONHOLD: ONE-SHOT COMPLETE")
        self._phase.request(NavState.ONHOLD)
        return True

    def _altitude(self) -> Optional[float]:
        location = self._ports.current_relative()
        self._altitude_available = location is not None
        return location.alt if location is not None else None


__all__ = ["RecoveryAction", "RecoveryPorts"]
