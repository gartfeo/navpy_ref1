"""Delivery's part of each decision: the approach result, then the POI review.

Core runs its safety holds first (NavigationDecision) and then calls
DeliveryFlow.decide. Delivery also owns where search restarts and resumes.
"""

from __future__ import annotations

from typing import Callable, Optional

from navpy.core.flow.nav_state import NavPhaseState, NavState
from navpy.core.flow.nav_status import NavigationStatusReporter
from navpy.core.flow.navigation_decision import ApproachOutcome, NavStateDecision
from navpy.core.ports.flight_mode import FlightMode
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.products.delivery.poi_status_decision import ConfirmationStatusDecision


class DeliveryFlow:
    """Choose the next phase from core's approach result, else review the POI."""

    def __init__(
        self,
        phase: NavPhaseState,
        approach: NavStateDecision,
        review: ConfirmationStatusDecision,
        status: NavigationStatusReporter,
        clear_active_poi: Callable[[], None],
    ) -> None:
        self._phase = phase
        self._approach = approach
        self._review = review
        self._status = status
        self._clear_active_poi = clear_active_poi
        self._approach_ended: dict[ApproachOutcome, Callable[[], None]] = {
            ApproachOutcome.ABANDONED: self._abandoned,
            ApproachOutcome.PASSED: self._passed,
            ApproachOutcome.FAILED: self._failed,
        }

    def decide(self, mode: FlightMode) -> None:
        outcome = self._approach.advance(mode)
        if outcome is ApproachOutcome.NONE:
            self._review.dispatch()
            return
        ended = self._approach_ended.get(outcome)
        if ended is not None:
            ended()

    def _abandoned(self) -> None:
        self._phase.request(NavState.DETECT)

    def _passed(self) -> None:
        self._status.ignore(2, "RESET: PASSED POI")
        self._phase.request(NavState.RESET)

    def _failed(self) -> None:
        self._clear_active_poi()
        self._phase.request(NavState.RESET)


def restart_search(vehicle: IVehicle) -> None:
    """Restart the AUTO search mission at its first item."""
    vehicle.set_mode(FlightMode.AUTO)
    vehicle.restart_mission(1)


def interrupted_leg(next_waypoint: Optional[int]) -> Optional[int]:
    """Resume one item back, so the interrupted search leg is flown again."""
    if next_waypoint is None:
        return None
    return max(next_waypoint - 1, 0)


__all__ = ["DeliveryFlow", "interrupted_leg", "restart_search"]
