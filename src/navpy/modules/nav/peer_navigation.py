"""Thin peer-navigation facade over approach and navigation task owners."""

from __future__ import annotations

from navpy.modules.nav.peer_approach_start import (
    PeerApproachStarter,
    PeerAssignmentSetup,
)
from navpy.modules.nav.peer_approach_ready_gate import PeerApproachReadyGate


class PeerNavigationCoordinator:
    """Coordinate assignment setup without owning planning or gate policy."""

    def __init__(
        self,
        assignment: PeerAssignmentSetup,
        approach: PeerApproachStarter,
        approach_ready_gate: PeerApproachReadyGate,
    ) -> None:
        self._assignment = assignment
        self._approach = approach
        self._approach_ready_gate = approach_ready_gate

    def setup(self) -> None:
        prepared = self._assignment.prepare()
        if prepared is not None:
            selected, navigation_location = prepared
            self._approach.start(selected, navigation_location)

    def has_assignment(self) -> bool:
        return self._approach_ready_gate.has_assignment()

    def near_target(self) -> bool:
        return self._approach_ready_gate.is_ready()


__all__ = ["PeerNavigationCoordinator"]
