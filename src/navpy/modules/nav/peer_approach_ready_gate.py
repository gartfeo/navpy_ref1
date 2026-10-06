"""Distance and altitude gate for a peer-assigned approach."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.nav_constants import ALT_HYST, PEER_APPROACH_MARGIN_M
from navpy.modules.nav.nav_state import NavigationTaskState


@dataclass(frozen=True)
class PeerApproachReadyPorts:
    selected_target: Callable[[], TaskAssignMsgData | None]
    current_absolute: Callable[[], Location | None]
    current_relative: Callable[[], Location | None]


class PeerApproachReadyGate:
    """Decide when distance and optional climb criteria are satisfied."""

    def __init__(
        self,
        ports: PeerApproachReadyPorts,
        navigation_task: NavigationTaskState,
        approach_kind: ApproachKind,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._approach_kind = approach_kind
        self._logger = logger

    def has_assignment(self) -> bool:
        return self._ports.selected_target() is not None

    def is_ready(self) -> bool:
        selected = self._ports.selected_target()
        if selected is None:
            return False
        location = selected.location
        target = Location(
            location.lat,
            location.lng,
            location.alt,
            is_absolute=True,
        )
        current = self._ports.current_absolute()
        if current is None:
            return False
        actual_distance = current.distance_to(target)
        approach_ready_distance = self._approach_ready_distance()
        near = actual_distance <= approach_ready_distance
        if near and self._navigation_task.orbit_approach_alt_rel_m is not None:
            relative = self._ports.current_relative()
            if (
                relative is not None
                and relative.alt
                < self._navigation_task.orbit_approach_alt_rel_m - ALT_HYST
            ):
                near = False
                self._logger.info(
                    f"PEER_APPROACH_WAIT: climbing alt={relative.alt:.0f}m < "
                    f"approach_alt={self._navigation_task.orbit_approach_alt_rel_m:.0f}m "
                    f"(dist={actual_distance:.0f}m)",
                    key="nav",
                )
        if near:
            self._logger.info(
                f"PEER_APPROACH_READY: dist={actual_distance:.0f}m "
                f"approach_ready_threshold={approach_ready_distance:.0f}m "
                f"approach_offset={self._navigation_task.peer_approach_distance_m:.0f}m "
                f"orbit_r={self._navigation_task.orbit_radius_m:.0f}m",
                key="nav",
            )
        return near

    def _approach_ready_distance(self) -> float:
        if (
            self._approach_kind == ApproachKind.ORBIT
            and self._navigation_task.orbit_radius_m > 0
        ):
            return self._navigation_task.orbit_radius_m + PEER_APPROACH_MARGIN_M
        return max(
            self._navigation_task.peer_approach_distance_m,
            self._navigation_task.orbit_radius_m,
        ) + PEER_APPROACH_MARGIN_M


__all__ = ["PeerApproachReadyGate", "PeerApproachReadyPorts"]
