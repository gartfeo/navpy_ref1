"""Composition of the delivery product's calls into core."""

from __future__ import annotations

from collections.abc import Callable

from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavStateOwnership,
    PoiMissionOwnership,
)
from navpy.core.flow.nav_recovery_ports import RecoveryHold
from navpy.core.flow.navigation_decision import NavStateDecision
from navpy.core.flow.navigation_task_reset import NavigationTaskResetTransaction
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.skills.acquire.peer_navigation import PeerNavigationCoordinator
from navpy.core.skills.search.navigation_task_action import NavigationTaskAction
from navpy.core.skills.search.task_focus import TaskFocus
from navpy.core.swarm.peer_task_priority import PeerTaskPriority
from navpy.products.delivery.fallback_mission_navigation import (
    FallbackMissionNavigation,
)
from navpy.products.delivery.flow import DeliveryFlow
from navpy.products.delivery.home_dock import HomeDockRules
from navpy.products.delivery.mission_catalog import MissionCatalog
from navpy.products.delivery.nav_oneshot_completion import (
    OneShotCompletion,
    OneShotCompletionPorts,
)
from navpy.products.delivery.poi_status_decision import ConfirmationStatusDecision
from navpy.products.delivery.work_source import DeliveryWorkSource, HubReturnPorts


def compose_delivery_flow(
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    observation: DetectionReviewOwnership,
    approach: NavStateDecision,
    review: ConfirmationStatusDecision,
) -> DeliveryFlow:
    return DeliveryFlow(
        state.phase,
        approach,
        review,
        observation.status,
        lambda: poi.confirmation_manager.clear_active_poi(),
    )


def compose_sortie_finish(
    vehicle: IVehicle,
    reset: NavigationTaskResetTransaction,
    logger: ILogger,
    is_stopping: Callable[[], bool],
    hold: RecoveryHold,
    catalog: MissionCatalog,
) -> OneShotCompletion:
    return OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=vehicle.is_simulated_autopilot,
            disarm=vehicle.disarm,
            is_stopping=is_stopping,
            return_to_hub=lambda: _start_hub_return(hold, catalog),
            end_return=hold.end_sortie_return,
        ),
        reset,
        logger,
    )


def _start_hub_return(hold: RecoveryHold, catalog: MissionCatalog) -> bool:
    """A failed one-shot dock flies to the mission's DDH; without one there is nowhere to park."""
    if catalog.default_delivery_hub is None:
        return False
    hold.start_sortie_return()
    return True


def compose_home_dock_rules(
    focus: TaskFocus,
    poi: PoiMissionOwnership,
    hold: RecoveryHold,
) -> HomeDockRules:
    return HomeDockRules(
        focus,
        lambda: poi.mission.default_delivery_hub,
        lambda: poi.mission.default_delivery_hub_active,
        lambda: hold.sortie_returning,
    )


def compose_work_source(
    state: NavStateOwnership,
    peer_task: PeerTaskPriority,
    peer_navigation: PeerNavigationCoordinator,
    fallback_navigation: FallbackMissionNavigation,
    tasks: NavigationTaskAction,
    home_dock: HomeDockRules,
    logger: ILogger,
) -> DeliveryWorkSource:
    hold = state.recovery_hold
    return DeliveryWorkSource(
        state.navigation_task,
        peer_task,
        peer_navigation,
        fallback_navigation,
        tasks,
        HubReturnPorts(
            returning=lambda: hold.sortie_returning,
            is_home_dock=home_dock.is_home_dock,
            rearmed=hold.take_return_rearm,
            skipped=_skip_reporter(logger),
        ),
    )


def _skip_reporter(logger: ILogger) -> Callable[[DetectedObject], None]:
    """Once per dock: a dock seen on the return to the DDH that is not the home dock is not taken."""
    reported: set[int] = set()

    def report(poi: DetectedObject) -> None:
        poi_id = get_poi_task_id(poi)
        key = poi.identity.obj_id if poi_id is None else poi_id
        if key in reported:
            return
        reported.add(key)
        logger.info(f"DDH return: P{key} is not the home dock; not taken", key="nav")

    return report


__all__ = [
    "compose_delivery_flow",
    "compose_home_dock_rules",
    "compose_sortie_finish",
    "compose_work_source",
]
