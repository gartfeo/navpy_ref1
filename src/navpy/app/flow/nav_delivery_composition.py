"""Composition of the delivery product's calls into core."""

from __future__ import annotations

from collections.abc import Callable

from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavStateOwnership,
    PoiMissionOwnership,
)
from navpy.core.flow.navigation_decision import NavStateDecision
from navpy.core.flow.navigation_task_reset import NavigationTaskResetTransaction
from navpy.core.kernel.logging.cache_logger import ILogger
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
from navpy.products.delivery.nav_oneshot_completion import (
    OneShotCompletion,
    OneShotCompletionPorts,
)
from navpy.products.delivery.poi_status_decision import ConfirmationStatusDecision
from navpy.products.delivery.work_source import DeliveryWorkSource


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
) -> OneShotCompletion:
    return OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=vehicle.is_simulated_autopilot,
            disarm=vehicle.disarm,
            is_stopping=is_stopping,
        ),
        reset,
        logger,
    )


def compose_home_dock_rules(
    focus: TaskFocus,
    poi: PoiMissionOwnership,
) -> HomeDockRules:
    return HomeDockRules(
        focus,
        lambda: poi.mission.default_delivery_hub,
        lambda: poi.mission.default_delivery_hub_active,
    )


def compose_work_source(
    state: NavStateOwnership,
    peer_task: PeerTaskPriority,
    peer_navigation: PeerNavigationCoordinator,
    fallback_navigation: FallbackMissionNavigation,
    tasks: NavigationTaskAction,
) -> DeliveryWorkSource:
    return DeliveryWorkSource(
        state.navigation_task,
        peer_task,
        peer_navigation,
        fallback_navigation,
        tasks,
    )


__all__ = [
    "compose_delivery_flow",
    "compose_home_dock_rules",
    "compose_sortie_finish",
    "compose_work_source",
]
