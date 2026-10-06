"""Thin, explicit composition root for the navigation application."""

from __future__ import annotations

from typing import Optional

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.navigation import Navigation
from navpy.modules.navigation.peer_offset import calc_peer_approach_offset
from navpy.modules.nav.nav_application import NavApplication
from navpy.modules.nav.nav_composition_types import NavCapabilities
from navpy.modules.nav.nav_confirmation_workflow_composition import (
    compose_confirmation_workflows,
)
from navpy.modules.nav.nav_decision_composition import compose_decision_workflows
from navpy.modules.nav.nav_task_composition import (
    compose_navigation_task_workflows,
    compose_mission_navigation_ownership,
)
from navpy.modules.nav.nav_foundation_composition import (
    compose_detection_review_ownership,
    compose_state_ownership,
    compose_poi_mission_ownership,
    compose_vehicle_approach_ownership,
)
from navpy.modules.nav.nav_runtime_composition import compose_nav_application
from navpy.modules.nav.nav_final_approach_composition import (
    compose_final_approach_workflows,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


def create_nav_application(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    navigation: Navigation,
    args: NavArgs,
    logger: ILogger,
    approach_kind: ApproachKind = ApproachKind.OFFSET,
    vision_profile: Optional[dict] = None,
    scheduler_cadence: Optional[SchedulerCadence] = None,
) -> NavApplication:
    """Build the application from bounded semantic ownership stages."""

    profile = vision_profile or {}
    navigation_capabilities = NavCapabilities(
        final_approach=navigation.final_approach,
        legacy_pois=navigation.legacy_pois,
        vehicle_commands=navigation.vehicle_commands,
        init=navigation.init,
        reset=navigation.reset,
        pause=navigation.pause_final_approach,
        nav=navigation.nav,
        bind_source_dispatch=navigation.bind_final_approach_source_dispatch,
        algorithm_info=lambda: navigation.algorithm_info,
    )
    state = compose_state_ownership(scheduler_cadence)
    poi = compose_poi_mission_ownership(
        vehicle,
        detection,
        args,
        logger,
        state,
    )
    approach = compose_vehicle_approach_ownership(
        vehicle,
        detection,
        args,
        logger,
        profile,
        approach_kind,
        state,
        navigation_capabilities,
        lambda *pos, **kw: calc_peer_approach_offset(*pos, **kw),
    )
    observation = compose_detection_review_ownership(
        vehicle,
        detection,
        args,
        logger,
        profile,
        state,
        poi,
        navigation_capabilities,
    )
    mission_navigation = compose_mission_navigation_ownership(
        vehicle,
        detection,
        args,
        logger,
        approach_kind,
        state,
        poi,
        approach,
        navigation_capabilities,
    )
    navigation_workflows = compose_navigation_task_workflows(
        vehicle,
        detection,
        logger,
        profile,
        approach_kind,
        state,
        poi,
        approach,
        observation,
        navigation_capabilities,
        mission_navigation,
    )
    final_approach = compose_final_approach_workflows(
        vehicle,
        detection,
        args,
        logger,
        approach_kind,
        state,
        poi,
        approach,
        observation,
        navigation_workflows,
        navigation_capabilities,
    )
    confirmation = compose_confirmation_workflows(
        detection,
        logger,
        state,
        poi,
        approach,
        observation,
        final_approach.admission,
        final_approach.reset,
        navigation_capabilities,
    )
    decision = compose_decision_workflows(
        vehicle,
        detection,
        args,
        logger,
        state,
        poi,
        approach,
        observation,
        navigation_workflows,
        confirmation,
        navigation_capabilities,
    )
    return compose_nav_application(
        vehicle,
        detection,
        args,
        logger,
        state,
        poi,
        approach,
        observation,
        navigation_workflows,
        confirmation,
        decision,
        navigation_capabilities,
        final_approach.nav,
    )


__all__ = ["create_nav_application"]
