"""Explicit construction of Navigation's focused collaborators."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional, cast

from navpy.args.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.logger.cache_logger import ILogger
from navpy.logger.navigation_logger import NavigationLogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.navigation_command_session import NavigationCommandSession
from navpy.modules.navigation.navigation_lifecycle import NavigationLifecycle
from navpy.modules.navigation.navigation_law_builders import compose_law_builders
from navpy.modules.navigation.navigation_mode import (
    NavigationModeBuilder,
    NavigationModeSelector,
    NavigationModeState,
    NavigationTerminalCapabilities,
    RuntimeBuildResult,
)
from navpy.modules.navigation.navigation_runtime import LegacyNavigationRuntime
from navpy.modules.navigation.navigation_source_dispatch import (
    BindSourceDispatch,
    NavigationSourceDispatchSlot,
)
from navpy.modules.navigation.navigation_terminal import TerminalNavigationService
from navpy.modules.navigation.navigation_vehicle_commands import NavigationVehicleCommands
from navpy.modules.navigation.legacy_destination_resolver import (
    LegacyNavigationState,
    LegacyDestinationResolver,
    navigation_target_location,
)
from navpy.modules.navigation.legacy_nav_adjustment import LegacyNavAdjustment
from navpy.modules.navigation.legacy_terminal_command import (
    LegacyTerminalCommand,
    LegacyTerminalCommandPorts,
)
from navpy.modules.navigation.mission_planner import MissionPlanner
from navpy.modules.navigation.nav.nav_law import NavLaw
from navpy.modules.navigation.nav.nav_law_factory import (
    NavigationLawLifecycle,
    get_nav_algorithm_spec,
)
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw
from navpy.modules.navigation.nav.vision_nav.runtime_composition import (
    TerminalVehicleActuator,
    TerminalVehicleDiagnosticReader,
    compose_terminal_runtime,
)
from navpy.modules.vehicle.pose_streams import (
    resolve_ardupilot_scheduler_rate_hz,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle


@dataclass(frozen=True)
class NavigationComposition:
    lifecycle: NavigationLifecycle
    mode_state: NavigationModeState
    terminal: TerminalNavigationService
    legacy_targets: LegacyDestinationResolver
    vehicle_commands: NavigationVehicleCommands
    bind_source_dispatch: BindSourceDispatch


def _identity_period(period_s: float) -> float:
    return period_s


def _legacy_runtime_factory(
    *,
    command_slot: NavigationCommandSlot,
    vehicle: IVehicle,
    geo_ref: GeoRefCalc,
    logger: ILogger,
    navigation_logger: NavigationLogger,
    target_resolver: LegacyDestinationResolver,
    legacy_state: LegacyNavigationState,
    adjustment: LegacyNavAdjustment,
) -> Callable[[NavigationLawLifecycle], RuntimeBuildResult]:
    def build(selected_law: NavigationLawLifecycle) -> RuntimeBuildResult:
        nav = cast(NavLaw, selected_law)
        command = LegacyTerminalCommand(LegacyTerminalCommandPorts(
            vehicle=vehicle,
            geo_ref=geo_ref,
            logger=logger,
            navigation_logger=navigation_logger,
            nav=nav,
            get_locked_target=legacy_state.locked_target,
            adjust_nav=adjustment.adjust,
            is_adjusted=legacy_state.is_adjusted,
            calc_nav_target=navigation_target_location,
        ))
        runtime = LegacyNavigationRuntime(
            command_slot=command_slot,
            target_resolver=target_resolver.resolve,
            command_executor=command.execute,
            reset_law=nav.reset,
        )
        return RuntimeBuildResult(runtime=runtime)

    return build


def _vision_runtime_factory(
    *,
    lock: threading.RLock,
    command_slot: NavigationCommandSlot,
    vehicle: IVehicle,
    geo_ref: GeoRefCalc,
    navigation_logger: NavigationLogger,
    wall_period_s: Callable[[float], float],
) -> Callable[[NavigationLawLifecycle], RuntimeBuildResult]:
    def build(selected_law: NavigationLawLifecycle) -> RuntimeBuildResult:
        law = cast(VisionNavLaw, selected_law)
        built = compose_terminal_runtime(
            lock=lock,
            slot=command_slot,
            sys_id=vehicle.target_system,
            actuator=TerminalVehicleActuator(vehicle.set_attitude),
            diagnostic_reader=TerminalVehicleDiagnosticReader(
                lambda: vehicle.attitude,
                lambda: vehicle.location(False),
            ),
            aircraft_roll_deg=lambda: (
                0.0 if vehicle.attitude is None else vehicle.attitude.roll
            ),
            law=law,
            aircraft_sequence=geo_ref.uas_seq,
            aircraft_degrees=geo_ref.degrees,
            navigation_logger=navigation_logger,
            wall_period_s=wall_period_s,
        )
        terminal = NavigationTerminalCapabilities(
            status=built.runtime,
            confirmation=built.confirmation,
        )
        return RuntimeBuildResult(runtime=built.runtime, terminal=terminal)

    return build


def compose_navigation(
    vehicle: IVehicle,
    geo_ref: GeoRefCalc,
    zc_util: Optional[ZcUtil],
    mission_planner: MissionPlanner,
    logger: ILogger,
    args: NavigationArgs,
    scheduler_cadence: Optional[SchedulerCadence] = None,
) -> NavigationComposition:
    """Build Navigation without passing the root object into collaborators."""
    command_lock = threading.RLock()
    command_event = threading.Event()
    source_dispatch = NavigationSourceDispatchSlot()
    command_slot = NavigationCommandSlot(command_lock, command_event)
    wall_period = (
        scheduler_cadence.wall_period_for_scheduler_period
        if scheduler_cadence is not None
        else _identity_period
    )
    set_process_command_cadence_active = (
        scheduler_cadence.set_command_cadence_active
        if scheduler_cadence is not None
        else lambda _active: None
    )
    scheduler_rate_hz = resolve_ardupilot_scheduler_rate_hz(vehicle)
    navigation_logger = NavigationLogger(
        vehicle.source_system,
        logger,
        cadence_interval=(wall_period if scheduler_cadence is not None else None),
    )
    legacy_state = LegacyNavigationState()
    target_resolver = LegacyDestinationResolver(vehicle, args, geo_ref, zc_util, legacy_state)
    adjustment = LegacyNavAdjustment(vehicle, mission_planner, legacy_state)
    legacy_factory = _legacy_runtime_factory(
        command_slot=command_slot,
        vehicle=vehicle,
        geo_ref=geo_ref,
        logger=logger,
        navigation_logger=navigation_logger,
        target_resolver=target_resolver,
        legacy_state=legacy_state,
        adjustment=adjustment,
    )
    vision_factory = _vision_runtime_factory(
        lock=command_lock,
        command_slot=command_slot,
        vehicle=vehicle,
        geo_ref=geo_ref,
        navigation_logger=navigation_logger,
        wall_period_s=wall_period,
    )

    mode_state = NavigationModeState()
    law_builders = compose_law_builders(vehicle, args)
    builders = {
        algorithm: NavigationModeBuilder(
            spec=get_nav_algorithm_spec(algorithm),
            build_law=law_builders[algorithm],
            runtime_factory=(
                vision_factory
                if algorithm is NavigationAlgorithm.VISION_NAV_PN
                else legacy_factory
            ),
        )
        for algorithm in NavigationAlgorithm
    }
    selector = NavigationModeSelector(args, builders, mode_state, logger)
    selector.refresh(force=True)
    lifecycle = NavigationLifecycle(
        mode_state,
        selector,
        legacy_state,
        navigation_logger,
        NavigationCommandSession(
            command_event=command_event,
            runtime_session=mode_state.runtime_session,
            wall_period_s=wall_period,
            logger=logger,
            scheduler_rate_hz=scheduler_rate_hz,
            source_dispatch=source_dispatch.dispatch_available,
            set_process_command_cadence_active=(
                set_process_command_cadence_active
            ),
            # The slot forwards to whatever the source bound, or to
            # nothing. Record-only either way.
            command_loop=source_dispatch,
        ),
    )
    return NavigationComposition(
        lifecycle=lifecycle,
        mode_state=mode_state,
        terminal=TerminalNavigationService(mode_state),
        legacy_targets=target_resolver,
        vehicle_commands=NavigationVehicleCommands(vehicle),
        bind_source_dispatch=source_dispatch.bind,
    )


__all__ = ["NavigationComposition", "compose_navigation"]
