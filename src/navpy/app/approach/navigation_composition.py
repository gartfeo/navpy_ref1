"""Explicit construction of Navigation's focused collaborators."""

from __future__ import annotations

from dataclasses import dataclass
from contextlib import nullcontext
from typing import Callable, ContextManager, Optional, cast

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.approach.logs.navigation_logger import NavigationLogger
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.core.command.navigation_command_slot import NavigationCommandSlot
from navpy.core.approach.navigation_command_session import NavigationCommandSession
from navpy.core.approach.navigation_lifecycle import NavigationLifecycle
from navpy.app.approach.navigation_law_builders import compose_law_builders
from navpy.core.approach.navigation_mode import (
    NavigationModeBuilder,
    NavigationModeSelector,
    NavigationModeState,
    NavigationGuidanceCapabilities,
    RuntimeBuildResult,
)
from navpy.core.approach.geo.legacy_navigation_runtime import LegacyNavigationRuntime
from navpy.core.approach.navigation_source_dispatch import (
    BindSourceDispatch,
    NavigationSourceDispatchSlot,
)
from navpy.core.approach.navigation_dock_outcome import GuidanceDockOutcomeService
from navpy.core.approach.navigation_guidance import GuidanceNavigationService
from navpy.core.command.navigation_vehicle_commands import NavigationVehicleCommands
from navpy.core.approach.geo.legacy_destination_resolver import (
    LegacyNavigationState,
    LegacyDestinationResolver,
    navigation_poi_location,
)
from navpy.core.skills.geo_approach.legacy_nav_adjustment import LegacyNavAdjustment
from navpy.core.approach.geo.legacy_guidance_command import (
    LegacyGuidanceCommand,
    LegacyGuidanceCommandPorts,
)
from navpy.core.skills.geo_approach.mission_planner import MissionPlanner
from navpy.core.ports.nav_law import NavLaw
from navpy.core.approach.nav_law_factory import (
    NavigationLawLifecycle,
    get_nav_algorithm_spec,
)
from navpy.plugins.laws.vision_pn.law import VisionNavLaw
from navpy.core.approach.vision_nav.episode_latch import TerminalObserver
from navpy.core.approach.vision_nav.runtime_composition import (
    GuidanceVehicleActuator,
    GuidanceVehicleDiagnosticReader,
    compose_guidance_runtime,
)
from navpy.plugins.vehicle.ardupilot.pose_streams import (
    resolve_ardupilot_scheduler_rate_hz,
)
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.app.approach.law_airspeed import LawAirspeed


@dataclass(frozen=True)
class NavigationComposition:
    lifecycle: NavigationLifecycle
    mode_state: NavigationModeState
    guidance: GuidanceNavigationService
    dock_outcome: GuidanceDockOutcomeService
    legacy_pois: LegacyDestinationResolver
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
    poi_resolver: LegacyDestinationResolver,
    legacy_state: LegacyNavigationState,
    adjustment: LegacyNavAdjustment,
) -> Callable[[NavigationLawLifecycle], RuntimeBuildResult]:
    def build(selected_law: NavigationLawLifecycle) -> RuntimeBuildResult:
        nav = cast(NavLaw, selected_law)
        command = LegacyGuidanceCommand(LegacyGuidanceCommandPorts(
            vehicle=vehicle,
            geo_ref=geo_ref,
            logger=logger,
            navigation_logger=navigation_logger,
            nav=nav,
            get_locked_poi=legacy_state.locked_poi,
            adjust_nav=adjustment.adjust,
            is_adjusted=legacy_state.is_adjusted,
            calc_nav_poi=navigation_poi_location,
        ))
        runtime = LegacyNavigationRuntime(
            command_slot=command_slot,
            poi_resolver=poi_resolver.resolve,
            command_executor=command.execute,
            reset_law=nav.reset,
        )
        return RuntimeBuildResult(runtime=runtime)

    return build


def _vision_runtime_factory(
    *,
    lock: rt.RLock,
    command_slot: NavigationCommandSlot,
    vehicle: IVehicle,
    geo_ref: GeoRefCalc,
    navigation_logger: NavigationLogger,
    logger: ILogger,
    wall_period_s: Callable[[float], float],
    on_terminal: Optional[TerminalObserver],
    law_airspeed: Optional[LawAirspeed] = None,
) -> Callable[[NavigationLawLifecycle], RuntimeBuildResult]:
    def build(selected_law: NavigationLawLifecycle) -> RuntimeBuildResult:
        law = cast(VisionNavLaw, selected_law)
        actuator = GuidanceVehicleActuator(vehicle.set_attitude)
        # HITL step 20: no write without an admitted, fresh airspeed source.
        guarded = actuator if law_airspeed is None else law_airspeed.guard(actuator)
        built = compose_guidance_runtime(
            lock=lock,
            slot=command_slot,
            sys_id=vehicle.target_system,
            actuator=guarded,
            diagnostic_reader=GuidanceVehicleDiagnosticReader(
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
            on_terminal=on_terminal,
            report_stale_hold=lambda check: logger.warning(
                f"NAV hold stale: {check.describe()}",
                key="nav",
            ),
            report_stale_command=lambda check: logger.warning(
                f"NAV command stale: {check.describe()}",
                key="nav",
            ),
        )
        guidance = NavigationGuidanceCapabilities(
            status=built.runtime.session,
            confirmation=built.confirmation,
            dock_outcome=built.dock_outcome,
        )
        return RuntimeBuildResult(runtime=built.runtime, guidance=guidance)

    return build


def compose_navigation(
    vehicle: IVehicle,
    geo_ref: GeoRefCalc,
    zc_util: Optional[ZcUtil],
    mission_planner: MissionPlanner,
    logger: ILogger,
    args: NavigationArgs,
    scheduler_cadence: Optional[SchedulerCadence] = None,
    on_terminal: Optional[TerminalObserver] = None,
    law_airspeed: Optional[LawAirspeed] = None,
    command_formation: Callable[[], ContextManager[None]] = nullcontext,
    command_pass: Callable[[], ContextManager[None]] = nullcontext,
) -> NavigationComposition:
    """Build Navigation without passing the root object into collaborators."""
    command_lock = rt.RLock()
    command_event = rt.Event()
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
    poi_resolver = LegacyDestinationResolver(vehicle, args, geo_ref, zc_util, legacy_state)
    adjustment = LegacyNavAdjustment(vehicle, mission_planner, legacy_state)
    legacy_factory = _legacy_runtime_factory(
        command_slot=command_slot,
        vehicle=vehicle,
        geo_ref=geo_ref,
        logger=logger,
        navigation_logger=navigation_logger,
        poi_resolver=poi_resolver,
        legacy_state=legacy_state,
        adjustment=adjustment,
    )
    vision_factory = _vision_runtime_factory(
        lock=command_lock,
        command_slot=command_slot,
        vehicle=vehicle,
        geo_ref=geo_ref,
        navigation_logger=navigation_logger,
        logger=logger,
        wall_period_s=wall_period,
        on_terminal=on_terminal,
        law_airspeed=law_airspeed,
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
            command_formation=command_formation,
            command_pass=command_pass,
        ),
    )
    return NavigationComposition(
        lifecycle=lifecycle,
        mode_state=mode_state,
        guidance=GuidanceNavigationService(mode_state),
        dock_outcome=GuidanceDockOutcomeService(mode_state),
        legacy_pois=poi_resolver,
        vehicle_commands=NavigationVehicleCommands(vehicle),
        bind_source_dispatch=source_dispatch.bind,
    )


__all__ = ["NavigationComposition", "compose_navigation"]
