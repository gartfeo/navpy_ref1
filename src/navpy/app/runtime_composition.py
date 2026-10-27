"""Application composition root for a single NavPy runtime."""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.exception_groups import BaseExceptionGroup, ExceptionGroup

from argparse import Namespace
from collections.abc import Callable
from dataclasses import dataclass
from typing import NoReturn

import cv2

from navpy.app.network_args import NetworkArgs
from navpy.app.conn_args import ConnArgs
from navpy.core.kernel.config.navigation_args import NavigationArgs
from navpy.sim.world.navigation_poi_args import NavigationPoiArgs
from navpy.core.kernel.config.logger_args import LoggerArgs
from navpy.core.kernel.config.nav_args import NavArgs
from navpy.app.navpy_argparse import make_parser
from navpy.app.vision_args import VisionArgs
from navpy.core.kernel.logging.logger_api import ILogger
from navpy.core.kernel.logging.logger_factory import initialize_logger
from navpy.core.link.status_logger_network import GroundStationLoggerNetwork
from navpy.core.ports.network_abc import NetworkAbc
from navpy.app.network_factory import create_network
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.app.approach.navigation import Navigation
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.core.skills.geo_approach.mission_planner import MissionPlanner, MissionPlannerArgs
from navpy.app.flow.nav_controller import NavController
from navpy.core.approach.dock_contact_episode import with_dock_evidence_cut
from navpy.app.vehicle_factory import create_vehicle, open_vehicle_bus
from navpy.app.wall_scheduler_cadence import speedup_scheduler_cadence
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.pose_streams import (
    resolve_ardupilot_scheduler_rate_hz,
)
from navpy.app.perception.vision_controller import VisionController
from navpy.sim.detection.camera_frame_ports import CameraFrameFactoryPort
from navpy.app.runtime_ports import RuntimePorts
from navpy.app.runtime_stop import (
    StoppedBeforeStart,
    raise_if_stopped,
    stop_hooks,
)
from navpy.app.runtime_resources import (
    RuntimeResourceStack,
    RuntimeSession,
    VehicleCloseGate,
)
from navpy.app.runtime_ready import emit_runtime_ready
from navpy.app.terminal_handoff import (
    TerminalHandoff, build_terminal_handoff, fenced_opener, siyi_fence, terminal_observer,
)
from navpy.core.kernel.config.arg_helper import (
    collect_non_default_args,
    collect_param_overrides,
    format_kv,
)
from navpy.core.perception.geo.zc_util_instance import ZcUtilInstance


@dataclass(frozen=True)
class RuntimeFoundation:
    logger: ILogger
    vehicle: IVehicle
    network: NetworkAbc | None
    navigation_args: NavigationArgs
    zc_util: ZcUtil | None
    vision_args: VisionArgs
    scheduler_cadence: SchedulerCadence | None
    vehicle_close_gate: VehicleCloseGate
    terminal_handoff: TerminalHandoff | None = None
    camera_frames: CameraFrameFactoryPort | None = None


def _raise_composition_failure(
    primary: BaseException,
    cleanup: BaseException,
) -> NoReturn:
    errors = [primary, cleanup]
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup("runtime construction and rollback failed", errors)
    raise BaseExceptionGroup("runtime construction and rollback failed", errors)


def _log_configuration(
    args: Namespace,
    vehicle: IVehicle,
    logger: ILogger,
) -> None:
    defaults = make_parser(description="NavPy Control System").parse_args(
        ["-ss", "0"]
    )
    non_defaults = collect_non_default_args(args, defaults)
    logger.info(f"Args (non-default): {format_kv(non_defaults)}")
    overrides = collect_param_overrides(
        vehicle.get_param_or_default,
        {**NavigationArgs.PARAMS, **NavArgs.PARAMS, **NavigationPoiArgs.PARAMS},
    )
    if overrides:
        logger.info(f"AAS params (non-default): {format_kv(overrides)}")


def _build_foundation(
    args: Namespace,
    resources: RuntimeResourceStack,
    ports: RuntimePorts = RuntimePorts(),
    stop_requested: Callable[[], bool] | None = None,
) -> RuntimeFoundation:
    connection_args = ConnArgs(args)
    logger_args = LoggerArgs(args)
    status_logger = GroundStationLoggerNetwork.create(
        connection_args.source_system,
        logger_args,
    )
    logger = initialize_logger(
        logger_args,
        connection_args.source_system,
        status_logger,
    )
    resources.own("logger", logger.close)
    ports.observe_logger(logger)
    resources.own_independent("OpenCV windows", cv2.destroyAllWindows)

    terminal_handoff = build_terminal_handoff(args)
    bus = open_vehicle_bus(connection_args, fenced_opener(ports.open_bus, terminal_handoff))
    # After the stop the vehicle's reads and mission transfers end at once,
    # so startup reaches a stop check without waiting on the link.
    vehicle = create_vehicle(connection_args, logger, bus=bus, stop_requested=stop_requested)
    close_gate = VehicleCloseGate(vehicle.close)
    # Quiescence: the MAVLink reader dispatches into the logger, so the
    # logger stays open until the reader has really stopped.
    resources.own_quiescence("vehicle", close_gate.close_vehicle)
    raise_if_stopped(stop_requested)
    status_logger.set_vehicle(vehicle)
    _log_configuration(args, vehicle, logger)

    network = create_network(
        NetworkArgs(args),
        vehicle.source_system,
        logger,
        vehicle,
    )
    if network is not None:
        resources.own("network", network.close)
    status_logger.set_network(network)

    navigation_args = NavigationArgs(args, vehicle, logger)
    if navigation_args.use_terrain:
        logger.verbose("Waiting for terrain data...")
        zc_util = ZcUtilInstance.get(max_distance=5000, degrees=True)
    else:
        zc_util = None

    vision_args = VisionArgs.from_args(args)
    scheduler_cadence = ports.scheduler_cadence(vehicle, bus)
    resources.own("simulation cadence", scheduler_cadence.close)
    return RuntimeFoundation(
        logger=logger,
        vehicle=vehicle,
        network=network,
        navigation_args=navigation_args,
        zc_util=zc_util,
        vision_args=vision_args,
        scheduler_cadence=scheduler_cadence,
        vehicle_close_gate=close_gate,
        terminal_handoff=terminal_handoff,
        camera_frames=ports.camera_frames,
    )


def _build_session(
    args: Namespace,
    foundation: RuntimeFoundation,
    resources: RuntimeResourceStack,
    hooks: dict[str, Callable[..., object]],
) -> RuntimeSession:
    vision = VisionController(
        foundation.vehicle,
        args,
        foundation.vision_args,
        foundation.logger,
        foundation.zc_util,
        scheduler_cadence=foundation.scheduler_cadence,
        siyi_fence=siyi_fence(foundation.terminal_handoff),
        camera_frames=foundation.camera_frames,
    )
    resources.own_quiescence(
        "vision",
        lambda: foundation.vehicle_close_gate.stop_vision(
            vision.stop,
            lambda: vision.is_quiescent,
        ),
    )
    planner = MissionPlanner(foundation.logger, MissionPlannerArgs(args))
    navigation = Navigation(
        foundation.vehicle,
        vision.geo_ref,
        foundation.zc_util,
        planner,
        foundation.logger,
        foundation.navigation_args,
        scheduler_cadence=foundation.scheduler_cadence,
        on_terminal=with_dock_evidence_cut(
            vision.dock_contact,
            terminal_observer(foundation.terminal_handoff),
        ),
    )
    resources.own_quiescence("navigation", navigation.stop)
    # Values read after the stop are defaults: never start on them.
    raise_if_stopped(hooks.get("stop_requested"))
    navigation.start()
    nav_args = NavArgs(args, foundation.vehicle)
    controller = NavController(
        foundation.vehicle,
        vision.coordination,
        navigation,
        nav_args,
        foundation.logger,
        approach_kind=vision.approach_kind,
        vision_profile=vision.profile,
        scheduler_cadence=foundation.scheduler_cadence,
        dock_contact=vision.dock_contact,
    )
    resources.own_quiescence("controller", controller.stop)
    controller.set_network(foundation.network)
    scheduler_rate_hz = resolve_ardupilot_scheduler_rate_hz(
        foundation.vehicle,
    )

    def check_runtime_health() -> None:
        vision.raise_if_failed()
        navigation.raise_if_failed()
        controller.raise_if_failed()

    def signal_runtime_ready() -> None:
        emit_runtime_ready({
            "mission_items": int(foundation.vehicle.mission_items_count),
            "targ_wps": int(foundation.vehicle.get_param_or_default(
                "AAS_TARG_WPS",
                NavigationPoiArgs.PARAMS["AAS_TARG_WPS"],
            )),
            "nav_last_wp": int(foundation.vehicle.get_param_or_default(
                "AAS_NAV_LAST_WP",
                NavArgs.PARAMS["AAS_NAV_LAST_WP"],
            )),
            "nav_min_wp_seq": int(nav_args.min_wp),
            "scheduler_rate_hz": scheduler_rate_hz,
        })

    return RuntimeSession(
        resources=resources,
        start_vision=vision.start,
        start_controller=lambda: controller.start(scheduler_rate_hz),
        health_check=check_runtime_health,
        ui_step=vision.ui_step,
        sleep=rt.sleep,
        ready_signal=signal_runtime_ready,
        **hooks,
    )


def compose_runtime(args: Namespace, ports: RuntimePorts = RuntimePorts()) -> RuntimeSession:
    if getattr(args, "stop_on_lifecycle", False):
        if getattr(args, "stop_on_stdin", False):
            raise ValueError("--stop-on-lifecycle and --stop-on-stdin are exclusive")
        if ports.lifecycle_stop is None:
            raise ValueError("--stop-on-lifecycle needs a lockstep participant's lifecycle stop")
    resources = RuntimeResourceStack()
    try:
        # Before the vehicle build, so a stop during startup is seen there.
        hooks = stop_hooks(args, ports.lifecycle_stop)
        foundation = _build_foundation(args, resources, ports, hooks.get("stop_requested"))
        return _build_session(args, foundation, resources, hooks)
    except StoppedBeforeStart:
        return RuntimeSession.stopped(resources, **hooks)
    except BaseException as primary:
        try:
            resources.close()
        except BaseException as cleanup:
            _raise_composition_failure(primary, cleanup)
        raise


__all__ = ["RuntimeFoundation", "compose_runtime", "speedup_scheduler_cadence"]
