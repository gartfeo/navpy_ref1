"""Application composition root for a single NavPy runtime."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import time
from argparse import Namespace
from dataclasses import dataclass
from typing import NoReturn

import cv2

from navpy.args.conn.network_args import NetworkArgs
from navpy.args.conn_args import ConnArgs
from navpy.args.navigation_args import NavigationArgs
from navpy.args.navigation_target_args import NavigationTargetArgs
from navpy.args.logger_args import LoggerArgs
from navpy.args.nav_args import NavArgs
from navpy.args.navpy_argparse import make_parser
from navpy.args.vision_args import VisionArgs
from navpy.logger.logger_api import ILogger
from navpy.logger.logger_factory import initialize_logger
from navpy.logger.status_logger_network import GroundStationLoggerNetwork
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.comm.network_factory import create_network
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.navigation import Navigation
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.navigation.mission_planner import MissionPlanner, MissionPlannerArgs
from navpy.modules.nav.nav_controller import NavController
from navpy.modules.vehicle.vehicle_factory import create_vehicle
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vehicle.pose_streams import (
    resolve_ardupilot_scheduler_rate_hz,
)
from navpy.modules.vision.vision_controller import VisionController
from navpy.runtime_resources import (
    RuntimeResourceStack,
    RuntimeSession,
    VehicleCloseGate,
)
from navpy.runtime_ready import emit_runtime_ready
from navpy.utils.arg_helper import (
    collect_non_default_args,
    collect_param_overrides,
    format_kv,
)
from navpy.utils.zc_util_instance import ZcUtilInstance


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
        NavigationArgs.PARAMS,
    )
    if overrides:
        logger.info(f"AAS params (non-default): {format_kv(overrides)}")


def _build_foundation(
    args: Namespace,
    resources: RuntimeResourceStack,
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
    resources.own_independent("OpenCV windows", cv2.destroyAllWindows)

    vehicle = create_vehicle(connection_args, logger)
    close_gate = VehicleCloseGate(vehicle.close)
    resources.own("vehicle", close_gate.close_vehicle)
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
    scheduler_cadence = SchedulerCadence(vehicle.sim_speedup)
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
    )


def _build_session(
    args: Namespace,
    foundation: RuntimeFoundation,
    resources: RuntimeResourceStack,
) -> RuntimeSession:
    vision = VisionController(
        foundation.vehicle,
        args,
        foundation.vision_args,
        foundation.logger,
        foundation.zc_util,
        scheduler_cadence=foundation.scheduler_cadence,
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
    )
    resources.own_quiescence("navigation", navigation.stop)
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
                NavigationTargetArgs.PARAMS["AAS_TARG_WPS"],
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
        sleep=time.sleep,
        ready_signal=signal_runtime_ready,
    )


def compose_runtime(args: Namespace) -> RuntimeSession:
    resources = RuntimeResourceStack()
    try:
        foundation = _build_foundation(args, resources)
        return _build_session(args, foundation, resources)
    except BaseException as primary:
        try:
            resources.close()
        except BaseException as cleanup:
            _raise_composition_failure(primary, cleanup)
        raise


__all__ = ["RuntimeFoundation", "compose_runtime"]
