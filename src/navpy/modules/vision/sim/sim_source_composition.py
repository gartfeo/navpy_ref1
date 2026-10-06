"""Compose simulator pose-source and frame-coordination services."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from navpy.modules.vehicle.pose_streams import (
    pose_frame_association_max_skew_s,
    request_pose_streams,
    request_simulator_truth_pose_stream,
    resolve_pose_stream_rate_hz,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
)
from navpy.modules.vision.sim.frame_generation_gate import FrameGenerationGate
from navpy.modules.vision.sim.frame_outcome_recorder import FrameOutcomeRecorder
from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSource
from navpy.modules.vision.sim.pose_associator import PoseAssociator
from navpy.modules.vision.sim.pose_frame_clock import PoseFrameClock
from navpy.modules.vision.sim.pose_inbox import PoseInbox
from navpy.modules.vision.sim.sim_detector_config import (
    SimDetectorDependencies,
    SimDetectorOptions,
)
from navpy.modules.vision.sim.sim_polling_pose_reader import SimPollingPoseReader
from navpy.modules.vision.sim.sim_source_activation import SimSourceActivation
from navpy.modules.vision.sim.source_frame_coordinator import SourceFrameCoordinator


@dataclass(frozen=True)
class SimSourceGraph:
    coordinator: SourceFrameCoordinator
    store: DetectionPublicationStore
    clock: PoseFrameClock
    pose_source: IdealPoseSource
    activation: SimSourceActivation
    record_outcome: FrameOutcomeRecorder


class SourceTimestampClock:
    def __init__(self, pose_source: IdealPoseSource) -> None:
        self._pose_source = pose_source

    def now(self) -> float:
        value = self._pose_source.source_now_s
        return float(value) if value is not None else time.time()


def build_source_graph(
    dependencies: SimDetectorDependencies,
    options: SimDetectorOptions,
) -> SimSourceGraph:
    vehicle = dependencies.vehicle
    stream_rate_hz = source_stream_rate_hz(vehicle)
    queue_capacity = max(8, int(math.ceil(stream_rate_hz * 2.0)))
    association_period_budget_s = pose_frame_association_max_skew_s(
        stream_rate_hz
    )
    record_outcome = FrameOutcomeRecorder(_target_system(vehicle))
    store = DetectionPublicationStore(
        source_driven=options.ideal_360,
        capacity=queue_capacity,
    )
    coordinator = _build_coordinator(
        dependencies,
        store,
        record_outcome,
        queue_capacity,
    )
    clock = PoseFrameClock(
        reorder_tolerance_s=association_period_budget_s,
        record_outcome=record_outcome,
    )
    pose_source = _build_pose_source(
        dependencies,
        coordinator,
        clock,
        association_period_budget_s,
    )
    activation = _build_activation(
        dependencies,
        stream_rate_hz,
        options.ideal_360,
    )
    return SimSourceGraph(
        coordinator,
        store,
        clock,
        pose_source,
        activation,
        record_outcome,
    )


def source_stream_rate_hz(
    vehicle: IVehicle,
) -> float:
    """Request raw pose samples at the achievable ArduPilot source rate.

    The ordinary simulator has one internal fixed polling cadence. A
    source-driven ideal sensor instead renders each admitted ArduPilot pose;
    throttling that stream creates wall-time jitter at high SITL speedups.
    """
    return resolve_pose_stream_rate_hz(vehicle)


def build_polling_reader(vehicle: IVehicle) -> SimPollingPoseReader:
    return SimPollingPoseReader(
        location=lambda: vehicle.location(False),
        attitude=lambda: vehicle.attitude,
        attitude_sample=lambda: vehicle.attitude_sample,
        air_speed=lambda: vehicle.air_speed,
    )


def _build_coordinator(
    dependencies: SimDetectorDependencies,
    store: DetectionPublicationStore,
    record_outcome: FrameOutcomeRecorder,
    queue_capacity: int,
) -> SourceFrameCoordinator:
    return SourceFrameCoordinator(
        gate=FrameGenerationGate(),
        inbox=PoseInbox(queue_capacity),
        publications=store,
        record_outcome=record_outcome,
        overload_warning=lambda capacity: dependencies.logger.warning(
            "DetectorSim ideal source overload: superseding stale pose window "
            f"at capacity={capacity}; newest pose retained"
        ),
    )


def _build_pose_source(
    dependencies: SimDetectorDependencies,
    coordinator: SourceFrameCoordinator,
    clock: PoseFrameClock,
    association_period_budget_s: float,
) -> IdealPoseSource:
    vehicle = dependencies.vehicle
    cadence = dependencies.scheduler_cadence
    if cadence is None:
        receipt_skew_s = lambda: association_period_budget_s
    else:
        receipt_skew_s = lambda: cadence.wall_period_for_scheduler_period(
            association_period_budget_s
        )
    associator = PoseAssociator(
        attitude_sample=lambda: vehicle.attitude_sample,
        truth_pose=lambda: vehicle.simulator_truth_pose,
        air_speed=lambda: vehicle.air_speed,
        maximum_receipt_skew_s=receipt_skew_s,
        require_event_pair=True,
    )
    return IdealPoseSource(
        subscribe=lambda message_type, callback: vehicle.on_message(
            message_type,
            callback,
        ),
        is_armed=lambda: vehicle.is_armed is not False,
        error=dependencies.logger.error,
        coordinator=coordinator,
        clock=clock,
        associator=associator,
    )


def _build_activation(
    dependencies: SimDetectorDependencies,
    stream_rate_hz: float,
    source_driven: bool,
) -> SimSourceActivation:
    vehicle = dependencies.vehicle
    logger = dependencies.logger

    def request_pose() -> None:
        request_pose_streams(vehicle, logger, rate_hz=stream_rate_hz)

    def request_truth() -> None:
        request_simulator_truth_pose_stream(
            vehicle,
            logger,
            rate_hz=stream_rate_hz,
        )

    return SimSourceActivation(
        request_pose=request_pose,
        request_truth=request_truth if source_driven else None,
    )


def _target_system(vehicle: IVehicle) -> int:
    try:
        return int(vehicle.target_system or 0)
    except (TypeError, ValueError):
        return 0


__all__ = [
    "SimSourceGraph",
    "SourceTimestampClock",
    "build_polling_reader",
    "build_source_graph",
    "source_stream_rate_hz",
]
