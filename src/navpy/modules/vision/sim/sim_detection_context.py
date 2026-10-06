"""Narrow runtime ports for one simulator detection frame."""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Optional, Tuple

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.sim_camera_ports import ZoomSynchronizer
from navpy.modules.vision.sim.sim_frame_timestamp import FrameTimestampPort
from navpy.modules.vision.sim.sim_frame_transactions import (
    DetectionFrameTransactionsPort,
    PendingPublicationPort,
)
from navpy.modules.vision.sim.sim_frame_types import FrameContext
from navpy.modules.vision.sim.sim_runtime_ports import (
    PoiProjectorPort,
    PoiSnapshotReader,
)
from navpy.modules.vision.simulation_object import SimulationObject


@dataclass(frozen=True)
class SimDetectionContext:
    """Exact operations used to render and commit one frame."""

    ideal_360: bool
    sync_camera_zoom: ZoomSynchronizer
    poi_snapshot: PoiSnapshotReader
    project_poi: PoiProjectorPort
    timestamps: FrameTimestampPort
    transactions: DetectionFrameTransactionsPort

    def generation(self, supplied: FrameGeneration | None) -> FrameGeneration:
        return supplied if supplied is not None else self.transactions.generation

    def sync_zoom(self) -> None:
        if not self.ideal_360:
            self.sync_camera_zoom()

    def resolve_timestamp(
        self,
        attitude_time_boot_s: Optional[float],
        frame_timestamp_s: Optional[float],
    ) -> Optional[float]:
        return self.timestamps.resolve(attitude_time_boot_s, frame_timestamp_s)

    def pois(self) -> tuple[SimulationObject, ...]:
        return self.poi_snapshot()

    def update(
        self,
        camera_location: Location,
        poi: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: Optional[float],
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]],
        navigation_attitude: Optional[Attitude],
    ) -> Optional[DetectedObject]:
        return self.project_poi(
            camera_location,
            poi,
            uas_attitude,
            timestamp_s=timestamp_s,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
            navigation_attitude=navigation_attitude,
        )

    def prepare(
        self,
        generation: FrameGeneration,
    ) -> AbstractContextManager[Optional[FrameGeneration]]:
        return self.transactions.admit(generation)

    def reserve(
        self,
        frame: FrameContext,
    ) -> AbstractContextManager[Optional[PendingPublicationPort]]:
        return self.transactions.reserve(frame)

    def commit(
        self,
        frame: FrameContext,
        pending: PendingPublicationPort,
    ) -> AbstractContextManager[Optional[PendingPublicationPort]]:
        return self.transactions.commit(frame, pending)


__all__ = ["SimDetectionContext"]
