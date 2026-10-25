"""Explicit construction of profile-backed simulated detectors."""

from __future__ import annotations

from argparse import Namespace
from pathlib import Path

import numpy as np

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.perception.geo.zc_util import ZcUtil
from navpy.core.ports.dock_contact import DockContactSensor
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.pose_streams import resolve_pose_stream_rate_hz
from navpy.sim.detection.camera_frame_ports import CameraFrameFactoryPort
from navpy.sim.detection.detector_sim import DetectorSim
from navpy.sim.sim_cv_threads import apply_sim_cv_threads
from navpy.app.perception.vision_detector_factory_ports import VisionDetectorNode
from navpy.core.perception.profile.vision_profile_types import CameraMountSpec, VisionProfile
from navpy.app.perception.vision_tracking_composition import build_tracking_config
from navpy.sim.debug_view.vision_ui_ports import SimDebugSnapshot, SimDebugSource
from navpy.app.perception.vision_zoom_profile import build_zoom_config
from navpy.sim.world import closest_approach_record
from navpy.sim.world.closest_approach_record import ClosestApproach
from navpy.sim.world.dock_contact_feed import SimDockContactFeed
from navpy.sim.world.dock_contact_sensor import SimDockContactSensor, max_segment_s
from navpy.sim.world.dock_net_radius import sim_dock_net_radius_m


class SimDetectorFactory:
    """Compose one simulator detector without exposing controller state."""

    def __init__(
        self,
        vehicle: IVehicle,
        args: Namespace,
        logger: ILogger,
        geo_ref: GeoRefCalc,
        profile: VisionProfile,
        zc_util: ZcUtil | None,
        scheduler_cadence: SchedulerCadence | None,
        camera_frames: CameraFrameFactoryPort | None = None,
    ) -> None:
        self._vehicle = vehicle
        self._args = args
        self._logger = logger
        self._geo_ref = geo_ref
        self._profile = profile
        self._zc_util = zc_util
        self._scheduler_cadence = scheduler_cadence
        self._camera_frames = camera_frames
        self._assets_path = Path(__file__).resolve().parents[4] / ".sim"
        self._dock_contact: SimDockContactSensor | None = None
        self._dock_contact_feed: SimDockContactFeed | None = None
        apply_sim_cv_threads()

    @property
    def dock_contact(self) -> DockContactSensor | None:
        """The net stand-in; only for a simulated autopilot (else None)."""
        return self._dock_contact

    def _wire_dock_contact(self, detector: DetectorSim) -> None:
        # One sensor per process, on the first detector's world: every
        # detector loads the same docks, so a dock uid is the same in each.
        if self._dock_contact is not None:
            return
        if not self._vehicle.is_simulated_autopilot():
            return
        radius_m = sim_dock_net_radius_m()
        self._logger.info(f"SIM dock net radius: {radius_m:g} m")
        # The same rate the sim detector requests the position stream at.
        position_rate_hz = resolve_pose_stream_rate_hz(self._vehicle)
        sensor = SimDockContactSensor(
            detector.sim_docks,
            position_rate_hz,
            radius_m,
            report=self._report_closest_approach,
        )
        # The subscription lives as long as the vehicle link that feeds it.
        self._dock_contact_feed = SimDockContactFeed(
            sensor,
            self._vehicle.on_message,
            # An attitude labels a position only within the joining window.
            attitude_max_age_s=max_segment_s(position_rate_hz),
        )
        self._dock_contact = sensor

    def _report_closest_approach(self, record: ClosestApproach) -> None:
        """Display evidence only: the NavPy log and the GCS stdout record."""
        distance = record.distance
        reason = "" if record.reason is None else f" reason={record.reason}"
        self._logger.info(
            f"SIM_CLOSEST_APPROACH: task={record.task_id} dock={record.dock_uid} "
            f"episode={record.episode} {record.outcome}{reason} 3d={distance.d3_m:.2f}m "
            f"h={distance.h_m:.2f}m v={distance.v_m:.2f}m"
        )
        closest_approach_record.publish(record)

    def create(
        self,
        mount_spec: CameraMountSpec,
        detector_settings: VisionProfile,
        mount_index: int,
    ) -> VisionDetectorNode:
        ideal_360 = bool(detector_settings.get("ideal_360", False))
        tracking_config = (
            None
            if ideal_360
            else build_tracking_config(
                mount_spec.device,
                sim=True,
            )
        )
        zoom_config = (
            None
            if ideal_360
            else build_zoom_config(
                mount_spec.device,
                self._profile,
            )
        )
        detector = DetectorSim(
            vehicle=self._vehicle,
            mount=mount_spec.mount,
            geo_ref=self._geo_ref,
            logger=self._logger,
            args=self._args,
            zc_util=self._zc_util,
            sim_assets_path=None if ideal_360 else str(self._assets_path),
            tracking_config=tracking_config,
            zoom_config=zoom_config,
            scheduler_cadence=self._scheduler_cadence,
            ideal_360=ideal_360,
            camera_frames=(
                None
                if self._camera_frames is None
                else self._camera_frames(mount_index)
            ),
        )
        self._wire_dock_contact(detector)
        if ideal_360:
            self._logger.info(
                f"DetectorSim({mount_spec.mount.name}): "
                "static ideal 360 enabled"
            )
        self._logger.info(
            f"Created sim detector for mount '{mount_spec.mount.name}'"
        )

        def read_debug_snapshot() -> SimDebugSnapshot:
            detections = tuple(detector.get_latest_detections())
            class_id = detections[0].classification.class_id if detections else None
            raw_intrinsics = mount_spec.mount.get_k()
            intrinsics = (
                None
                if raw_intrinsics is None
                else np.array(raw_intrinsics, copy=True)
            )
            if intrinsics is not None:
                intrinsics.setflags(write=False)
            return SimDebugSnapshot(
                source_name=detector.source_name,
                gimbal=mount_spec.mount.get_gimbal_data(),
                image_width=mount_spec.mount.image_width or 1920,
                image_height=mount_spec.mount.image_height or 1080,
                intrinsics=intrinsics,
                current_zoom=mount_spec.mount.get_current_zoom(),
                detections=detections,
                rate_result=detector.rate_result,
                zoom_result=detector.get_zoom_result(),
                zoom_target_pixels=detector.get_zoom_target_pixels(class_id),
            )

        return VisionDetectorNode(
            detector=detector,
            cleanup=detector,
            real_ui=None,
            sim_debug=SimDebugSource(
                read_snapshot=read_debug_snapshot,
            ),
        )


__all__ = ["SimDetectorFactory"]
