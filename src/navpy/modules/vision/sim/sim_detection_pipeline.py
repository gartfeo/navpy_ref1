"""Detection rendering followed by one generation-gated commit."""

from __future__ import annotations

import time
from typing import Optional, Tuple

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.projection_run_recorder import ProjectionRunRecorder, projection_frame_scope
from navpy.modules.vision.sim.sim_confirmation_capture import ConfirmationCapture
from navpy.modules.vision.sim.sim_detection_context import SimDetectionContext
from navpy.modules.vision.sim.sim_detection_gap import ForcedDetectionGapPolicy
from navpy.modules.vision.sim.sim_frame_types import FrameContext
from navpy.modules.vision.sim.sim_runtime_ports import OverlayPosition
from navpy.modules.vision.sim.sim_tracking_update import SimTrackingUpdater


class SimDetectionPipeline:
    """Render unlocked, then commit side effects and publish last."""

    def __init__(
        self,
        context: SimDetectionContext,
        *,
        confirmation_capture: ConfirmationCapture,
        forced_gap_policy: ForcedDetectionGapPolicy,
        tracking_updater: SimTrackingUpdater,
        evidence_recorder: Optional[ProjectionRunRecorder] = None,
    ) -> None:
        self._context = context
        self._confirmation_capture = confirmation_capture
        self._forced_gap_policy = forced_gap_policy
        self._tracking_updater = tracking_updater
        self._evidence_recorder = evidence_recorder

    def detect_pois(
        self,
        c_g_loc: Location,
        uas_att: Attitude,
        attitude_time_boot_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        *,
        frame_timestamp_s: Optional[float] = None,
        frame_receipt_timestamp_s: Optional[float] = None,
        frame_air_speed_mps: Optional[float] = None,
        frame_navigation_attitude: Optional[Attitude] = None,
        frame_epoch: Optional[int] = None,
        frame_generation: Optional[FrameGeneration] = None,
        frame_source_discontinuity: Optional[bool] = None,
    ) -> bool:
        generation = self._context.generation(frame_generation)
        with self._context.prepare(generation) as active:
            if active is None:
                return False
            self._context.sync_zoom()
            timestamp_s = self._context.resolve_timestamp(
                attitude_time_boot_s,
                frame_timestamp_s,
            )
        if timestamp_s is None:
            return False
        frame = FrameContext(
            timestamp_s,
            generation,
            frame_receipt_timestamp_s,
            frame_air_speed_mps,
            frame_source_discontinuity,
        )
        with self._context.reserve(frame) as pending:
            if pending is None:
                return False
            with projection_frame_scope(self._evidence_recorder, frame, frame_epoch) as evidence:
                gap_plan = self._forced_gap_policy.plan(frame.timestamp_s)
                detections, positions = self._collect_pois(
                    c_g_loc,
                    uas_att,
                    uas_body_rates_rad_s,
                    frame_navigation_attitude,
                    frame,
                )
                with self._context.commit(frame, pending) as publication:
                    if publication is None:
                        if evidence is not None:
                            evidence.publication = 'rejected'
                        return False
                    if self._forced_gap_policy.commit(gap_plan):
                        detections = []
                        if positions is not None:
                            positions = []
                    self._confirmation_capture.capture(detections, positions)
                    self._tracking_updater.update(
                        detections,
                        c_g_loc,
                        uas_att,
                        frame.timestamp_s,
                    )
                    accepted = publication.publish(detections)
                    if evidence is not None:
                        evidence.published(accepted, len(detections))
                    return accepted

    def _collect_pois(
        self,
        camera_location: Location,
        uas_attitude: Attitude,
        body_rates: Optional[Tuple[float, float, float]],
        navigation_attitude: Optional[Attitude],
        frame: FrameContext,
    ) -> tuple[list[DetectedObject], Optional[list[OverlayPosition]]]:
        positions = [] if self._confirmation_capture.wants_frame() else None
        detections = []
        for poi in self._context.pois():
            detection = self._context.update(
                camera_location,
                poi,
                uas_attitude,
                timestamp_s=frame.timestamp_s,
                uas_body_rates_rad_s=body_rates,
                navigation_attitude=navigation_attitude,
            )
            if detection is None:
                continue
            self._stamp_source_metadata(detection, frame)
            if detection.pixel.u_px is None or detection.pixel.v_px is None:
                continue
            detections.append(detection)
            if positions is not None:
                positions.append(self._confirmation_capture.overlay_position(
                    detection,
                    poi,
                    camera_location,
                ))
        return detections, positions

    @staticmethod
    def _stamp_source_metadata(
        detection: DetectedObject,
        frame: FrameContext,
    ) -> None:
        detection.record_receipt(
            frame.receipt_timestamp_s,
            time.time if frame.receipt_timestamp_s is not None else None,
            frame.air_speed_mps,
        )


__all__ = ["SimDetectionPipeline"]
