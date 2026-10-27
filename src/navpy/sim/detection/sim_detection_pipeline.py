"""Detection rendering followed by one generation-gated commit."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.detect_data import DetectedObject
from navpy.sim.detection.frame_generation_gate import FrameGeneration
from navpy.sim.camera.projection_run_recorder import (
    ProjectionRunRecorder,
    projection_frame_scope,
)
from navpy.sim.detection.sim_confirmation_capture import ConfirmationCapture
from navpy.sim.detection.sim_detection_context import SimDetectionContext
from navpy.sim.detection.sim_detection_gap import ForcedDetectionGapPolicy
from navpy.sim.detection.sim_frame_commit import SimFrameCommit
from navpy.sim.detection.sim_frame_types import FrameContext
from navpy.sim.detection.sim_runtime_ports import OverlayPosition
from navpy.sim.detection.sim_tracking_update import SimTrackingUpdater


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
        stage: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._context = context
        self._confirmation_capture = confirmation_capture
        self._commit = SimFrameCommit(
            context, confirmation_capture, forced_gap_policy, tracking_updater, self._mark,
        )
        self._evidence_recorder = evidence_recorder
        # Diagnostics only: the poll timer's stage boundaries ("project" after
        # projection, "publish" after the commit, before the evidence write).
        self._stage = stage

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
        frame = self._open_frame(
            frame_generation,
            attitude_time_boot_s,
            frame_timestamp_s,
            frame_receipt_timestamp_s,
            frame_air_speed_mps,
            frame_source_discontinuity,
        )
        if frame is None:
            return False
        with self._context.reserve(frame) as pending:
            if pending is None:
                return False
            with projection_frame_scope(self._evidence_recorder, frame, frame_epoch) as evidence:
                gap_plan = self._commit.plan(frame.timestamp_s)
                detections, positions = self._collect_pois(
                    c_g_loc,
                    uas_att,
                    uas_body_rates_rad_s,
                    frame_navigation_attitude,
                    frame,
                )
                self._mark("project")
                return self._commit.commit(
                    frame, pending, gap_plan, detections, positions,
                    c_g_loc, uas_att, evidence,
                )

    def capture_frame(
        self,
        c_g_loc: Location,
        uas_att: Attitude,
        attitude_time_boot_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        *,
        frame_receipt_timestamp_s: Optional[float] = None,
        frame_air_speed_mps: Optional[float] = None,
    ) -> Optional["CapturedFrame"]:
        """Render now with this tick's pose, gimbal, optics and POIs;
        :meth:`CapturedFrame.publish` commits later (``camera_frames``).
        Projection evidence is not recorded."""
        frame = self._open_frame(
            None, attitude_time_boot_s, None,
            frame_receipt_timestamp_s, frame_air_speed_mps, None,
        )
        if frame is None:
            return None
        detections, positions = self._collect_pois(
            c_g_loc,
            uas_att,
            uas_body_rates_rad_s,
            None,
            frame,
        )
        self._mark("project")
        return CapturedFrame(self, frame, detections, positions, c_g_loc, uas_att)

    def _open_frame(
        self,
        frame_generation: Optional[FrameGeneration],
        attitude_time_boot_s: Optional[float],
        frame_timestamp_s: Optional[float],
        receipt_timestamp_s: Optional[float],
        air_speed_mps: Optional[float],
        source_discontinuity: Optional[bool],
    ) -> Optional[FrameContext]:
        generation = self._context.generation(frame_generation)
        with self._context.prepare(generation) as active:
            if active is None:
                return None
            self._context.sync_zoom()
            timestamp_s = self._context.resolve_timestamp(
                attitude_time_boot_s,
                frame_timestamp_s,
            )
        if timestamp_s is None:
            return None
        return FrameContext(
            timestamp_s,
            generation,
            receipt_timestamp_s,
            air_speed_mps,
            source_discontinuity,
        )

    def _publish_captured(
        self,
        captured: "CapturedFrame",
        receipt_timestamp_s: float,
    ) -> bool:
        # The receipt is the publication: the capture time stays the frame's.
        frame = replace(captured.frame, receipt_timestamp_s=receipt_timestamp_s)
        for detection in captured.detections:
            self._stamp_source_metadata(detection, frame)
        with self._context.reserve(frame) as pending:
            if pending is None:
                return False
            return self._commit.commit(
                frame, pending, self._commit.plan(frame.timestamp_s),
                captured.detections, captured.positions,
                captured.camera_location, captured.uas_attitude, None,
            )

    def _mark(self, stage: str) -> None:
        if self._stage is not None:
            self._stage(stage)

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
            rt.time if frame.receipt_timestamp_s is not None else None,
            frame.air_speed_mps,
        )


@dataclass(frozen=True)
class CapturedFrame:
    """A frame rendered at its capture tick, published once by its camera loop."""

    pipeline: SimDetectionPipeline
    frame: FrameContext
    detections: list[DetectedObject]
    positions: Optional[list[OverlayPosition]]
    camera_location: Location
    uas_attitude: Attitude

    @property
    def timestamp_s(self) -> float:
        return self.frame.timestamp_s

    def publish(self, receipt_timestamp_s: float) -> bool:
        """Commit now, unless a reset or stop since the capture rejects it."""
        return self.pipeline._publish_captured(self, receipt_timestamp_s)


__all__ = ["CapturedFrame", "SimDetectionPipeline"]
