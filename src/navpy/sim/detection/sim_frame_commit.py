"""The generation-gated commit of one rendered simulator frame."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.detect_data import DetectedObject
from navpy.sim.camera.projection_run_recorder import FrameEvidence
from navpy.sim.detection.sim_confirmation_capture import ConfirmationCapture
from navpy.sim.detection.sim_detection_context import SimDetectionContext
from navpy.sim.detection.sim_detection_gap import (
    ForcedDetectionGapPolicy,
    ForcedGapPlan,
)
from navpy.sim.detection.sim_frame_transactions import PendingPublicationPort
from navpy.sim.detection.sim_frame_types import FrameContext
from navpy.sim.detection.sim_runtime_ports import OverlayPosition
from navpy.sim.detection.sim_tracking_update import SimTrackingUpdater


class SimFrameCommit:
    """Commit side effects (forced gap, confirmation, tracking), then publish last."""

    def __init__(
        self,
        context: SimDetectionContext,
        confirmation_capture: ConfirmationCapture,
        forced_gap_policy: ForcedDetectionGapPolicy,
        tracking_updater: SimTrackingUpdater,
        mark: Callable[[str], None],
    ) -> None:
        self._context = context
        self._confirmation_capture = confirmation_capture
        self._forced_gap_policy = forced_gap_policy
        self._tracking_updater = tracking_updater
        self._mark = mark

    def plan(self, timestamp_s: float) -> ForcedGapPlan:
        return self._forced_gap_policy.plan(timestamp_s)

    def commit(
        self,
        frame: FrameContext,
        pending: PendingPublicationPort,
        gap_plan: ForcedGapPlan,
        detections: list[DetectedObject],
        positions: Optional[list[OverlayPosition]],
        c_g_loc: Location,
        uas_att: Attitude,
        evidence: Optional[FrameEvidence],
    ) -> bool:
        accepted = False
        with self._context.commit(frame, pending) as publication:
            if publication is None:
                if evidence is not None:
                    evidence.publication = 'rejected'
            else:
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
        self._mark("publish")
        return accepted


__all__ = ["SimFrameCommit"]
