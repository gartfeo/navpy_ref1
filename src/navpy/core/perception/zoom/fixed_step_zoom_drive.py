"""Drives the fixed-step zoom policy on one mount (HITL step 18).

``FixedStepZoom`` is the shared part: the calibrated steps and the settle evidence, which the frame association
(perception) and this drive (command) both read, so a frame is admitted under the same evidence the policy decides
from. The drive is the only zoom writer while fixed zoom is configured, and it writes calibrated steps only, as
absolute zoom commands through the mount's hardware zoom.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.perception.zoom.fixed_step_zoom_policy import FixedStepZoomPolicy, FrameBox
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps, ZoomStep
from navpy.core.perception.zoom.poi_zoom_geometry import extract_bbox
from navpy.core.perception.zoom.poi_zoom_ports import ZoomLogger
from navpy.core.perception.zoom.zoom_settle_evidence import (
    STATUS_ZOOM_ABORTED,
    STATUS_ZOOM_UNKNOWN,
    ZoomJudgement,
    ZoomSettleEvidence,
)
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory


@dataclass(frozen=True)
class FixedStepZoom:
    steps: FixedZoomSteps
    evidence: ZoomSettleEvidence


class FixedStepZoomDrive:
    def __init__(
        self,
        fixed: FixedStepZoom,
        history_reader: Callable[[], GimbalCaptureHistory | None],
        write: Callable[[str], bool],
        logger: ZoomLogger,
        policy: FixedStepZoomPolicy | None = None,
    ) -> None:
        self._fixed = fixed
        self._history_reader = history_reader
        self._write = write
        self._logger = logger
        self._policy = policy or FixedStepZoomPolicy(fixed.steps)
        self._failed: ZoomStep | None = None
        self._failed_session: int | None = None
        self._last: ZoomJudgement | None = None

    @property
    def is_stable(self) -> bool:
        return self._last is not None and self._last.settled

    @property
    def judgement(self) -> ZoomJudgement | None:
        return self._last

    def judge(self) -> ZoomJudgement:
        self._last = self._fixed.evidence.judge_latest(self._history_reader(), rt.perf_counter())
        return self._last

    def update(self, poi: DetectedObject | None, *, frozen: bool = False) -> None:
        """One decision from the newest POI (None: no detection this update)."""
        if poi is None:
            self.update_measurement(None, None, frozen=frozen)
            return
        box = _frame_box(poi)
        # A detection without a usable box makes no containment decision and is not a loss.
        self.update_measurement(box, poi.optics.zoom_command, frozen=frozen, detected=box is None)

    def update_measurement(
        self, box: FrameBox | None, box_step: str | None, *, frozen: bool = False, detected: bool = False,
    ) -> None:
        """One decision. ``box`` is a detection in a frame associated at zoom ``box_step``, or None for no
        detection (``detected``: a detection without a usable box, so no decision). ``frozen`` (guidance)
        holds the current step. In every case an unknown lens state is restored to the widest step."""
        judgement = self.judge()
        if judgement.status == STATUS_ZOOM_ABORTED:
            return
        failed = self._failed
        if failed is not None and (self._failed_session != judgement.session
                                   or (frozen and failed != self._fixed.steps.widest)):
            failed = self._failed = None        # a reconnect or a freeze since: the old write is void
        if failed is not None:
            self._command(failed, "retry")
            return
        if frozen or detected:
            if judgement.status == STATUS_ZOOM_UNKNOWN:
                self._command(self._fixed.steps.widest, "frozen_unknown" if frozen else STATUS_ZOOM_UNKNOWN)
            return
        if box is not None and (judgement.step is None or box_step != judgement.step.command):
            return                          # a detection from another step's frame: no decision
        step = self._policy.decide(judgement, box)
        if step is not None and step != judgement.step:
            self._command(step, judgement.status)

    def command_widest(self, reason: str) -> bool:
        """Command the widest step unless the lens is already set by it; True when written or not needed."""
        self._policy.reset()
        self._failed = None                 # an explicit widest command voids any pending write
        judgement = self.judge()
        widest = self._fixed.steps.widest
        if judgement.status == STATUS_ZOOM_ABORTED:
            return False                    # aborted evidence: no more zoom writes this session
        if judgement.step == widest and judgement.status != STATUS_ZOOM_UNKNOWN:
            return True
        return self._command(widest, reason)

    def _command(self, step: ZoomStep, reason: str) -> bool:
        ok = self._write(step.command) is True
        self._failed = None if ok else step
        self._failed_session = None if ok or self._last is None else self._last.session
        self._logger.info(f"fixed zoom: step {step.command} ({reason}){'' if ok else ' write failed'}")
        return ok


def _frame_box(poi: DetectedObject) -> FrameBox | None:
    bbox = extract_bbox(poi)
    width, height = poi.optics.frame_width_px, poi.optics.frame_height_px
    if bbox is None or width is None or height is None:
        return None
    return FrameBox.from_cxcywh(bbox, width, height)


__all__ = ["FixedStepZoom", "FixedStepZoomDrive"]
