"""Transactional target-zoom session and reset lifecycle."""

from __future__ import annotations

from navpy.modules.vision.target_zoom_actuator_epoch import (
    TargetZoomActuatorEpoch,
)
from navpy.modules.vision.target_zoom_continuous import ContinuousZoomTick
from navpy.modules.vision.target_zoom_drive import ZoomDrive
from navpy.modules.vision.target_zoom_readback import ZoomReadbackState
from navpy.modules.vision.target_zoom_session import TargetZoomSession
from navpy.modules.vision.target_zoom_types import ZoomStopPlan
from navpy.modules.vision.target_zoom_update import TargetZoomWarningState


class TargetZoomLifecycle:
    def __init__(
        self,
        drive: ZoomDrive,
        readback: ZoomReadbackState,
        continuous: ContinuousZoomTick,
        session: TargetZoomSession,
        warnings: TargetZoomWarningState,
        epoch: TargetZoomActuatorEpoch,
    ) -> None:
        self._drive = drive
        self._readback = readback
        self._continuous = continuous
        self._session = session
        self._warnings = warnings
        self._epoch = epoch

    def set_size_demand(self, enabled: bool) -> None:
        self._session.set_size_demand(enabled)

    def start_session(self) -> bool:
        plan = self.prepare_session_start()
        if plan is None:
            return False
        self.commit_session_start(plan)
        return True

    def prepare_session_start(self) -> ZoomStopPlan | None:
        self._epoch.refresh()
        return self._drive.prepare_stop(lifecycle=True)

    def commit_session_start(self, plan: ZoomStopPlan) -> None:
        self._drive.commit_stop(plan, lifecycle=True)
        self._commit_reset(clear_gate=not plan.was_active)
        self._session.start()

    def reset(self) -> bool:
        self._epoch.refresh()
        plan = self._drive.prepare_stop(lifecycle=True)
        if plan is None:
            return False
        self._drive.commit_stop(plan, lifecycle=True)
        self._commit_reset(clear_gate=not plan.was_active)
        return True

    def reset_to_min(self) -> bool:
        self._epoch.refresh()
        plan = self._drive.prepare_stop(lifecycle=True)
        if plan is None:
            return False
        readback = self._readback.capture()
        if readback.invalid:
            return False
        if not self._drive.seek_minimum(readback.sample_id):
            return False
        self._drive.commit_stop(
            plan,
            lifecycle=True,
            sample_id=readback.sample_id,
        )
        self._commit_reset(clear_gate=False)
        return True

    def _commit_reset(self, *, clear_gate: bool) -> None:
        if clear_gate:
            self._drive.clear_without_hold()
        self._continuous.reset()
        self._session.reset_result()
        self._warnings.reset()


__all__ = ["TargetZoomLifecycle"]
