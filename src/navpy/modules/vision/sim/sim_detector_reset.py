"""Reset transaction contents for a simulated detector."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from typing import Optional, Sequence

from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSource
from navpy.modules.vision.sim.sim_camera_ports import CameraRefresher
from navpy.modules.vision.sim.sim_detector_controls import SimTrackingControls
from navpy.modules.vision.sim.sim_detector_state import (
    ForcedGapState,
    SimCaptureState,
)
from navpy.modules.vision.sim.sim_runtime_ports import ResetAction
from navpy.modules.vision.sim.sim_target_catalog import SimTargetCatalog


class SimDetectorReset:
    """Prepare camera state and clear every reset-owned component."""

    def __init__(
        self,
        *,
        pose_source: IdealPoseSource,
        target_catalog: SimTargetCatalog,
        capture_state: SimCaptureState,
        gap_state: ForcedGapState,
        tracking: SimTrackingControls,
        camera_refresh: CameraRefresher,
        ideal_camera: Optional[IdealCameraState],
    ) -> None:
        self._pose_source = pose_source
        self._target_catalog = target_catalog
        self._capture_state = capture_state
        self._gap_state = gap_state
        self._tracking = tracking
        self._camera_refresh = camera_refresh
        self._ideal_camera = ideal_camera

    def prepare(self) -> None:
        """Prepare mutable sensor state while frame admission is closed."""
        actions: list[ResetAction] = [
            self._pose_source.reset_state,
            self._camera_refresh,
        ]
        if self._ideal_camera is not None:
            actions.append(self._ideal_camera.prepare)
        self._run_all(actions)

    def refresh(self) -> None:
        """Clear all owners; the coordinator opens only if every step passes."""
        actions: list[ResetAction] = [
            self._pose_source.reset_state,
            self._target_catalog.refresh,
            self._capture_state.reset,
            self._gap_state.reset,
            self._tracking.reset,
        ]
        if self._ideal_camera is not None:
            actions.append(self._ideal_camera.reset)
        actions.append(self._camera_refresh)
        if self._ideal_camera is not None:
            actions.append(self._ideal_camera.prepare)
        self._run_all(actions)

    @staticmethod
    def _run_all(actions: Sequence[ResetAction]) -> None:
        errors: list[Exception] = []
        for action in actions:
            try:
                action()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise ExceptionGroup("simulator detector reset failed", errors)


__all__ = ["SimDetectorReset"]
