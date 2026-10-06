"""Source-timestamped visual tracking transaction."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace

from navpy.modules.navigation.gimbal_navigation_state import (
    GimbalDetectionMemory,
    GimbalSessionCommandGate,
    GimbalSessionFence,
    GimbalTrackers,
)
from navpy.modules.navigation.gimbal_loss_recovery import (
    GimbalLossRecovery,
    LossRecoveryResult,
)
from navpy.modules.navigation.gimbal_zoom_control import GimbalZoomController
from navpy.modules.vision.gimbal_rate_types import (
    GimbalObservationDisposition,
    GimbalTrackResult,
    TrackingState,
)
from navpy.modules.vision.gimbal_tracking_sample import GimbalTargetProjector
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class _VisualSession:
    generation: int
    recentered: bool
    holding: bool
    last_track_time: float | None
    cached_for_zoom: DetectedObject | None
    terminal_zoom_frozen: bool


class GimbalVisualTracking:
    """Consume visual targets and commit only to the current session."""

    def __init__(
        self,
        trackers: GimbalTrackers,
        gate: GimbalSessionCommandGate,
        fence: GimbalSessionFence,
        detection: GimbalDetectionMemory,
        zoom: GimbalZoomController,
        loss_recovery: GimbalLossRecovery,
    ) -> None:
        self._trackers = trackers
        self._gate = gate
        self._fence = fence
        self._detection = detection
        self._zoom = zoom
        self._loss_recovery = loss_recovery

    def update(
        self,
        target: DetectedObject | None,
        now: float | None = None,
        principal_point: tuple[float, float] | None = None,
    ) -> None:
        measurement_time, loss_time = _observation_times(target, now)
        with self._gate.lock:
            session = self._snapshot()
            if session is None:
                return
            disposition, pointing = self._update_rate(
                target,
                measurement_time,
                principal_point,
                session,
            )
            if disposition is GimbalObservationDisposition.STALE_NOOP:
                return
            effective_target = (
                target
                if disposition is GimbalObservationDisposition.ACCEPTED
                else None
            )
            recovery = self._recover(
                disposition,
                loss_time,
                session,
            )
            if recovery is None:
                return
            pointing = _loss_pointing(pointing, recovery)
            cached_for_zoom = self._zoom.update(
                effective_target,
                session.cached_for_zoom,
                session.terminal_zoom_frozen,
                pointing,
            )
            self._commit(session, recovery, cached_for_zoom)

    def _snapshot(self) -> _VisualSession | None:
        with self._fence.lock:
            if self._detection.tracking_obj_id is None:
                return None
            return _VisualSession(
                self._fence.generation,
                self._detection.recentered,
                self._detection.holding,
                self._detection.last_track_time,
                self._detection.last_tracked_for_zoom,
                self._detection.terminal_zoom_frozen_at_min,
            )

    def _update_rate(
        self,
        target: DetectedObject | None,
        measurement_time: float | None,
        principal_point: tuple[float, float] | None,
        session: _VisualSession,
    ) -> tuple[GimbalObservationDisposition, GimbalTrackResult | None]:
        tracker = self._trackers.rate
        if tracker is None:
            disposition = (
                GimbalObservationDisposition.ACCEPTED
                if target is not None and measurement_time is not None
                else GimbalObservationDisposition.NO_OBSERVATION
            )
            return disposition, None
        if target is None or measurement_time is None:
            return GimbalObservationDisposition.NO_OBSERVATION, tracker.last_result
        sample = GimbalTargetProjector.project(
            target,
            measurement_time,
            principal_point,
        )
        if sample is None:
            return GimbalObservationDisposition.NO_OBSERVATION, tracker.last_result
        before_actuation = None
        if session.recentered:
            before_actuation = lambda: self._loss_recovery.prepare_reacquire(
                True
            )
        if before_actuation is None:
            rate_update = tracker.update(sample)
        else:
            rate_update = tracker.update(
                sample,
                before_actuation=before_actuation,
            )
        return rate_update.disposition, rate_update.result

    def _recover(
        self,
        disposition: GimbalObservationDisposition,
        now: float,
        session: _VisualSession,
    ) -> LossRecoveryResult | None:
        if self._trackers.rate is None:
            return LossRecoveryResult(
                False,
                False,
                now if disposition is GimbalObservationDisposition.ACCEPTED else None,
                TrackingState.TRACKING,
            )
        if disposition is GimbalObservationDisposition.ACCEPTED:
            return self._loss_recovery.accept_target(now, session.holding)
        if disposition is GimbalObservationDisposition.NO_OBSERVATION:
            return self._loss_recovery.advance_loss(
                now,
                session.last_track_time,
                session.recentered,
                session.holding,
            )
        return None

    def _commit(
        self,
        session: _VisualSession,
        recovery: LossRecoveryResult,
        cached_for_zoom: DetectedObject | None,
    ) -> None:
        with self._fence.lock:
            if (
                self._detection.tracking_obj_id is None
                or self._fence.generation != session.generation
            ):
                return
            self._detection.last_tracked_for_zoom = cached_for_zoom
            self._detection.recentered = recovery.recentered
            self._detection.holding = recovery.holding
            self._detection.last_track_time = recovery.last_track_time


def _observation_times(
    target: DetectedObject | None,
    now: float | None,
) -> tuple[float | None, float]:
    explicit_time = _finite_time(now)
    target_time = _finite_time(
        None
        if target is None
        else (
            target.timing.detection_timestamp_s
            if isinstance(target, DetectedObject)
            else getattr(target, "timestamp", None)
        )
    )
    target_now = _clock_time(
        None
        if target is None
        else (
            target.timing.detection_now_s
            if isinstance(target, DetectedObject)
            else getattr(target, "timestamp_now_s", None)
        )
    )
    measurement_time = (
        target_time if target_time is not None else explicit_time
    )
    if target is None:
        measurement_time = None
    loss_time = next(
        (
            value
            for value in (explicit_time, target_now, target_time)
            if value is not None
        ),
        time.time(),
    )
    return measurement_time, loss_time


def _finite_time(value: object) -> float | None:
    if not isinstance(value, (int, float)):
        return None
    timestamp = float(value)
    return timestamp if math.isfinite(timestamp) else None


def _clock_time(provider: object) -> float | None:
    if callable(provider):
        try:
            provider = provider()
        except (OSError, RuntimeError, TypeError, ValueError):
            return None
    return _finite_time(provider)


def _loss_pointing(
    pointing: GimbalTrackResult | None,
    recovery: LossRecoveryResult,
) -> GimbalTrackResult | None:
    if pointing is None or recovery.pointing_state is TrackingState.TRACKING:
        return pointing
    return replace(
        pointing,
        state=recovery.pointing_state,
        has_target=False,
    )


__all__ = ["GimbalVisualTracking"]
