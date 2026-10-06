"""One source-driven POI-zoom observation transaction."""

from __future__ import annotations

from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.poi_zoom_actuator_epoch import (
    PoiZoomActuatorEpoch,
)
from navpy.modules.vision.poi_zoom_continuous import ContinuousZoomTick
from navpy.modules.vision.poi_zoom_drive import ZoomDrive
from navpy.modules.vision.poi_zoom_geometry import build_observation, extract_bbox
from navpy.modules.vision.poi_zoom_ports import (
    ZoomCapabilityReader,
    ZoomLogger,
    ZoomOpticsPort,
)
from navpy.modules.vision.poi_zoom_readback import ZoomReadbackState
from navpy.modules.vision.poi_zoom_session import PoiZoomSession
from navpy.modules.vision.poi_zoom_setpoint import (
    plan_absolute,
    plan_discrete,
    plan_widen,
)
from navpy.modules.vision.poi_zoom_types import (
    ZoomCapabilities,
    ZoomObservation,
    ZoomTrackResult,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


class PoiZoomWarningState:
    def __init__(self) -> None:
        self._unsupported = False
        self._discrete = False

    def first_unsupported(self) -> bool:
        if self._unsupported:
            return False
        self._unsupported = True
        return True

    def first_discrete(self) -> bool:
        if self._discrete:
            return False
        self._discrete = True
        return True

    def reset(self) -> None:
        self._unsupported = False
        self._discrete = False


def _unsupported(has_poi: bool) -> ZoomTrackResult:
    return ZoomTrackResult(
        ZoomTrackingState.UNSUPPORTED,
        has_poi,
        reason="unsupported",
    )


def _readback_failure(observation: ZoomObservation) -> ZoomTrackResult:
    return ZoomTrackResult(
        ZoomTrackingState.HOLDING,
        True,
        size_px=observation.size_px,
        target_pixels=observation.target_pixels,
        reason="readback",
    )


def _optics_failure(has_poi: bool) -> ZoomTrackResult:
    return ZoomTrackResult(
        ZoomTrackingState.HOLDING,
        has_poi,
        reason="optics",
    )


def _settling(
    observation: ZoomObservation,
    current: float | None,
) -> ZoomTrackResult:
    return ZoomTrackResult(
        ZoomTrackingState.HOLDING,
        True,
        size_px=observation.size_px,
        target_pixels=observation.target_pixels,
        reason="settling",
        current_zoom=current,
        transition_pending=True,
    )


class PoiZoomUpdate:
    def __init__(
        self,
        capabilities: ZoomCapabilityReader,
        optics: ZoomOpticsPort,
        readback: ZoomReadbackState,
        drive: ZoomDrive,
        continuous: ContinuousZoomTick,
        session: PoiZoomSession,
        warnings: PoiZoomWarningState,
        logger: ZoomLogger,
        epoch: PoiZoomActuatorEpoch,
    ) -> None:
        self._capabilities = capabilities
        self._optics = optics
        self._readback = readback
        self._drive = drive
        self._continuous = continuous
        self._session = session
        self._warnings = warnings
        self._logger = logger
        self._epoch = epoch

    def update(
        self,
        poi: object | None,
        pointing: GimbalTrackResult | None,
    ) -> ZoomTrackResult:
        self._epoch.refresh()
        capabilities = self._capabilities.capabilities
        if not capabilities.available:
            return self._session.record(_optics_failure(poi is not None))
        if not capabilities.supported:
            if self._warnings.first_unsupported():
                self._logger.warning(
                    "PoiZoomTracker disabled: no zoom capability"
                )
            return self._session.record(_unsupported(poi is not None))
        if self._optics.sync_optics() is None:
            return self._session.record(_optics_failure(poi is not None))
        bbox = extract_bbox(poi)
        if bbox is None:
            return self._on_poi_loss()

        target_pixels = self._session.target_pixels(poi)
        geometry = self._optics.geometry()
        if geometry.invalid:
            return self._session.record(_optics_failure(True))
        observation = build_observation(
            bbox,
            target_pixels,
            pointing,
            geometry.value,
            self._session.size_demand,
        )
        if self._session.widen_pending and capabilities.absolute:
            widened = self._update_widen(observation, capabilities)
            if widened is not None:
                return widened
        if capabilities.continuous:
            return self._session.record(
                self._continuous.update(observation, capabilities)
            )
        if capabilities.absolute:
            return self._update_absolute(observation, capabilities)
        return self._update_discrete(observation, capabilities)

    def _on_poi_loss(self) -> ZoomTrackResult:
        was_active = self._drive.active
        if not self._drive.stop(reason="poi-loss", lifecycle=True):
            active = self._drive.continuous_direction or ZoomTrackingState.HOLDING
            return self._session.record(
                ZoomTrackResult(active, False, reason="actuator-error")
            )
        self._continuous.reset()
        if not was_active:
            self._readback.clear_command_gate()
        self._session.reset_result()
        return self._session.last_result

    def _update_widen(
        self,
        observation: ZoomObservation,
        capabilities: ZoomCapabilities,
    ) -> ZoomTrackResult | None:
        readback = self._readback.capture(current=True)
        if readback.current.invalid or readback.fresh.invalid:
            return self._session.record(_readback_failure(observation))
        if readback.current.value is None:
            return None
        plan = plan_widen(
            readback.current.value,
            capabilities,
        )
        if plan.command_key is not None and not readback.sample_advanced:
            return self._session.record(
                _settling(observation, readback.current.value)
            )
        outcome = self._drive.apply_absolute(
            plan,
            size_px=observation.size_px,
            target_pixels=observation.target_pixels,
            sample_id=readback.sample_id,
        )
        if outcome.committed:
            self._session.complete_widen()
        return self._session.record(outcome.result)

    def _update_absolute(
        self,
        observation: ZoomObservation,
        capabilities: ZoomCapabilities,
    ) -> ZoomTrackResult:
        readback = self._readback.capture(current=True)
        if readback.invalid or readback.current.value is None:
            return self._session.record(_readback_failure(observation))
        plan = plan_absolute(
            observation,
            readback.current.value,
            readback.fresh.value,
            capabilities,
        )
        if (
            plan.command_key is not None
            and plan.command_key != self._drive.absolute_target
            and not readback.sample_advanced
        ):
            return self._session.record(
                _settling(observation, readback.current.value)
            )
        outcome = self._drive.apply_absolute(
            plan,
            size_px=observation.size_px,
            target_pixels=observation.target_pixels,
            sample_id=readback.sample_id,
        )
        return self._session.record(outcome.result)

    def _update_discrete(
        self,
        observation: ZoomObservation,
        capabilities: ZoomCapabilities,
    ) -> ZoomTrackResult:
        if self._warnings.first_discrete():
            self._logger.warning(
                "Using calibrated-level discrete zoom fallback"
            )
        readback = self._readback.capture(level=True)
        if readback.invalid or readback.level.value is None:
            return self._session.record(_readback_failure(observation))
        plan = plan_discrete(
            observation,
            readback.level.value,
            readback.fresh.value,
            capabilities,
        )
        if plan.command_key is not None and not readback.sample_advanced:
            return self._session.record(
                _settling(observation, readback.level.value)
            )
        outcome = self._drive.apply_discrete(
            plan,
            size_px=observation.size_px,
            target_pixels=observation.target_pixels,
            sample_id=readback.sample_id,
        )
        return self._session.record(outcome.result)


__all__ = ["PoiZoomUpdate", "PoiZoomWarningState"]
