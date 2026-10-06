"""Opt-in observation of one unchanged finite projection and validation call."""
from __future__ import annotations
import threading
from typing import Optional, Protocol

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject, DetectResult, DetectStatus
from navpy.modules.vision.sim.finite_projection_evidence import EvidenceErrorSink, EvidenceSink, ProjectionCapture
from navpy.modules.vision.sim.sim_camera_ports import FrameSize, PixelValidator
from navpy.modules.vision.simulation_object import SimulationObject


class ObservedProjector(Protocol):
    def __call__(self, location: Location, target: SimulationObject, attitude: Attitude, *,
                 timestamp_s: Optional[float], uas_body_rates_rad_s: Optional[tuple[float, float, float]],
                 _evidence: ProjectionCapture) -> Optional[DetectedObject]: ...


class FiniteProjectionObserver:
    def __init__(self, sink: EvidenceSink, error_sink: Optional[EvidenceErrorSink]) -> None:
        self._sink = sink
        self._error_sink = error_sink
        self._failure_lock = threading.Lock()
        self._failures = 0

    @property
    def failures(self) -> int:
        with self._failure_lock:
            return self._failures

    def detect(self, project: ObservedProjector, pixel_valid: PixelValidator,
               frame_size: FrameSize, location: Location, target: SimulationObject,
               attitude: Attitude, timestamp: Optional[float],
               body_rates: Optional[tuple[float, float, float]]) -> DetectResult:
        capture = ProjectionCapture()
        capture.observe(capture.inputs, target, location, attitude, timestamp, body_rates, frame_size)
        try:
            projected = project(location, target, attitude, timestamp_s=timestamp,
                                uas_body_rates_rad_s=body_rates, _evidence=capture)
            if projected is None:
                self._emit(capture, capture.early_outcome or 'no_projection')
                return DetectResult(DetectStatus.OutOfView)
            valid = pixel_valid(float(projected.pixel.u_px), float(projected.pixel.v_px))
            self._emit(capture, 'detected' if valid else 'out_of_view', pixel_valid=bool(valid))
            return DetectResult(DetectStatus.DETECTED, projected) if valid else DetectResult(DetectStatus.OutOfView)
        except Exception as error:
            self._emit(capture, 'exception', exception=error)
            raise

    def _emit(self, capture: ProjectionCapture, outcome: str, *,
              pixel_valid: Optional[bool] = None, exception: Optional[Exception] = None) -> None:
        try:
            record = capture.finish(outcome, pixel_valid=pixel_valid, exception=exception)
            self._sink(record)
        except Exception as error:
            self._record_failure(type(error).__name__)

    def _record_failure(self, reason: str) -> None:
        with self._failure_lock:
            self._failures += 1
        if self._error_sink is not None:
            try:
                self._error_sink(f'SIM_PROJECTION_EVIDENCE_ERROR reason={reason}')
            except Exception:
                pass  # A failed warning cannot alter the production result.
