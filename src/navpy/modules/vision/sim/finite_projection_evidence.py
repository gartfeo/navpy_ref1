"""Opt-in immutable inputs and results of simulated delivery projections."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import itertools
import json
import math
import os
from typing import Any, Callable, Optional

import numpy as np
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.simulation_object import SimulationObject

AttitudeValues = tuple[float, float, float]  # pitch, yaw, roll
LocationValues = tuple[float, float, float, bool]  # lat, lng, alt, is_absolute


def attitude_values(value: Optional[Attitude]) -> Optional[AttitudeValues]:
    return None if value is None else (float(value.pitch), float(value.yaw), float(value.roll))


def location_values(value: Location) -> LocationValues:
    return (float(value.lat), float(value.lng), float(value.alt), bool(value.is_absolute))


@dataclass(frozen=True)
class GimbalSnapshot:
    attitude: Optional[AttitudeValues]
    reference_aircraft_attitude: Optional[AttitudeValues]
    sequence: str
    degrees: bool
    setup_attitude: Optional[AttitudeValues]
    setup_sequence: str
    setup_degrees: bool
    setup_translation: tuple[float, ...]
    max_detect_distance: float
    timestamp_s: Optional[float]
    name: str

    @classmethod
    def capture(cls, value: GimbalData) -> GimbalSnapshot:
        return cls(attitude_values(value.att), attitude_values(value.reference_aircraft_attitude),
                   str(value.g_seq), bool(value.degrees), attitude_values(value.setup.att),
                   str(value.setup.seq), bool(value.setup.degrees), tuple(map(float, value.setup.dist)),
                   float(value.max_detect_distance),
                   None if value.timestamp_s is None else float(value.timestamp_s), str(value.name))


@dataclass(frozen=True)
class ProjectionSource:
    target_uid: Optional[int] = None
    source_timestamp_s: Optional[float] = None
    frame_timestamp_provided: bool = False
    camera_location: Optional[LocationValues] = None
    target_location: Optional[LocationValues] = None
    aircraft_attitude: Optional[AttitudeValues] = None
    body_rates_rad_s: Optional[tuple[float, ...]] = None
    configured_frame_size: Optional[tuple[int, int]] = None


@dataclass(frozen=True)
class ProjectionGeometry:
    matrix_values: Optional[tuple[float, ...]] = None
    matrix_shape: Optional[tuple[int, ...]] = None
    matrix_dtype: Optional[str] = None
    raw_gimbal: Optional[GimbalSnapshot] = None
    effective_gimbal: Optional[GimbalSnapshot] = None
    p_ned: Optional[tuple[float, ...]] = None
    pixel_uv: Optional[tuple[Optional[float], Optional[float]]] = None


@dataclass(frozen=True)
class ProjectionOutcome:
    name: str
    complete: bool
    snapshot_complete: bool
    source_identity_complete: bool
    pixel_valid: Optional[bool]
    exception_type: Optional[str]


@dataclass(frozen=True)
class FiniteProjectionEvidence:
    """Complete does not certify publication, logging integrity, or replay."""
    source: ProjectionSource
    geometry: ProjectionGeometry
    outcome: ProjectionOutcome


EvidenceSink = Callable[[FiniteProjectionEvidence], None]
EvidenceErrorSink = Callable[[str], None]


def projection_log_sink(debug: Callable[[str], None], vehicle_sysid: int, *,
                        is_debug_enabled: Optional[Callable[[], bool]] = None) -> Optional[EvidenceSink]:
    """Existing logger owns persistence. Rotation/overhead remain unqualified."""
    if os.environ.get('NAVPY_SIM_PROJECTION_EVIDENCE') != '1':
        return None
    if is_debug_enabled is not None and not is_debug_enabled():
        raise ValueError('projection evidence requires DEBUG logging')
    sequence = itertools.count(1)

    def emit(record: FiniteProjectionEvidence) -> None:
        envelope = dict(pid=os.getpid(), vehicle_sysid=vehicle_sysid,
                        projection_sequence=next(sequence), projection=asdict(record))
        debug('SIM_PROJECTION_EVIDENCE ' + json.dumps(envelope, allow_nan=False, separators=(',', ':')))

    return emit


@dataclass
class ProjectionCapture:
    """Call-local snapshots; failed conversion leaves explicitly incomplete data."""
    values: dict[str, Any] = field(default_factory=dict)
    complete: bool = True
    early_outcome: Optional[str] = None

    def observe(self, operation: Callable[..., dict[str, Any]], *args: Any) -> None:
        try:
            self.values.update(operation(*args))
        except Exception:
            self.complete = False

    @staticmethod
    def inputs(target: SimulationObject, location: Location, attitude: Attitude,
               timestamp: Optional[float], body_rates: Optional[tuple[float, ...]],
               frame_size: FrameSize) -> dict[str, Any]:
        return dict(target_uid=int(target.uid),
                    source_timestamp_s=None if timestamp is None else float(timestamp),
                    frame_timestamp_provided=timestamp is not None and not isinstance(timestamp, bool),
                    camera_location=location_values(location), target_location=location_values(target.g_loc),
                    aircraft_attitude=attitude_values(attitude),
                    body_rates_rad_s=None if body_rates is None else tuple(map(float, body_rates)),
                    configured_frame_size=(frame_size.width_px, frame_size.height_px))

    @staticmethod
    def optics(matrix: np.ndarray, gimbal: GimbalData) -> dict[str, Any]:
        return dict(matrix_values=tuple(map(float, matrix.flat)), matrix_shape=tuple(matrix.shape),
                    matrix_dtype=str(matrix.dtype), raw_gimbal=GimbalSnapshot.capture(gimbal))

    @staticmethod
    def projection_inputs(p_ned: np.ndarray, gimbal: GimbalData) -> dict[str, Any]:
        return dict(p_ned=tuple(map(float, p_ned)), effective_gimbal=GimbalSnapshot.capture(gimbal))

    def finish(self, outcome: str, *, pixel_valid: Optional[bool] = None,
               exception: Optional[Exception] = None) -> FiniteProjectionEvidence:
        source = ProjectionSource(**{key: value for key, value in self.values.items()
                                     if key in ProjectionSource.__dataclass_fields__})
        geometry = ProjectionGeometry(**{key: value for key, value in self.values.items()
                                         if key in ProjectionGeometry.__dataclass_fields__})
        identity_complete = (source.frame_timestamp_provided and source.source_timestamp_s is not None
                             and math.isfinite(source.source_timestamp_s))
        result = ProjectionOutcome(outcome, self.complete and identity_complete, self.complete,
                                   identity_complete, pixel_valid,
                                   None if exception is None else type(exception).__name__)
        return FiniteProjectionEvidence(source, geometry, result)
