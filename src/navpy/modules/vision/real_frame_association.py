"""Frame-atomic pose and optics association for the real detector."""

from __future__ import annotations

import copy
import math
import time
from dataclasses import dataclass

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.pose_streams import POSE_FRAME_ASSOCIATION_MAX_SKEW_S
from navpy.modules.vision.camera_mount import CameraMountFrameState
from navpy.modules.vision.capture_lookup import (
    CaptureLookupCalibration,
    attitude_lookup_wall_s,
    capture_estimate_s,
    interpolate_attitude_at,
    quarantine_window_s,
    validate_calibration,
)
from navpy.modules.vision.frame_provider import FrameSnapshot
from navpy.modules.vision.real_frame_association_gate import (
    CaptureTiming,
    finite_body_rates,
    finite_time,
    real_frame_association_quality,
)
from navpy.modules.vision.real_detector_ports import (
    AirSpeedReader,
    AttitudeHistoryReader,
    AttitudeSampleReader,
    AttitudeSampleView,
    LinkIdentityReader,
    LocationReader,
    MountFrameStateReader,
)

# Enough retained ATTITUDE samples to bracket a lookup that sits a full
# admitted capture latency behind now: the 0.5 s absolute skew ceiling at the
# 40 Hz pose rate is 20 samples; 64 leaves margin without copying the whole
# 1024-deep store on every frame.
ATTITUDE_HISTORY_COUNT = 64


def frame_pose_from_sample(
    attitude_sample: AttitudeSampleView | None,
) -> tuple[
    Attitude,
    tuple[float, float, float] | None,
    float | None,
    float | None,
] | None:
    """Extract one cached ATTITUDE value/receipt-time association."""
    if attitude_sample is None:
        return None
    uas_att = attitude_sample.attitude
    rates = attitude_sample.body_rates_rad_s
    body_rates = None
    if rates is not None:
        try:
            body_rates = (float(rates[0]), float(rates[1]), float(rates[2]))
        except (IndexError, TypeError, ValueError):
            body_rates = None
    pose_time_s = attitude_sample.time_boot_s
    try:
        pose_time_s = None if pose_time_s is None else float(pose_time_s)
    except (TypeError, ValueError):
        pose_time_s = None
    receipt_time_s = attitude_sample.receipt_time_s
    try:
        receipt_time_s = None if receipt_time_s is None else float(receipt_time_s)
    except (TypeError, ValueError):
        receipt_time_s = None
    if receipt_time_s is not None and not math.isfinite(receipt_time_s):
        receipt_time_s = None
    return uas_att, body_rates, pose_time_s, receipt_time_s


@dataclass(frozen=True)
class RealFrameAssociation:
    """Pixels and every state sample frozen for one detector publication."""

    frame: np.ndarray
    frame_width: int
    frame_height: int
    frame_sequence: int
    frame_timestamp_s: float | None
    associated_at_s: float
    uas_att: Attitude | None
    uas_body_rates_rad_s: tuple[float, float, float] | None
    attitude_boot_time_s: float | None
    attitude_receipt_time_s: float | None
    mount_state: CameraMountFrameState | None
    c_g_loc: Location | None
    air_speed_mps: float | None
    pose_is_frame_atomic: bool
    pose_status: str
    pose_age_s: float | None
    capture: CaptureTiming | None = None


class FrameAssociationBuilder:
    """Freezes all pose and camera state for the exact inference frame."""

    def __init__(
            self,
            attitude_sample_reader: AttitudeSampleReader,
            mount_state_reader: MountFrameStateReader,
            location_reader: LocationReader,
            air_speed_reader: AirSpeedReader,
            attitude_history_reader: AttitudeHistoryReader,
            link_identity_reader: LinkIdentityReader,
            calibration: CaptureLookupCalibration | None = None,
            source_id: str = "",
            maximum_skew_s: float = POSE_FRAME_ASSOCIATION_MAX_SKEW_S,
    ) -> None:
        self._attitude_sample_reader = attitude_sample_reader
        self._mount_state_reader = mount_state_reader
        self._location_reader = location_reader
        self._air_speed_reader = air_speed_reader
        self._attitude_history_reader = attitude_history_reader
        self._link_identity_reader = link_identity_reader
        self._calibration = calibration
        self._source_id = source_id
        self._maximum_skew_s = float(maximum_skew_s)

    def set_maximum_skew(self, maximum_skew_s: float) -> None:
        self._maximum_skew_s = float(maximum_skew_s)

    def _capture_lookup(
            self,
            snapshot: FrameSnapshot,
    ) -> tuple[CaptureTiming | None, str, object | None]:
        """Map the frame stamp to a lookup and interpolate, fail closed.

        Returns (timing, status, interpolated) where status is "bracketed"
        only when `interpolated` describes the capture instant.
        """
        stamp = getattr(snapshot, "capture", None)
        if stamp is None:
            return None, "real_frame_capture_timestamp_unavailable", None
        refusal = validate_calibration(
            self._calibration,
            self._source_id,
            self._link_identity_reader(),
        )
        if refusal is not None:
            return None, "real_frame_capture_uncalibrated", None
        calibration = self._calibration
        try:
            lookup_s = attitude_lookup_wall_s(stamp, calibration)
        except (TypeError, ValueError):
            # An unrecognized stamp kind fails closed, never rides a branch.
            return None, "real_frame_capture_timestamp_unavailable", None
        timing = CaptureTiming(
            kind=stamp.kind,
            captured_at_s=stamp.captured_at_s,
            lookup_wall_s=lookup_s,
            estimate_s=capture_estimate_s(lookup_s, calibration),
        )
        # The stream's cadence bounds follow the scheduler-derived runtime
        # rate: maximum_skew is TWO periods (set by PoseStreamRequester), so
        # the bracket ceiling is exactly it and the seam tolerance one period.
        interpolated, status = interpolate_attitude_at(
            self._attitude_history_reader(ATTITUDE_HISTORY_COUNT),
            lookup_s,
            quarantine_window_s(stamp.kind, calibration),
            bracket_max_width_s=self._maximum_skew_s,
            seam_tolerance_s=self._maximum_skew_s / 2.0,
        )
        return timing, status, interpolated

    def capture(
            self,
            snapshot: FrameSnapshot,
            frame: np.ndarray,
    ) -> RealFrameAssociation:
        frame_timestamp_s = finite_time(snapshot.published_at_s)
        capture_timing, capture_status, interpolated = self._capture_lookup(
            snapshot,
        )
        if interpolated is not None and capture_status == "bracketed":
            # The de-rotating attitude describes the CAPTURE instant.
            uas_att = interpolated.attitude
            body_rates = interpolated.body_rates_rad_s
            attitude_boot_time_s = interpolated.time_boot_s
            attitude_receipt_time_s = capture_timing.lookup_wall_s
        else:
            # Display/diagnostics path only: the latest sample keeps the
            # association representable; the gate below refuses atomicity.
            pose = frame_pose_from_sample(self._attitude_sample_reader())
            if pose is None:
                uas_att = None
                body_rates = None
                attitude_boot_time_s = None
                attitude_receipt_time_s = None
            else:
                (
                    uas_att,
                    body_rates,
                    attitude_boot_time_s,
                    attitude_receipt_time_s,
                ) = pose
                uas_att = copy.deepcopy(uas_att)

        try:
            mount_state = self._mount_state_reader(snapshot.width, snapshot.height)
        except Exception:
            mount_state = None
        try:
            c_g_loc = copy.deepcopy(self._location_reader())
        except Exception:
            c_g_loc = None
        associated_at_s = time.time()
        body_rates = finite_body_rates(body_rates)
        air_speed_mps = finite_time(self._air_speed_reader())
        if air_speed_mps is not None and air_speed_mps <= 0.0:
            air_speed_mps = None

        pose_is_atomic, pose_status, pose_age_s = real_frame_association_quality(
            capture=capture_timing,
            capture_status=capture_status,
            associated_at_s=associated_at_s,
            mount_state=mount_state,
            max_skew_s=self._maximum_skew_s,
        )
        return RealFrameAssociation(
            frame=frame,
            frame_width=snapshot.width,
            frame_height=snapshot.height,
            frame_sequence=snapshot.sequence,
            frame_timestamp_s=frame_timestamp_s,
            associated_at_s=associated_at_s,
            uas_att=uas_att,
            uas_body_rates_rad_s=body_rates,
            attitude_boot_time_s=attitude_boot_time_s,
            attitude_receipt_time_s=attitude_receipt_time_s,
            mount_state=mount_state,
            c_g_loc=c_g_loc,
            air_speed_mps=air_speed_mps,
            pose_is_frame_atomic=pose_is_atomic,
            pose_status=pose_status,
            pose_age_s=pose_age_s,
            capture=capture_timing,
        )


__all__ = [
    "ATTITUDE_HISTORY_COUNT",
    "CaptureTiming",
    "FrameAssociationBuilder",
    "RealFrameAssociation",
    "finite_body_rates",
    "finite_time",
    "frame_pose_from_sample",
    "real_frame_association_quality",
]
