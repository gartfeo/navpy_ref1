"""Association of raw AP attitude with simulator truth used for rendering."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, TypeVar

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.sim_runtime_ports import (
    AttitudeSampleReader,
    AttitudeSampleView,
    OptionalFloatReader,
)


IDEAL_POSE_MESSAGE_TYPES = ("ATTITUDE", "GLOBAL_POSITION_INT", "SIM_STATE")


class TruthPoseView(Protocol):
    location: Location
    attitude: Attitude
    receipt_time_s: float


class TruthPoseReader(Protocol):
    def __call__(self) -> Optional[TruthPoseView]: ...


@dataclass(frozen=True)
class AssociatedPose:
    attitude_sample: AttitudeSampleView
    truth_pose: TruthPoseView
    attitude_timestamp_s: float
    attitude_receipt_s: float
    truth_receipt_s: float
    air_speed_mps: float


def copy_attitude(attitude: Attitude) -> Attitude:
    return Attitude(
        float(attitude.pitch), float(attitude.yaw), float(attitude.roll),
    )


def copy_location(location: Location) -> Location:
    return Location(
        float(location.lat),
        float(location.lng),
        float(location.alt),
        heading=float(location.heading),
        is_absolute=bool(location.is_absolute),
    )


class PoseAssociator:
    """Own event-pair and receipt freshness state for pose construction."""

    def __init__(
            self,
            *,
            attitude_sample: AttitudeSampleReader,
            truth_pose: TruthPoseReader,
            air_speed: OptionalFloatReader,
            maximum_receipt_skew_s: Callable[[], float],
            require_event_pair: bool,
    ) -> None:
        self._attitude_sample = attitude_sample
        self._truth_pose = truth_pose
        self._air_speed = air_speed
        self._maximum_receipt_skew_s = maximum_receipt_skew_s
        self._require_event_pair = bool(require_event_pair)
        self._event_sequence = 0
        self._attitude_event_sequence: Optional[int] = None
        self._truth_event_sequence: Optional[int] = None
        self._last_attitude_receipt_s: Optional[float] = None
        self._last_truth_receipt_s: Optional[float] = None

    @property
    def require_event_pair(self) -> bool:
        return self._require_event_pair

    def note_event(self, message_type: Optional[str]) -> None:
        if message_type not in IDEAL_POSE_MESSAGE_TYPES:
            return
        self._event_sequence += 1
        if message_type == "ATTITUDE":
            self._attitude_event_sequence = self._event_sequence
        elif message_type in {"SIM_STATE", "GLOBAL_POSITION_INT"}:
            self._truth_event_sequence = self._event_sequence

    def associate(self) -> Optional[AssociatedPose]:
        attitude_sample = self._attitude_sample()
        truth_pose = self._truth_pose()
        if attitude_sample is None or truth_pose is None:
            return None
        try:
            attitude_timestamp_s = float(attitude_sample.time_boot_s)
            attitude_receipt_s = float(attitude_sample.receipt_time_s)
            truth_receipt_s = float(truth_pose.receipt_time_s)
        except (AttributeError, TypeError, ValueError):
            return None
        if not all(map(math.isfinite, (
                attitude_timestamp_s,
                attitude_receipt_s,
                truth_receipt_s,
        ))):
            return None
        if not self._receipts_associated(attitude_receipt_s, truth_receipt_s):
            return None
        if not self._event_pair_complete():
            return None
        if not self._receipts_new(attitude_receipt_s, truth_receipt_s):
            return None
        try:
            air_speed_mps = float(self._air_speed())
        except (TypeError, ValueError):
            return None
        if not math.isfinite(air_speed_mps) or air_speed_mps <= 0.0:
            return None
        return AssociatedPose(
            attitude_sample,
            truth_pose,
            attitude_timestamp_s,
            attitude_receipt_s,
            truth_receipt_s,
            air_speed_mps,
        )

    def build_sample(
            self,
            association: AssociatedPose,
            *,
            timestamp_s: float,
            version: FrameGeneration,
            sample_type: type["PoseSampleT"],
    ) -> Optional["PoseSampleT"]:
        try:
            source_attitude = association.attitude_sample.attitude
            body_rates = association.attitude_sample.body_rates_rad_s
            return sample_type(
                timestamp_s=timestamp_s,
                location=copy_location(association.truth_pose.location),
                render_attitude=copy_attitude(association.truth_pose.attitude),
                navigation_attitude=Attitude(
                    pitch=float(source_attitude.pitch),
                    yaw=0.0,
                    roll=float(source_attitude.roll),
                ),
                generation=version,
                receipt_time_s=max(
                    association.attitude_receipt_s,
                    association.truth_receipt_s,
                ),
                air_speed_mps=association.air_speed_mps,
                body_rates_rad_s=(tuple(body_rates) if body_rates is not None else None),
            )
        except (AttributeError, TypeError, ValueError):
            return None

    def commit(self, association: AssociatedPose) -> None:
        self._last_attitude_receipt_s = association.attitude_receipt_s
        self._last_truth_receipt_s = association.truth_receipt_s
        self._attitude_event_sequence = None
        self._truth_event_sequence = None

    def reset(self) -> None:
        self._last_attitude_receipt_s = None
        self._last_truth_receipt_s = None
        self._attitude_event_sequence = None
        self._truth_event_sequence = None

    def _receipts_associated(
            self,
            attitude_receipt_s: float,
            truth_receipt_s: float,
    ) -> bool:
        try:
            maximum_skew_s = float(self._maximum_receipt_skew_s())
        except (TypeError, ValueError):
            return False
        if not math.isfinite(maximum_skew_s) or maximum_skew_s < 0.0:
            return False
        skew_s = abs(attitude_receipt_s - truth_receipt_s)
        return skew_s <= maximum_skew_s or math.isclose(
            skew_s,
            maximum_skew_s,
            rel_tol=1e-12,
            abs_tol=1e-12,
        )

    def _event_pair_complete(self) -> bool:
        pairing_active = (
            self._require_event_pair
            or self._attitude_event_sequence is not None
            or self._truth_event_sequence is not None
        )
        if not pairing_active:
            return True
        return (
            self._attitude_event_sequence is not None
            and self._truth_event_sequence is not None
            and abs(self._attitude_event_sequence - self._truth_event_sequence)
            < len(IDEAL_POSE_MESSAGE_TYPES)
        )

    def _receipts_new(
            self,
            attitude_receipt_s: float,
            truth_receipt_s: float,
    ) -> bool:
        return not (
            self._last_attitude_receipt_s is not None
            and attitude_receipt_s <= self._last_attitude_receipt_s
            or self._last_truth_receipt_s is not None
            and truth_receipt_s <= self._last_truth_receipt_s
        )


PoseSampleT = TypeVar("PoseSampleT")
