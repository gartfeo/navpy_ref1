"""Callback-driven ideal simulator pose source.

POI truth is retained only long enough to render a frame-local visual ray.
The paired navigation attitude is yaw-redacted before it leaves this source.
"""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

import threading
from dataclasses import dataclass
from typing import Any, Iterable, Optional

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.message_subscriptions import Subscription
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.sim_coordinator_ports import (
    ArmedReader,
    MessageSubscriber,
    PoseSourceCoordinatorPort,
)
from navpy.modules.vision.sim.sim_runtime_ports import ErrorSink
from navpy.modules.vision.sim.pose_associator import (
    IDEAL_POSE_MESSAGE_TYPES,
    PoseAssociator,
    copy_attitude,
    copy_location,
)
from navpy.modules.vision.sim.pose_frame_clock import (
    POSE_CLOCK_FRESH_BOOT_WINDOW_S,
    POSE_CLOCK_REBOOT_JUMP_S,
    PoseFrameClock,
    TimestampValue,
)


IDEAL_SOURCE_MESSAGE_TYPES = (*IDEAL_POSE_MESSAGE_TYPES, "HEARTBEAT")


@dataclass(frozen=True)
class IdealPoseSample:
    """Renderer truth paired with allowed AP pitch/roll at one source time."""

    timestamp_s: float
    location: Location
    render_attitude: Attitude
    navigation_attitude: Attitude
    generation: FrameGeneration
    receipt_time_s: Optional[float] = None
    air_speed_mps: Optional[float] = None
    source_discontinuity: bool = False
    body_rates_rad_s: tuple[float, float, float] | None = None


# Stable constant/function aliases used by source-time characterization tests.
_IDEAL_POSE_MESSAGE_TYPES = IDEAL_POSE_MESSAGE_TYPES
_IDEAL_SOURCE_MESSAGE_TYPES = IDEAL_SOURCE_MESSAGE_TYPES
_POSE_CLOCK_REBOOT_JUMP_S = POSE_CLOCK_REBOOT_JUMP_S
_POSE_CLOCK_FRESH_BOOT_WINDOW_S = POSE_CLOCK_FRESH_BOOT_WINDOW_S
_IdealPoseSample = IdealPoseSample
_copy_attitude = copy_attitude
_copy_location = copy_location


class IdealPoseSource:
    """Own public subscriptions and raw-AP pose association."""

    def __init__(
        self,
        *,
        subscribe: MessageSubscriber,
        is_armed: ArmedReader,
        error: ErrorSink,
        coordinator: PoseSourceCoordinatorPort,
        clock: PoseFrameClock,
        associator: PoseAssociator,
    ) -> None:
        self._subscribe = subscribe
        self._is_armed = is_armed
        self._error = error
        self._coordinator = coordinator
        self._clock = clock
        self._associator = associator
        self._lifecycle_lock = threading.Lock()
        self._subscriptions: tuple[Subscription, ...] = ()
        self._source_armed = True

    @staticmethod
    def message_boot_time_s(message: Any) -> Optional[float]:
        return PoseFrameClock.message_boot_time_s(message)

    @staticmethod
    def pose_clock_restarted(
        candidate_s: float,
        current_s: float,
        *,
        reorder_tolerance_s: float = 0.0,
    ) -> bool:
        return PoseFrameClock.restarted(
            candidate_s,
            current_s,
            reorder_tolerance_s=reorder_tolerance_s,
        )

    @property
    def source_now_s(self) -> Optional[float]:
        return self._clock.source_now_s

    @property
    def subscriptions(self) -> tuple[Subscription, ...]:
        return self._subscriptions

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._coordinator.stop_event.is_set() or self._subscriptions:
                return
            registered: list[Subscription] = []
            try:
                for message_type in IDEAL_SOURCE_MESSAGE_TYPES:
                    subscription = self._subscribe(message_type, self.on_message)
                    subscription.cancel
                    registered.append(subscription)
            except Exception as registration_error:
                if isinstance(registration_error, AttributeError):
                    registration_error = RuntimeError(
                        "DetectorSim ideal sensor requires public subscriptions"
                    )
                    self._error(str(registration_error))
                cancellation_errors = self._cancel_all(registered)
                if cancellation_errors:
                    raise ExceptionGroup(
                        "ideal pose subscription registration failed",
                        [registration_error, *cancellation_errors],
                    ) from None
                raise registration_error
            self._subscriptions = tuple(registered)

    def detach(self) -> None:
        with self._lifecycle_lock:
            subscriptions = self._subscriptions
            self._subscriptions = ()
        errors = self._cancel_all(subscriptions)
        if errors:
            raise ExceptionGroup(
                "ideal pose subscription cancellation failed",
                errors,
            )

    def _cancel_all(
        self,
        subscriptions: Iterable[Subscription],
    ) -> list[Exception]:
        errors: list[Exception] = []
        for subscription in subscriptions:
            try:
                subscription.cancel()
            except Exception as exc:
                self._error(
                    "DetectorSim ideal subscription cancellation failed",
                    exc,
                )
                errors.append(exc)
        return errors

    def on_message(self, message: Any) -> None:
        if self._coordinator.stop_event.is_set():
            return
        try:
            message_type = str(message.get_type())
        except Exception:
            message_type = None
        self.enqueue(
            message_type=message_type,
            source_clock_timestamp_s=self.message_boot_time_s(message),
        )

    def enqueue(
        self,
        *,
        message_type: Optional[str] = None,
        source_clock_timestamp_s: Optional[float] = None,
    ) -> bool:
        token = self._coordinator.token
        return self._coordinator.admit_pose(
            token,
            lambda generation: self._build_pose(
                generation,
                message_type,
                source_clock_timestamp_s,
            ),
        )

    def _build_pose(
        self,
        generation: FrameGeneration,
        message_type: Optional[str],
        source_clock_timestamp_s: Optional[float],
    ) -> Optional[IdealPoseSample]:
        if not self._is_armed():
            if self._source_armed:
                self._source_armed = False
                self.reset()
            return None
        self._source_armed = True
        self._associator.note_event(message_type)
        if source_clock_timestamp_s is not None:
            if self._clock.advance(
                source_clock_timestamp_s,
                on_restart=self.reset,
            ) is None:
                return None
        association = self._associator.associate()
        if association is None:
            return None
        if source_clock_timestamp_s is None and not self._associator.require_event_pair:
            self._clock.advance(
                association.attitude_timestamp_s,
                on_restart=self.reset,
            )
        timestamp_s = self._clock.accept_attitude_timestamp(
            association.attitude_timestamp_s,
            on_restart=self.reset,
            record_emitted=False,
        )
        if timestamp_s is None:
            return None
        pose = self._associator.build_sample(
            association,
            timestamp_s=timestamp_s,
            version=generation,
            sample_type=IdealPoseSample,
        )
        if pose is not None:
            self._associator.commit(association)
        return pose

    def reset_state(self) -> None:
        self._associator.reset()
        self._clock.reset()

    def reset(self) -> None:
        with self._coordinator.reset() as resetting:
            if resetting:
                self.reset_state()

    def advance_source_now(self, value: TimestampValue) -> Optional[float]:
        return self._clock.advance(value, on_restart=self.reset)


__all__ = [
    "IDEAL_POSE_MESSAGE_TYPES",
    "IDEAL_SOURCE_MESSAGE_TYPES",
    "IdealPoseSample",
    "IdealPoseSource",
    "copy_attitude",
    "copy_location",
]
