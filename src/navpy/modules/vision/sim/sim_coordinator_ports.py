"""Segregated coordinator contracts for simulator leaves."""

from __future__ import annotations

import threading
from contextlib import AbstractContextManager
from typing import Any, Optional, Protocol, Sequence, TypeVar

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.detection_publication_store import PublicationSlot
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vehicle.message_subscriptions import Subscription


PoseT = TypeVar("PoseT")


class PoseBuilder(Protocol[PoseT]):
    def __call__(self, generation: FrameGeneration) -> Optional[PoseT]: ...


class OpaqueMessageHandler(Protocol):
    def __call__(self, message: Any) -> None: ...


class MessageSubscriber(Protocol):
    def __call__(
        self,
        message_type: str,
        callback: OpaqueMessageHandler,
    ) -> Subscription: ...


class ArmedReader(Protocol):
    def __call__(self) -> bool: ...


class PoseSourceCoordinatorPort(Protocol):
    @property
    def token(self) -> FrameGeneration: ...

    @property
    def stop_event(self) -> threading.Event: ...

    def admit_pose(
        self,
        token: FrameGeneration,
        build_pose: PoseBuilder[PoseT],
    ) -> bool: ...

    def reset(self) -> AbstractContextManager[bool]: ...


class WorkerCoordinatorPort(Protocol[PoseT]):
    @property
    def stop_event(self) -> threading.Event: ...

    def wait_and_pop_pose(self) -> Optional[PoseT]: ...


class LifecycleCoordinatorPort(Protocol):
    def reset(self) -> AbstractContextManager[bool]: ...

    def stop(self) -> None: ...


class FrameTransactionCoordinatorPort(Protocol):
    @property
    def token(self) -> FrameGeneration: ...

    def admission(
        self,
        token: FrameGeneration,
    ) -> AbstractContextManager[Optional[FrameGeneration]]: ...

    def reserve_publication(
        self,
        token: FrameGeneration,
        source_timestamp_s: float,
    ) -> Optional[PublicationSlot]: ...

    def commit(self, token: FrameGeneration) -> AbstractContextManager[bool]: ...

    def abandon_publication(self, slot: PublicationSlot) -> None: ...

    def reject_publication(
        self,
        slot: PublicationSlot,
        source_timestamp_s: float,
    ) -> None: ...

    def publish_detection(
        self,
        slot: PublicationSlot,
        targets: Sequence[DetectedObject],
        *,
        source_timestamp_s: float,
        source_receipt_timestamp_s: Optional[float],
        source_name: Optional[str],
        source_discontinuity: Optional[bool],
    ) -> bool: ...


__all__ = [
    "FrameTransactionCoordinatorPort",
    "LifecycleCoordinatorPort",
    "ArmedReader",
    "MessageSubscriber",
    "OpaqueMessageHandler",
    "PoseBuilder",
    "PoseSourceCoordinatorPort",
    "WorkerCoordinatorPort",
]
