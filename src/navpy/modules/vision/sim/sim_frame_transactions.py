"""Exact frame-transaction adapter over source coordination."""

from __future__ import annotations

from contextlib import AbstractContextManager, contextmanager
from typing import Iterator, Optional, Protocol, Sequence

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.detection_publication_store import PublicationSlot
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.sim_coordinator_ports import (
    FrameTransactionCoordinatorPort,
)
from navpy.modules.vision.sim.sim_frame_types import FrameContext
from navpy.modules.vision.sim.sim_runtime_ports import SourceNameResolver


class PublicationCommitter(Protocol):
    def __call__(
        self,
        slot: PublicationSlot,
        frame: FrameContext,
        targets: Sequence[DetectedObject],
    ) -> bool: ...


class PublicationReleaser(Protocol):
    def __call__(self, slot: PublicationSlot) -> None: ...


class PublicationRejecter(Protocol):
    def __call__(
        self,
        slot: PublicationSlot,
        source_timestamp_s: float,
    ) -> None: ...


class PendingPublicationPort(Protocol):
    @property
    def published(self) -> bool: ...

    def publish(self, targets: list[DetectedObject]) -> bool: ...

    def reject(self) -> None: ...


class DetectionFrameTransactionsPort(Protocol):
    @property
    def generation(self) -> FrameGeneration: ...

    def admit(
        self,
        generation: FrameGeneration,
    ) -> AbstractContextManager[Optional[FrameGeneration]]: ...

    def reserve(
        self,
        frame: FrameContext,
    ) -> AbstractContextManager[Optional[PendingPublicationPort]]: ...

    def commit(
        self,
        frame: FrameContext,
        pending: PendingPublicationPort,
    ) -> AbstractContextManager[Optional[PendingPublicationPort]]: ...


class PendingDetectionPublication:
    """One reserved publication with exact commit/release operations."""

    def __init__(
        self,
        slot: PublicationSlot,
        frame: FrameContext,
        publish: PublicationCommitter,
        abandon: PublicationReleaser,
        reject: PublicationRejecter,
    ) -> None:
        self._slot = slot
        self._frame = frame
        self._publish = publish
        self._abandon = abandon
        self._reject = reject
        self._published = False

    @property
    def published(self) -> bool:
        return self._published

    def publish(self, targets: list[DetectedObject]) -> bool:
        self._published = self._publish(self._slot, self._frame, targets)
        return self._published

    def abandon(self) -> None:
        self._abandon(self._slot)

    def reject(self) -> None:
        self._reject(self._slot, self._frame.timestamp_s)


class DetectionFrameTransactions:
    """Bind source coordination to the detector's frame transaction port."""

    def __init__(
        self,
        coordinator: FrameTransactionCoordinatorPort,
        source_name: SourceNameResolver,
    ) -> None:
        self._coordinator = coordinator
        self._source_name = source_name

    @property
    def generation(self) -> FrameGeneration:
        return self._coordinator.token

    @contextmanager
    def admit(
        self,
        generation: FrameGeneration,
    ) -> Iterator[Optional[FrameGeneration]]:
        with self._coordinator.admission(generation) as active:
            yield active

    @contextmanager
    def reserve(
        self,
        frame: FrameContext,
    ) -> Iterator[Optional[PendingPublicationPort]]:
        slot = self._coordinator.reserve_publication(
            frame.generation,
            frame.timestamp_s,
        )
        if slot is None:
            yield None
            return
        pending = PendingDetectionPublication(
            slot,
            frame,
            self._publish,
            self._coordinator.abandon_publication,
            self._coordinator.reject_publication,
        )
        try:
            yield pending
        finally:
            if not pending.published:
                pending.abandon()

    @contextmanager
    def commit(
        self,
        frame: FrameContext,
        pending: PendingPublicationPort,
    ) -> Iterator[Optional[PendingPublicationPort]]:
        with self._coordinator.commit(frame.generation) as accepted:
            if accepted:
                yield pending
                return
            pending.reject()
            yield None

    def _publish(
        self,
        slot: PublicationSlot,
        frame: FrameContext,
        targets: Sequence[DetectedObject],
    ) -> bool:
        return self._coordinator.publish_detection(
            slot,
            targets,
            source_timestamp_s=frame.timestamp_s,
            source_receipt_timestamp_s=frame.receipt_timestamp_s,
            source_name=self._source_name(targets),
            source_discontinuity=frame.source_discontinuity,
        )


__all__ = [
    "DetectionFrameTransactions",
    "DetectionFrameTransactionsPort",
    "PendingDetectionPublication",
    "PendingPublicationPort",
]
