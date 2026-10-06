"""Confirmation registry and round-state composition."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Optional

from navpy.modules.nav.confirmation_round_state import ConfirmationRoundState
from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationResponseKind,
    ConfirmationRound,
    ConfirmationWorkerLease,
)
from navpy.modules.nav.confirmation_worker_state import ConfirmationWorkerState
from navpy.modules.nav.confirmation_registry_state import ConfirmationRegistryState, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


class ConfirmationManagerState:
    """Compose POI status, worker leases, and exact confirmation rounds."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.registry = ConfirmationRegistryState(self._lock)
        self._workers = ConfirmationWorkerState(self._lock)
        self._rounds = ConfirmationRoundState(
            self._lock,
            self.registry,
            self._workers,
        )

    def start_review(
        self,
        poi: DetectedObject,
        event_factory: Callable[[], threading.Event],
    ) -> ConfirmationWorkerLease:
        poi_id = get_poi_task_id(poi)
        with self._lock:
            if poi_id is not None:
                self._rounds.start_review(poi_id)
                self.registry.set_status(poi_id, ConfirmationStatus.CONFIRMING)
            return self._workers.start(poi_id, event_factory)

    def worker_is_current(self, lease: ConfirmationWorkerLease) -> bool:
        return self._workers.is_current(lease)

    def finish_worker(self, lease: ConfirmationWorkerLease) -> None:
        self._workers.finish(lease)
        self._rounds.retire_worker(lease)

    def set_worker_status(
        self,
        lease: ConfirmationWorkerLease,
        status: ConfirmationStatus,
    ) -> bool:
        with self._lock:
            if lease.poi_id is None or not self._workers.is_current(lease):
                return False
            self.registry.set_status(lease.poi_id, status)
            return True

    def begin_round(
        self,
        poi: DetectedObject,
        lease: ConfirmationWorkerLease,
        event_factory: Callable[[], threading.Event],
        request_ref: Optional[ConfirmationRequestRef] = None,
    ) -> Optional[ConfirmationRound]:
        return self._rounds.begin(poi, lease, event_factory, request_ref)

    def complete_round(
        self,
        confirmation: ConfirmationRound,
        status: ConfirmationStatus,
    ) -> bool:
        return self._rounds.complete(confirmation, status)

    def retire_round(self, confirmation: ConfirmationRound) -> bool:
        return self._rounds.retire(confirmation)

    def send_if_current(
        self,
        confirmation: ConfirmationRound,
        send: Callable[[], None],
    ) -> bool:
        return self._rounds.send_if_current(confirmation, send)

    def pending_poi(self, poi_id: int) -> Optional[DetectedObject]:
        return self._rounds.pending_poi(poi_id)

    def send_pending_if_current(
        self,
        poi_id: int,
        poi: DetectedObject,
        request_ref: Optional[ConfirmationRequestRef],
        send: Callable[[], None],
    ) -> bool:
        return self._rounds.send_pending_if_current(
            poi_id,
            poi,
            request_ref,
            send,
        )

    def resolve_response(
        self,
        poi_id: int,
        is_confirmed: bool,
        response_ref: Optional[ConfirmationRequestRef] = None,
    ) -> ConfirmationResponseKind:
        return self._rounds.resolve_response(poi_id, is_confirmed, response_ref)

    def reset(self) -> None:
        with self._lock:
            self._workers.reset()
            self.registry.reset()
            self._rounds.clear_resolved()
            rounds = self._rounds.snapshot()
        self._rounds.retire_many(rounds)

    def pending_events(self) -> dict[int, threading.Event]:
        return self._rounds.pending_events()

    @property
    def active_worker_count(self) -> int:
        return self._workers.count


__all__ = [
    "ConfirmationResponseKind",
    "ConfirmationRequestRef",
    "ConfirmationRound",
    "ConfirmationWorkerLease",
    "ConfirmationManagerState",
    "ConfirmationRegistryState",
    "ConfirmationStatus",
]
