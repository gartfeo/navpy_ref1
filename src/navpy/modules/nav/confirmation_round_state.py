"""Exact confirmation-round registry and outbound transaction coordination."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Optional

from navpy.modules.nav.confirmation_response_state import (
    ConfirmationResponseLedger,
    matches_round,
    response_for_current_round,
)
from navpy.modules.nav.confirmation_send_gate import ConfirmationSendGate
from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationResponseKind,
    ConfirmationRound,
    ConfirmationWorkerLease,
)
from navpy.modules.nav.confirmation_worker_state import ConfirmationWorkerState
from navpy.modules.nav.confirmation_registry_state import ConfirmationRegistryState, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject


class ConfirmationRoundState:
    """Own exact-round identity and outbound-to-shared lock ordering."""

    def __init__(
        self,
        lock: threading.RLock,
        registry: ConfirmationRegistryState,
        workers: ConfirmationWorkerState,
    ) -> None:
        self._lock = lock
        self._registry = registry
        self._workers = workers
        self._rounds: dict[int, ConfirmationRound] = {}
        self._responses = ConfirmationResponseLedger(registry)
        self._sends = ConfirmationSendGate(lock, workers, self._rounds)

    def start_review(self, poi_id: int) -> None:
        with self._lock:
            self._responses.start_review(poi_id)

    def begin(
        self,
        poi: DetectedObject,
        lease: ConfirmationWorkerLease,
        event_factory: Callable[[], threading.Event],
        request_ref: Optional[ConfirmationRequestRef] = None,
    ) -> Optional[ConfirmationRound]:
        poi_id = lease.poi_id
        if poi_id is None:
            return None
        confirmation: Optional[ConfirmationRound] = None
        while True:
            with self._lock:
                if not self._workers.is_current(lease):
                    return None
                if confirmation is None:
                    confirmation = ConfirmationRound(
                        poi_id,
                        lease.generation,
                        event_factory(),
                        poi,
                        lease,
                        request_ref,
                        self._responses.legacy_allowed(poi_id),
                    )
                previous = self._rounds.get(poi_id)
                if previous is None:
                    self._rounds[poi_id] = confirmation
                    self._responses.install(poi_id)
                    return confirmation

            def replace_previous() -> Optional[bool]:
                with self._lock:
                    if not self._workers.is_current(lease):
                        return False
                    if self._rounds.get(poi_id) is not previous:
                        return None
                    self._rounds[poi_id] = confirmation
                    self._responses.install(poi_id)
                previous.response_event.set()
                return True

            replaced = previous.outbound.run(replace_previous)
            if replaced is True:
                return confirmation
            if replaced is False:
                return None

    def retire_worker(self, lease: ConfirmationWorkerLease) -> None:
        with self._lock:
            confirmation = (
                None if lease.poi_id is None else self._rounds.get(lease.poi_id)
            )
            if confirmation is not None and confirmation.worker is not lease:
                confirmation = None
        if confirmation is not None:
            self._retire_exact(confirmation)

    def complete(
        self,
        confirmation: ConfirmationRound,
        status: ConfirmationStatus,
    ) -> bool:
        def complete_exact() -> bool:
            with self._lock:
                if self._rounds.get(confirmation.poi_id) is not confirmation:
                    return False
                if not self._workers.is_current(confirmation.worker):
                    return False
                self._rounds.pop(confirmation.poi_id)
                self._registry.set_status(confirmation.poi_id, status)
                self._responses.remember_completion(confirmation, status)
            confirmation.response_event.set()
            return True

        return confirmation.outbound.run(complete_exact)

    def retire(self, confirmation: ConfirmationRound) -> bool:
        return self._retire_exact(confirmation)

    def send_if_current(
        self,
        confirmation: ConfirmationRound,
        send: Callable[[], None],
    ) -> bool:
        return self._sends.send_round(confirmation, send)

    def pending_poi(self, poi_id: int) -> Optional[DetectedObject]:
        with self._lock:
            confirmation = self._rounds.get(poi_id)
            return confirmation.poi if confirmation is not None else None

    def send_pending_if_current(
        self,
        poi_id: int,
        poi: DetectedObject,
        request_ref: Optional[ConfirmationRequestRef],
        send: Callable[[], None],
    ) -> bool:
        return self._sends.send_pending(poi_id, poi, request_ref, send)

    def resolve_response(
        self,
        poi_id: int,
        is_confirmed: bool,
        response_ref: Optional[ConfirmationRequestRef] = None,
    ) -> ConfirmationResponseKind:
        while True:
            with self._lock:
                confirmation = self._rounds.get(poi_id)
                if confirmation is None:
                    return self._responses.resolve_without_round(
                        poi_id,
                        is_confirmed,
                        response_ref,
                    )

            def resolve_exact() -> Optional[ConfirmationResponseKind]:
                with self._lock:
                    if self._rounds.get(poi_id) is not confirmation:
                        return None
                    if not matches_round(confirmation, response_ref):
                        return ConfirmationResponseKind.LATE_OR_DUPLICATE
                    self._rounds.pop(poi_id)
                    result = (
                        response_for_current_round(
                            self._registry,
                            poi_id,
                            is_confirmed,
                        )
                        if self._workers.is_current(confirmation.worker)
                        else ConfirmationResponseKind.LATE_OR_DUPLICATE
                    )
                    self._responses.record_result(confirmation, result)
                confirmation.response_event.set()
                return result

            result = confirmation.outbound.run(resolve_exact)
            if result is not None:
                return result

    def snapshot(self) -> tuple[ConfirmationRound, ...]:
        with self._lock:
            return tuple(self._rounds.values())

    def clear_resolved(self) -> None:
        with self._lock:
            self._responses.clear()

    def retire_many(self, confirmations: tuple[ConfirmationRound, ...]) -> None:
        for confirmation in confirmations:
            self._retire_exact(confirmation)

    def pending_events(self) -> dict[int, threading.Event]:
        with self._lock:
            return {
                poi_id: confirmation.response_event
                for poi_id, confirmation in self._rounds.items()
            }

    def _retire_exact(self, confirmation: ConfirmationRound) -> bool:
        def retire() -> bool:
            with self._lock:
                if self._rounds.get(confirmation.poi_id) is not confirmation:
                    return False
                self._rounds.pop(confirmation.poi_id)
            confirmation.response_event.set()
            return True

        return confirmation.outbound.run(retire)


__all__ = ["ConfirmationRoundState"]
