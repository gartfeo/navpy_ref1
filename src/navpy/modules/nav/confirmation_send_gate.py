"""Exact-round outbound fencing for request and confirmation media sends."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Optional

from navpy.modules.nav.confirmation_response_state import matches_round
from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationRound,
)
from navpy.modules.nav.confirmation_worker_state import ConfirmationWorkerState
from navpy.modules.vision.models.detect_data import DetectedObject


class ConfirmationSendGate:
    def __init__(
        self,
        lock: threading.RLock,
        workers: ConfirmationWorkerState,
        rounds: dict[int, ConfirmationRound],
    ) -> None:
        self._lock = lock
        self._workers = workers
        self._rounds = rounds

    def send_round(
        self,
        confirmation: ConfirmationRound,
        send: Callable[[], None],
    ) -> bool:
        def transaction() -> bool:
            with self._lock:
                if (
                    self._rounds.get(confirmation.target_id) is not confirmation
                    or confirmation.response_event.is_set()
                    or not self._workers.is_current(confirmation.worker)
                ):
                    return False
            send()
            return True

        return confirmation.outbound.run(transaction)

    def send_pending(
        self,
        target_id: int,
        target: DetectedObject,
        request_ref: Optional[ConfirmationRequestRef],
        send: Callable[[], None],
    ) -> bool:
        with self._lock:
            confirmation = self._rounds.get(target_id)
        if confirmation is None:
            return False

        def transaction() -> bool:
            with self._lock:
                if (
                    self._rounds.get(target_id) is not confirmation
                    or confirmation.target is not target
                    or confirmation.response_event.is_set()
                    or not self._workers.is_current(confirmation.worker)
                    or not matches_round(confirmation, request_ref)
                ):
                    return False
            send()
            return True

        return confirmation.outbound.run(transaction)


__all__ = ["ConfirmationSendGate"]
