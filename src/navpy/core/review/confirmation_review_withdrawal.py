"""Withdraw a POI review still under way (HITL step 25)."""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.review.confirmation_registry_state import ConfirmationRegistryState, ConfirmationStatus
from navpy.core.review.confirmation_round_state import ConfirmationRoundState
from navpy.core.review.confirmation_worker_state import ConfirmationWorkerState


class ConfirmationReviewWithdrawal:
    """Cancel a CONFIRMING review's worker and round and clear its status.

    The cancelled worker can no longer set a status (its auto-confirm or timeout path), an answer
    to the retired round is late, and with the status cleared the POI can be asked again.
    """

    def __init__(
        self,
        lock: rt.RLock,
        registry: ConfirmationRegistryState,
        workers: ConfirmationWorkerState,
        rounds: ConfirmationRoundState,
    ) -> None:
        self._lock = lock
        self._registry = registry
        self._workers = workers
        self._rounds = rounds

    def withdraw(self, poi: DetectedObject) -> bool:
        """True when a review under way was withdrawn."""
        poi_id = get_poi_task_id(poi)
        if poi_id is None:
            return False
        with self._lock:
            if self._registry.status_by_id(poi_id) is not ConfirmationStatus.CONFIRMING:
                return False
            lease = self._workers.cancel(poi_id)
            self._registry.clear_status(poi)
        if lease is not None:
            self._rounds.retire_worker(lease)
        return True


__all__ = ["ConfirmationReviewWithdrawal"]
