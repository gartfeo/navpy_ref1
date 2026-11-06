"""The acquisition gates' state on the confirmation lock (HITL step 25).

The navigation tick publishes whether the fresh-lock and orbit gates hold for its POI. Under the
same lock as every status write: while they do not hold, the registry refuses CONFIRMED from any
path (operator answer, timeout or auto-confirm worker, local confirm), and a review under way is
withdrawn (its worker cancelled, its round retired, its status cleared), so a later answer is late
and the POI is asked again once the gates hold. An answer is therefore accepted only while the
gates held on the most recent tick.
"""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt
from navpy.core.review.confirmation_registry_state import ConfirmationRegistryState, ConfirmationStatus
from navpy.core.review.confirmation_round_state import ConfirmationRoundState
from navpy.core.review.confirmation_worker_state import ConfirmationWorkerState


class ConfirmationAcquisitionHold:
    """Publish the gates' state and withdraw a review when they stop holding."""

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

    def publish(self, poi_id: int, held: bool) -> bool:
        """Record the gates' state; True when a review under way was withdrawn."""
        with self._lock:
            self._registry.set_acquisition_held(poi_id, held)
            if held or self._registry.status_by_id(poi_id) is not ConfirmationStatus.CONFIRMING:
                return False
            lease = self._workers.cancel(poi_id)
            self._registry.clear_status_by_id(poi_id)
        if lease is not None:
            self._rounds.retire_worker(lease)
        return True


__all__ = ["ConfirmationAcquisitionHold"]
