"""The acquisition gates' state on the confirmation lock (HITL step 25).

The navigation tick publishes whether the fresh-lock and orbit gates hold for its POI. Under the
same lock as every status write: while they do not hold, the registry refuses CONFIRMED from any
path (operator answer, timeout or auto-confirm worker, local confirm), and a review under way is
withdrawn (its worker cancelled, its round retired, its status cleared), so a later answer is late
and the POI is asked again once the gates hold. A CONFIRMED is revoked the same way (fail closed):
an answer can be judged against the previous tick while this tick's sensors already show the loss,
so a confirmation counts only while the gates have held on every tick since it (until NAV; the
gates are not published in NAV).
"""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt
from navpy.core.review.confirmation_registry_state import ConfirmationRegistryState, ConfirmationStatus
from navpy.core.review.confirmation_round_state import ConfirmationRoundState
from navpy.core.review.confirmation_worker_state import ConfirmationWorkerState


_WITHDRAWN = (ConfirmationStatus.CONFIRMING, ConfirmationStatus.CONFIRMED)


class ConfirmationAcquisitionHold:
    """Publish the gates' state; on a loss withdraw a review or revoke a confirmation."""

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
        """Record the gates' state; True when a review or a confirmation was withdrawn."""
        with self._lock:
            self._registry.set_acquisition_held(poi_id, held)
            status = self._registry.status_by_id(poi_id)
            if held or status not in _WITHDRAWN:
                return False
            lease = self._workers.cancel(poi_id) if status is ConfirmationStatus.CONFIRMING else None
            self._registry.clear_status_by_id(poi_id)
        if lease is not None:
            self._rounds.retire_worker(lease)
        return True


__all__ = ["ConfirmationAcquisitionHold"]
