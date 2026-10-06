"""Retryable teardown transaction for one navigation network session."""

from __future__ import annotations

from collections.abc import Callable

from navpy.exception_groups import ExceptionGroup
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.nav.confirm_override_listener import ConfirmOverrideListener
from navpy.modules.nav.failure_health import ComponentFailureLatch
from navpy.modules.nav.peer_poi_dispatch import PeerPoiDispatchWorker
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.swarm.task_actor import TaskActor


def disconnect_nav_network_session(
    worker: PeerPoiDispatchWorker | None,
    actor: TaskActor | None,
    network: NetworkAbc | None,
    override_listener: ConfirmOverrideListener | None,
    confirmation_manager: ConfirmationManager,
    health: ComponentFailureLatch,
) -> None:
    """Retire effects before their network and POI-manager dependencies."""
    errors: list[Exception] = []

    def attempt(action: Callable[[], None]) -> None:
        try:
            action()
        except Exception as error:
            errors.append(error)

    if worker is not None:
        try:
            worker.stop()
        except Exception as error:
            # The worker still owns POI-manager and task-actor ports. Keep
            # the full session installed for a bounded cleanup retry.
            health.harvest(worker)
            raise ExceptionGroup(
                "Navigation network cleanup failed",
                [error],
            ) from None
        health.harvest(worker)
    if network is not None:
        for listener in (actor, confirmation_manager, override_listener):
            if listener is not None:
                attempt(
                    lambda listener=listener: network.remove_listener(listener)
                )
    attempt(lambda: confirmation_manager.set_network(None))
    if actor is not None:
        # Inbound delivery is detached before checkout/shutdown, while the
        # actor still owns the sender for its intentional final message.
        attempt(actor.shutdown)
        health.harvest(actor)
    if errors:
        raise ExceptionGroup("Navigation network cleanup failed", errors)


__all__ = ["disconnect_nav_network_session"]
