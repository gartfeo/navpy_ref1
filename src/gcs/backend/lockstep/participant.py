"""The backend participant's hooks: ``gcs.backend.lockstep.participant:backend``.

Loaded by ``participant_main`` before ``REGISTER``, so this module imports
only the standard library and ``navpy.sim.lockstep`` (bootstrap step 1); the
product is imported in :meth:`BackendHooks.construct`, on the managed
bootstrap thread at the admission tick, which builds the
:class:`~gcs.backend.lockstep.backend_host.BackendHost`.

Tick input order (backend): lifecycle, cancels, requests in id order (each
starts its task on the loop), then the ``lockstep:serial1`` bytes; then
quiescence, with timers due at ``t_k`` after the deliveries. Every input is
delivered from the coordinator thread in the input phase.

Role config (the spawn record's ``config``): ``settings``, the backend's
settings document (written run-locally, the settings store's file), and
optionally ``companions`` for the launcher port (``launcher_port``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.part_wire import Lifecycle, Request, RequestId
from navpy.sim.lockstep.participant_hooks import (
    Outbox,
    OutboxBuffer,
    ParticipantContext,
    ParticipantHooks,
)

if TYPE_CHECKING:
    from gcs.backend.lockstep.backend_host import BackendHost

#: The backend monitor's logical device.
SERIAL1 = "lockstep:serial1"


class BackendHooks(ParticipantHooks):
    def __init__(self, context: ParticipantContext) -> None:
        self._context = context
        self.link = LockstepLink(context.participant.runtime.baton, SERIAL1)
        self.links = {SERIAL1: self.link}
        self.out = OutboxBuffer()
        self.host = None

    def construct(self) -> None:
        from gcs.backend.lockstep.backend_host import BackendHost  # the product, at admission

        self.host = BackendHost(self._context, self.link, self.out)
        self.host.start()

    def _require_host(self, what: str) -> BackendHost:
        if self.host is None:
            raise RuntimeError(f"{what} before the backend was constructed")
        return self.host

    def on_end(self) -> None:
        self._require_host("drain end").end()

    def on_lifecycle(self, entry: Lifecycle) -> None:
        self._require_host("lifecycle record").lifecycle(entry)

    def on_cancel(self, request_id: RequestId) -> None:
        self._require_host("cancel").cancel(request_id)

    def on_request(self, request: Request) -> None:
        self._require_host("request").request(request)

    def finished(self) -> bool:
        return self.host is not None and self.host.finished

    def drain_outbox(self) -> Outbox:
        return self.out.take()


def backend(context: ParticipantContext) -> BackendHooks:
    return BackendHooks(context)


__all__ = ["BackendHooks", "SERIAL1", "backend"]
