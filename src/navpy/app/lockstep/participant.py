"""NavPy's participant hooks: ``navpy.app.lockstep.participant:navpy``.

Loaded by ``participant_main`` before ``REGISTER``, so this module imports
only the standard library and ``navpy.sim.lockstep`` (bootstrap step 1); the
product is imported in :meth:`NavpyHooks.construct`, on the managed
bootstrap thread at the admission tick, which builds the
:class:`~navpy.app.lockstep.navpy_host.NavpyHost` and runs NavPy's
composition and run loop to their end (``lockstep_slice.md``, "NavPy
participant").

Inputs (the coordinator thread, the tick's input phase): a lifecycle
``stop`` and the drain ``end`` latch the run loop's stop request; the bytes
of its device (``lockstep:serial0.<sysid>``) go to the link. Either may arrive before the
bootstrap thread built the host: they are held and applied when it is.

Role config (the spawn record's ``config``): ``device``, its logical device,
and ``argv``, NavPy's command line
(``gcs.backend.navpy_process_launch.lockstep_navpy_argv``: the same device
and ``--stop-on-lifecycle``).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.part_wire import Lifecycle, RequestId
from navpy.sim.lockstep.participant_hooks import (
    Outbox,
    OutboxBuffer,
    ParticipantContext,
    ParticipantHooks,
)
from navpy.sim.lockstep.pose_recording import enable_pose_cadence

if TYPE_CHECKING:
    from navpy.app.lockstep.navpy_host import NavpyHost

class NavpyHooks(ParticipantHooks):
    def __init__(self, context: ParticipantContext) -> None:
        self._context = context
        self.link = LockstepLink(context.participant.runtime.baton, context.device)
        self.links = {context.device: self.link}
        self.out = OutboxBuffer()
        self.host: NavpyHost | None = None
        self._stop_id: RequestId | None = None
        self._ended = False
        self._finished = False

    def construct(self) -> None:
        enable_pose_cadence(self._context.config)  # read once, at the product's import
        from navpy.app.lockstep.navpy_host import NavpyHost  # the product, at admission

        try:
            self.host = NavpyHost(self._context, self.link, self.out,
                                  held_stop_id=self._stop_id, held_end=self._ended)
            self.host.run()
        finally:
            self._finished = True

    def on_lifecycle(self, entry: Lifecycle) -> None:
        if entry.kind != "stop":
            raise ValueError(f"NavPy got a {entry.kind!r} lifecycle record")
        if self._stop_id is None:
            self._stop_id = entry.request
        if self.host is not None:
            self.host.stop.deliver_stop(entry.request)

    def on_end(self) -> None:
        """Drain: stop the run loop (only if no participant stop ended it first)."""
        self._ended = True
        if self.host is not None:
            self.host.stop.deliver_end()

    def finished(self) -> bool:
        return self._finished

    def flushes(self) -> Sequence[Callable[[], object]]:
        if self.host is None:
            return (self._never_built,)
        return self.host.flushes()

    @staticmethod
    def _never_built() -> None:
        raise RuntimeError("NavPy ended before its host was built")

    def drain_outbox(self) -> Outbox:
        return self.out.take()


def navpy(context: ParticipantContext) -> NavpyHooks:
    return NavpyHooks(context)


__all__ = ["NavpyHooks", "navpy"]
