"""The participant side of a lockstep run: bootstrap, admission and drain flush.

Bootstrap (``lockstep_slice.md``, "Participant bootstrap"), in order:

1. The entry imports only the standard library,
   ``navpy.core.kernel.runtime_time`` and ``navpy.sim.lockstep``; this
   package imports no product module, so nothing has used ``rt`` yet.
2. :func:`bootstrap` binds the :class:`LockstepRuntime` and installs the
   logging stamp and the thread guard (and the native-thread defaults).

Steps 3+ (``REGISTER``, the admission tick, the wire) are the participant
loop's: at the admission tick it calls :meth:`Participant.admit`, so the
product is imported and constructed by a managed bootstrap thread, under the
baton. At its end :meth:`Participant.drain_flush` runs the product's flushes
on the baton and reports the ``flushed`` its ending ``DONE`` carries.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import BatonRuntime, ManagedThread
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.log_stamp import install_log_stamp
from navpy.sim.lockstep.thread_guard import apply_native_thread_defaults, install_thread_guard

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class ParticipantSpec:
    """What the spawn record gives a participant: all run inputs, no host values."""

    seed: int
    ordinal: int
    generation: int
    #: Unix time of sim time 0, in microseconds (the run manifest's epoch).
    epoch_us: int


@dataclass(frozen=True)
class LockstepIdentity:
    """A participant's process identity: ``process_pid`` is its ordinal.

    Callable as ``runtime_ready``'s process identity
    (``runtime_ready.set_process_identity``).
    """

    ordinal: int
    generation: int

    def __post_init__(self) -> None:
        if self.ordinal < 1:
            raise ValueError("participant ordinals start at 1 (0 is the operator)")

    def __call__(self) -> dict[str, object]:
        return {"ordinal": self.ordinal, "generation": self.generation, "process_pid": self.ordinal}


class Participant:
    def __init__(
        self,
        runtime: LockstepRuntime,
        identity: LockstepIdentity,
        uninstalls: Sequence[Callable[[], None]],
    ) -> None:
        self.runtime = runtime
        self.identity = identity
        self._uninstalls = tuple(uninstalls)

    def admit(self, construct: Callable[[], object]) -> ManagedThread:
        """Import and construct the product on a managed bootstrap thread.

        Called by the coordinator thread before the admission tick; the thread
        first runs in that tick's quiescence.
        """
        return self.runtime.baton.spawn(construct, name="bootstrap")

    def drain_flush(self, flushes: Sequence[Callable[[], object]]) -> bool:
        """Run every flush in order on the baton; ``flushed`` only if all succeed.

        NavPy's are its log worker (``log_output_latch.close_strictly``: the
        FIFO drained as cleanup does, then a raise if any line failed to reach
        its file) and ``runtime_stop.flush_pose_cadence`` (the stop ack's strict
        pose-cadence flush). A failure is logged and the rest still run.
        """
        self.runtime.baton.current()  # the end runs on a managed thread
        flushed = True
        for flush in flushes:
            try:
                flush()
            except Exception:
                _log.exception("lockstep drain flush failed: %r", flush)
                flushed = False
        return flushed

    def uninstall(self) -> None:
        """Remove the stamp and the guard (tests; the binding stays: it is once only)."""
        for uninstall in reversed(self._uninstalls):
            uninstall()


def bootstrap(spec: ParticipantSpec) -> Participant:
    """Bootstrap step 2: bind the runtime, then the stamp and the guard.

    The calling thread becomes the baton's coordinator.
    """
    identity = LockstepIdentity(spec.ordinal, spec.generation)
    baton = BatonRuntime()
    runtime = LockstepRuntime.create(
        baton, epoch_us=spec.epoch_us, seed=spec.seed, ordinal=spec.ordinal,
    )
    rt.bind(runtime)
    uninstalls = [install_log_stamp(runtime.clock), install_thread_guard(baton)]
    apply_native_thread_defaults()
    return Participant(runtime, identity, uninstalls)


__all__ = ["LockstepIdentity", "Participant", "ParticipantSpec", "bootstrap"]
