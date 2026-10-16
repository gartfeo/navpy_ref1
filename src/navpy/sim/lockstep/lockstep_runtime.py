"""LockstepRuntime: the ``rt`` port's three facets on a :class:`BatonRuntime`.

- **Clock:** ``monotonic`` and ``perf_counter`` are ``t_k``; ``time`` is the
  run's epoch plus ``t_k``; ``now`` is that instant as a ``datetime``; ``sleep``
  parks to the first tick at or after its float deadline. The epoch is a run
  input, never the host clock. Both are integer microseconds, so ``time`` and
  ``now`` are exact.
- **Threads:** :class:`~navpy.sim.lockstep.baton_threads.LockstepThreads`.
- **Entropy:** one generator per participant, ``Random(seed, ordinal)``
  (:func:`participant_seed`); ``uuid4`` and ``token_urlsafe`` draw from it.

``now(None)`` is a naive UTC ``datetime``: the host time zone is not a run
input, so local time would differ host to host.
"""

from __future__ import annotations

import base64
import hashlib
import random
import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone, tzinfo
from types import SimpleNamespace

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_sync import sleep
from navpy.sim.lockstep.baton_threads import LockstepThreads

_UNIX_EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)
#: ``secrets.token_urlsafe``'s default size (``secrets.DEFAULT_ENTROPY``).
DEFAULT_TOKEN_BYTES = 32
#: Domain tag of the per-participant seed derivation; a new derivation gets a new tag.
ENTROPY_DOMAIN = "navpy.lockstep.entropy.v1"


class LockstepClock:
    def __init__(self, baton: BatonRuntime, epoch_us: int) -> None:
        self._baton = baton
        self._epoch_us = int(epoch_us)
        self._start_us = baton.now_us

    def since_start_us(self) -> int:
        """Sim microseconds since the runtime was created (``relativeCreated``)."""
        return self._baton.now_us - self._start_us

    def unix_us(self) -> int:
        """Epoch plus ``t_k``, in integer microseconds."""
        return self._epoch_us + self._baton.now_us

    def monotonic(self) -> float:
        return self._baton.monotonic()

    def time(self) -> float:
        return self.unix_us() / 1_000_000

    def perf_counter(self) -> float:
        return self._baton.monotonic()

    def thread_time(self) -> float:
        # No host CPU time in a lockstep artifact: it would differ run to run.
        # The sim clock keeps diagnostic CPU columns deterministic (equal to wall).
        return self._baton.monotonic()

    def now(self, tz: tzinfo | None = None) -> datetime:
        instant = _UNIX_EPOCH + timedelta(microseconds=self.unix_us())
        return instant.replace(tzinfo=None) if tz is None else instant.astimezone(tz)

    def sleep(self, seconds: float) -> None:
        sleep(self._baton, seconds)


def participant_seed(seed: int, ordinal: int) -> int:
    """SHA-256 of ``"<domain>:<seed>:<ordinal>"`` as a 256-bit integer.

    One run seed gives every participant its own independent stream, and the
    same seed and ordinal always give the same stream.
    """
    material = f"{ENTROPY_DOMAIN}:{int(seed)}:{int(ordinal)}".encode("ascii")
    return int.from_bytes(hashlib.sha256(material).digest(), "big")


class LockstepEntropy:
    def __init__(self, seed: int, ordinal: int) -> None:
        self._random = random.Random(participant_seed(seed, ordinal))

    def rng(self) -> random.Random:
        return self._random

    def uuid4(self) -> uuid.UUID:
        return uuid.UUID(int=self._random.getrandbits(128), version=4)

    def token_urlsafe(self, nbytes: int | None = None) -> str:
        raw = self._random.randbytes(DEFAULT_TOKEN_BYTES if nbytes is None else nbytes)
        return base64.urlsafe_b64encode(raw).rstrip(b"=").decode("ascii")


@dataclass(frozen=True)
class LockstepRuntime(rt.RuntimeTime):
    """A participant's ``RuntimeTime`` and the baton its facets run on."""

    baton: BatonRuntime

    @classmethod
    def create(cls, baton: BatonRuntime, *, epoch_us: int, seed: int, ordinal: int) -> "LockstepRuntime":
        return cls(
            clock=LockstepClock(baton, epoch_us),
            threads=LockstepThreads(baton),
            entropy=LockstepEntropy(seed, ordinal),
            baton=baton,
        )


#: The ``rt`` module functions, by the facet that serves each.
_PORT_FUNCTIONS = {
    "clock": ("monotonic", "time", "perf_counter", "thread_time", "now", "sleep"),
    "threads": (
        "Lock", "RLock", "Condition", "Event", "Semaphore", "BoundedSemaphore",
        "Thread", "Timer", "current_thread", "get_ident", "Queue", "shielded", "detached",
    ),
    "entropy": ("rng", "uuid4", "token_urlsafe"),
}


def module_port(runtime: rt.RuntimeTime) -> SimpleNamespace:
    """The ``rt`` module surface served by ``runtime``.

    For a process that stays on the wall runtime (the coordinator, whose relay
    is host IO) but runs some modules on a baton: their ``rt`` is set to this.
    """
    functions = {
        name: getattr(getattr(runtime, facet), name)
        for facet, names in _PORT_FUNCTIONS.items()
        for name in names
    }
    return SimpleNamespace(Empty=rt.Empty, **functions)


__all__ = [
    "DEFAULT_TOKEN_BYTES",
    "ENTROPY_DOMAIN",
    "LockstepClock",
    "LockstepEntropy",
    "LockstepRuntime",
    "module_port",
    "participant_seed",
]
