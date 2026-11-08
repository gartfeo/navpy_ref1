"""The navigation command fence at the MAVLink send boundary (docs/design/navpy_recovery.md, step 2).

Guided targets, attitude targets and NavPy's own GUIDED request are admitted only while the fence is open and the
command was formed in the current generation. Recovery (the NAV loop's hold, the watchdog) closes it; a closed fence
or an old generation drops the send and counts it. Mode requests other than GUIDED, heartbeats, parameters and
missions are never fenced, so the hold's LOITER/AUTO requests still go out.

Contract (MavTransport.fenced): a write is admitted only while holding the send lock and, inside it, the fence lock,
with the fence open and the generation current. close() takes the fence lock only, never the send lock or a worker's
lock, so a hung sender cannot block it. A write admitted before close() is in flight (at most one: the send lock
serializes writes); none is admitted after.

A command states its generation with ``formed_in(generation)`` on the thread that forms and sends it. A send with no
stated generation is judged by open/closed alone. Unlike the HITL LinkFence (app/terminal_handoff.py), which drops
every write for good, this fence drops only navigation commands and can reopen with a new generation.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import AbstractContextManager, contextmanager

from navpy.core.kernel import runtime_time as rt


class MavCommandFence:
    def __init__(self) -> None:
        self._lock = rt.Lock()   # guards the state below only; never held across a write
        self._generation = 0
        self._open = True
        self._latched = False   # closed for good: reopen() no longer opens
        self._dropped = 0
        self._first_drop_logged_for: int | None = None
        self._stated: dict[int, int] = {}   # thread ident -> stated generation

    @property
    def generation(self) -> int:
        with self._lock:
            return self._generation

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._open

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    def close(self) -> int:
        """Drop every later navigation command, including ones already formed; returns the closed generation."""
        with self._lock:
            if self._open:
                self._open = False
                self._generation += 1
            return self._generation

    def latch_closed(self) -> int:
        """Close for good (a latched component failure, app/runtime_escalation.py): a reopen() racing this, or after
        it, leaves the fence closed. Both take the fence lock, so either order ends closed."""
        with self._lock:
            self._latched = True
            if self._open:
                self._open = False
                self._generation += 1
            return self._generation

    def reopen(self) -> int:
        """Admit commands again under a new generation; commands formed before stay dropped. Not once latched."""
        with self._lock:
            if self._latched:
                return self._generation
            self._generation += 1
            self._open = True
            return self._generation

    @contextmanager
    def formed_in(self, generation: int) -> Iterator[None]:
        """Sends on this thread inside the block carry ``generation``."""
        thread = rt.get_ident()
        with self._lock:
            previous = self._stated.get(thread)
            self._stated[thread] = int(generation)
        try:
            yield
        finally:
            with self._lock:
                if previous is None:
                    self._stated.pop(thread, None)
                else:
                    self._stated[thread] = previous

    def formed_now(self) -> AbstractContextManager[None]:
        """Sends on this thread inside the block carry the generation current at entry: a close meanwhile drops them."""
        return self.formed_in(self.generation)

    def admit(self) -> bool:
        """Called by MavTransport while it holds the send lock: True admits this write, False drops and counts it."""
        thread = rt.get_ident()
        with self._lock:
            stated = self._stated.get(thread)
            if self._open and (stated is None or stated == self._generation):
                return True
            self._dropped += 1
            return False

    def first_drop_of_generation(self) -> bool:
        """True once per generation, so a 50 Hz command stream logs one line, not one per drop."""
        with self._lock:
            if self._first_drop_logged_for == self._generation:
                return False
            self._first_drop_logged_for = self._generation
            return True


__all__ = ["MavCommandFence"]
