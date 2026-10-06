"""The navigation worker's own ledger: iterations, and the commands they issued.

Its own object, and its own module, because the worker thread's evidence is
keyed differently from every other row in the trace. Two consequences that
matter more than the code:

**An iteration index is NOT an autopilot slot index.** ``NavigationCommandWorker``
counts its own loop passes; it skips a missed deadline outright
(``_next_fixed_deadline``, "Never replay a missed slot"), so iteration j and
autopilot slot j diverge the first time the host is late. Nothing here may be
subtracted from a slot number. Building the common grid that would let it be is
Landing 2.

**An iteration has no epoch.** Every other row is attributed to an activation
epoch, because the source knows which leg it is in; the worker does not.
Entries here are keyed by iteration alone, which is why they are held separately
instead of being stamped with a leg they cannot vouch for.

Bounded like the row log, and for the same reason: a runaway must latch
``overflowed`` rather than grow. Not internally lock-free -- it holds one lock,
and it does NOT nest with the trace's row lock: ``record_output`` reads the
iteration and releases this lock BEFORE acquiring that one, so the two are
never held together. Each note is ONE critical section (``determinism_seal``),
so its whole entry is built under the lock: ``int(iteration)`` runs there, and
so does the command digest, which reads attributes off the command object. A
supplied ``__int__`` or a command's getter could acquire anything. Production
passes a plain int and a ``CalcData``, whose attributes are plain, so nothing
calls out in practice.

A post-leg reader takes the whole ledger as one ``CommandCapture``, never as
properties read one lock acquisition at a time.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from navpy.modules.vision.sim.determinism_seal import (
    SealState,
    record_under,
    seal_under,
)
from navpy.modules.vision.sim.determinism_slots import command_digest


# One entry per iteration that issued a command:
#     (iteration, digest)
# and the current iteration is held separately, so a dispatch can stamp its
# output row with the iteration it is running in. While tracing, each pass is
# also sampled, one per iteration:
#     (iteration, observed, admitted)
# the ATTITUDE ledger's cutoff when the pass began (``PassObserver``, D3).
COMMAND_ENTRY_ITERATION = 0
COMMAND_ENTRY_DIGEST = 1
# A digest of exactly these bytes means the iteration RAISED. Distinct from
# None, which means it ran work and returned no command: two runs that differ
# only in which one crashed are not the same run.
COMMAND_RAISED = b"raised"
# And these mean a command object WAS returned but could not be digested.
# Also distinct from None: "the law produced nothing" and "the recorder
# could not read what the law produced" are different facts about a run.
COMMAND_UNREADABLE = b"unreadable"


def _digest_of(command: Any, raised: bool) -> bytes | None:
    """Three outcomes, three answers -- None means "produced nothing"."""
    if raised:
        return COMMAND_RAISED
    if command is None:
        return None
    digest = command_digest(command)
    return COMMAND_UNREADABLE if digest is None else digest


@dataclass(frozen=True)
class CommandCapture:
    """The whole ledger as of ONE acquisition of its lock, and immutable.

    Entries and counters read under separate acquisitions can describe two
    different instants: a pass noted between the reads is in one and not the
    other, a pair no instant ever held. A post-leg reader takes everything from
    one of these instead, and what counts as a HOLE is decided here, once.

    ``refused`` counts notes that reached the lock after ``seal`` and changed
    nothing. An observation, not a hole, so ``incomplete`` does not read it.
    A pass sample dropped for budget counts in ``dropped``, like an entry.
    """

    entries: tuple[tuple[int, bytes | None], ...]
    iterations: int
    dropped: int
    failed: bool
    refused: int = 0
    passes: tuple[tuple[int, int | None, int | None], ...] = ()

    @property
    def unreadable(self) -> int:
        """Entries holding a marker instead of command bytes.

        DERIVED, never counted alongside the entries. A separate counter can
        disagree with them in either direction -- incremented before the
        append it overcounts when the append fails, incremented after it
        undercounts when the increment fails -- and both break the partition
        the summary reports. Reading the entries cannot.
        """
        return sum(
            1 for _, digest in self.entries if digest == COMMAND_UNREADABLE
        )

    @property
    def incomplete(self) -> bool:
        """Whether the ledger has a HOLE.

        An entry dropped for budget, an internal fault, or an entry holding a
        marker instead of command bytes. ``None`` and ``COMMAND_RAISED`` are
        NOT holes -- both are outcomes two runs can be compared on.
        """
        return bool(self.dropped or self.failed or self.unreadable)

    @property
    def bounded(self) -> tuple[int, tuple[tuple, ...]]:
        """Its drop count and the records its capacity bounds, for
        ``status_contradiction``: one count for both, so a drop means the
        entries or the pass samples are full, and neither ever shrinks."""
        return self.dropped, (self.entries, self.passes)

    def contradiction(self) -> str | None:
        """None, or fewer passes counted than held: ``note_pass`` counts first."""
        if self.iterations < len(self.passes):
            return f"{self.iterations} passes counted, {len(self.passes)} held"
        return None


class CommandLoopLog:
    """Bounded per-iteration command evidence, safe from the worker thread."""

    def __init__(self, capacity: int) -> None:
        self._capacity = max(0, int(capacity))
        self._lock = threading.Lock()
        self._entries: list[tuple[int, bytes | None]] = []
        self._passes: list[tuple[int, int | None, int | None]] = []
        self._iteration: int | None = None
        self._iterations = 0
        self._dropped = 0
        # Sealed, refused and faulted, all under the lock.
        self._seal = SealState()

    def note_iteration(self, iteration: int) -> None:
        """The worker entered iteration ``iteration``.

        Called at the TOP of the loop body, not before the source dispatch: a
        command executed earlier in the same pass has to carry the same number
        as the dispatch that follows it, or the two cannot be compared.
        """
        def change() -> None:
            self._iteration = int(iteration)
            self._iterations += 1

        record_under(self._lock, self._seal, change)

    def note_pass(
        self, iteration: int, observed: int | None, admitted: int | None
    ) -> None:
        """``note_iteration``, with the ATTITUDE ledger's cutoff as the pass
        began, which ``PassObserver`` read and released before this lock.

        The sample is built whole first. The iteration advances even when
        the sample is dropped for budget, as it does for an entry.
        """
        def change() -> None:
            sample = (
                int(iteration),
                None if observed is None else int(observed),
                None if admitted is None else int(admitted),
            )
            self._iteration = sample[0]
            self._iterations += 1
            if len(self._passes) >= self._capacity:
                self._dropped += 1
                return
            self._passes.append(sample)

        record_under(self._lock, self._seal, change)

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None:
        """Digest the command that iteration actually issued.

        ``command`` is the object ``execute_work`` RETURNED -- nothing is
        recomputed and no control input is re-read. A ``None`` command still
        gets an entry: "ran work, produced nothing" is a comparable outcome
        and a missing entry is not.
        """
        def change() -> None:
            digest = _digest_of(command, raised)
            # Built whole before anything is mutated, so a bad iteration
            # cannot fail halfway through a mutation.
            entry = (int(iteration), digest)
            if len(self._entries) >= self._capacity:
                self._dropped += 1
                return
            self._entries.append(entry)

        record_under(self._lock, self._seal, change)

    def seal(self) -> None:
        """Refuse every later note, and count each: ``determinism_seal``."""
        seal_under(self._lock, self._seal)

    def current_iteration(self) -> int | None:
        with self._lock:
            return self._iteration

    def capture(self) -> CommandCapture:
        """Entries and counters from ONE acquisition; see ``CommandCapture``.

        O(entries) under the lock, which is affordable because this is a
        POST-LEG read: the summary asks, through the trace's own capture, and
        nothing on the engagement path does. Do not move it onto a per-frame
        path without replacing the copy -- the trace is decision-neutral, NOT
        free.
        """
        with self._lock:
            return CommandCapture(
                entries=tuple(self._entries),
                iterations=self._iterations,
                dropped=self._dropped,
                failed=self._seal.faulted,
                refused=self._seal.refused,
                passes=tuple(self._passes),
            )

    def entries(self) -> list[tuple[int, bytes | None]]:
        with self._lock:
            return list(self._entries)

    @property
    def iterations(self) -> int:
        with self._lock:
            return self._iterations

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    @property
    def failed(self) -> bool:
        with self._lock:
            return self._seal.faulted

    @property
    def unreadable(self) -> int:
        """Derived from one capture; see ``CommandCapture.unreadable``."""
        return self.capture().unreadable

    @property
    def incomplete(self) -> bool:
        """Whether this ledger has a HOLE, from ONE atomic capture.

        The ledger decides what counts, because the ledger owns the counters,
        and ``CommandCapture.incomplete`` is the one place that says how. One
        acquisition, so a reader cannot see a torn combination of
        separately-locked properties.
        """
        return self.capture().incomplete


class NullCommandLoop:
    """The off switch, as an object: the worker pays two no-op calls a pass.

    A separate object rather than ``None`` on the port, so the worker never
    grows an ``if observer is not None`` on its own hot loop.
    """

    __slots__ = ()

    def note_iteration(self, iteration: int) -> None: ...

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None: ...


NULL_COMMAND_LOOP = NullCommandLoop()


__all__ = [
    "COMMAND_ENTRY_DIGEST",
    "COMMAND_ENTRY_ITERATION",
    "COMMAND_RAISED",
    "COMMAND_UNREADABLE",
    "NULL_COMMAND_LOOP",
    "CommandCapture",
    "CommandLoopLog",
    "NullCommandLoop",
]
