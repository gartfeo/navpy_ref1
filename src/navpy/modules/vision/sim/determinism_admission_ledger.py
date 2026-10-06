"""The ATTITUDE admission ledger: every ruling the router made, in order.

The associator acts on the newest ATTITUDE the router admitted, and the rows
say what it decided; this says what the router ruled. The router hands every
ruling it makes on ATTITUDE, an admission or a rejection with its reason, to
its admission tap (``admission_tap``), and the owner subscribes ``append``
there before the source starts. The design is D2 and D3 of
the LANDING2 step-1 plan.

An entry is (ruling, time_boot_ms or None, accepted, reason or None): plain
values, built whole under the lock. Two figures are kept live beside the
entries, because a reader on another thread needs them while the leg runs:
the latest ruling OBSERVED and the latest ADMITTED, both from ``cutoff`` in
one acquisition. Where nothing was dropped and nothing faulted they are the
last entry's and the last admitted entry's, and a reader holds a capture to
that (``LedgerCapture.contradiction``). Everything else is DERIVED from the
entries when a capture is read, never counted beside them, so no figure can
disagree with the entries it describes (the precedent is
``CommandCapture.unreadable``).

Bounded like the other stores: past its capacity it keeps the EARLIEST
entries and counts the rest, and the live figures still advance, so an
overflow loses entries and never the fact that rulings were made. Its lock is
a leaf: nothing is called under it but the ruling's own fields. Every append
is ONE critical section, refused once sealed (``determinism_seal``), and
nothing raises into the tap but an interrupt, which the tap flags on the
subscription itself.

``PassObserver`` is the command worker's side of D3: each pass samples the
cutoff, releases this lock, and hands the pair to the command log as VALUES.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Protocol

from navpy.modules.vision.sim.determinism_seal import (
    SealState,
    record_under,
    seal_under,
)


@dataclass(frozen=True)
class LedgerCapture:
    """The whole ledger as of ONE acquisition of its lock, and immutable.

    ``refused`` counts appends that reached the lock after ``seal`` and
    changed nothing: an observation, not a hole, so ``incomplete`` does not
    read it. The derived figures describe the entries HELD; past an overflow
    they can miss what was dropped, and an overflowed ledger is incomplete.
    """

    entries: tuple[tuple[int, int | None, bool, str | None], ...]
    observed: int | None
    admitted: int | None
    dropped: int
    failed: bool
    refused: int = 0

    @property
    def overflowed(self) -> bool:
        return self.dropped > 0

    @property
    def bounded(self) -> tuple[int, tuple[tuple, ...]]:
        """Its drop count and the records its capacity bounds, for
        ``status_contradiction``: a ruling is dropped only when the entries
        are full, and no entry is ever removed."""
        return self.dropped, (self.entries,)

    @property
    def gapped(self) -> bool:
        """A ruling missing between two the ledger holds: one it was owed and
        never got. Whether the first and the last are the ones it was owed
        takes the subscription's bounds, which the owner reports."""
        return any(
            later[0] != earlier[0] + 1
            for earlier, later in zip(self.entries, self.entries[1:])
        )

    @property
    def rejections(self) -> dict[str, int]:
        """Rejected rulings, counted by the router's reason."""
        counts: dict[str, int] = {}
        for _, _, accepted, reason in self.entries:
            if not accepted:
                counts[str(reason)] = counts.get(str(reason), 0) + 1
        return counts

    @property
    def stampless(self) -> int:
        """Admitted rulings with no stamp, which no clock label can place."""
        return sum(
            1
            for _, stamp, accepted, _ in self.entries
            if accepted and stamp is None
        )

    @property
    def admitted_stamp(self) -> int | None:
        """The stamp of the latest admitted entry held."""
        for _, stamp, accepted, _ in reversed(self.entries):
            if accepted:
                return stamp
        return None

    @property
    def max_admitted_stamp(self) -> int | None:
        stamps = [
            stamp
            for _, stamp, accepted, _ in self.entries
            if accepted and stamp is not None
        ]
        return max(stamps) if stamps else None

    @property
    def discontinuous(self) -> bool:
        """D9: an admitted stamp below the admitted stamp before it. The
        router admits a step back only as a reboot: more than 60 s back, or
        into the fresh-boot window."""
        previous = None
        for _, stamp, accepted, _ in self.entries:
            if accepted and stamp is not None:
                if previous is not None and stamp < previous:
                    return True
                previous = stamp
        return False

    @property
    def incomplete(self) -> bool:
        """Whether the ledger has a HOLE: an entry dropped for budget, an
        internal fault, or a ruling missing between two it holds."""
        return bool(self.dropped or self.failed or self.gapped)

    def contradiction(self) -> str | None:
        """What of the live figures the entries show false: None when
        nothing does. With nothing dropped and no fault, every ruling handed
        to ``append`` was held whole, so the latest observed is the last
        entry's and the latest admitted the last accepted entry's, or None.
        Past a drop or a fault they may be later: no contradiction."""
        if self.dropped or self.failed:
            return None
        last = self.entries[-1][0] if self.entries else None
        admitted = next(
            (entry[0] for entry in reversed(self.entries) if entry[2]), None
        )
        if (self.observed, self.admitted) == (last, admitted):
            return None
        return (
            f"observed {self.observed!r} and admitted {self.admitted!r}, where"
            f" the entries end at {last!r} and {admitted!r}"
        )


class AdmissionLedger:
    """Every ATTITUDE ruling handed to it, under one leaf lock, from any
    thread. Slotted like every store the trace holds, so nothing can be
    parked on it (``determinism_journal`` says why that matters)."""

    __slots__ = (
        "_capacity",
        "_lock",
        "_entries",
        "_dropped",
        "_observed",
        "_admitted",
        "_seal",
    )

    def __init__(self, capacity: int) -> None:
        self._capacity = max(0, int(capacity))
        self._lock = threading.Lock()
        self._entries: list[tuple[int, int | None, bool, str | None]] = []
        self._dropped = 0
        self._observed: int | None = None
        self._admitted: int | None = None
        # Sealed, refused and faulted, all under the lock.
        self._seal = SealState()

    def append(self, ruling: Any) -> None:
        """The tap's callback: one ruling, as ONE recording.

        The entry is built whole before anything changes, so a ruling whose
        fields cannot be read changes nothing and is a fault. The live
        figures advance BEFORE the capacity check: an overflow drops the
        entry, never the fact that the ruling was made.
        """
        def change() -> None:
            stamp, reason = ruling.time_boot_ms, ruling.reason
            entry = (
                int(ruling.ruling),
                None if stamp is None else int(stamp),
                bool(ruling.accepted),
                None if reason is None else str(reason),
            )
            self._observed = entry[0]
            if entry[2]:
                self._admitted = entry[0]
            if len(self._entries) >= self._capacity:
                self._dropped += 1
                return
            self._entries.append(entry)

        record_under(self._lock, self._seal, change)

    def cutoff(self) -> tuple[int | None, int | None]:
        """(latest ruling observed, latest admitted), from ONE acquisition,
        each None while the ledger has none.

        Never raises but an interrupt: its callers are the command worker's
        pass and the source's recorders. A read that fails returns
        (None, None) and flags the ledger, which every later capture reads as
        failed, so the cutoff it could not give cannot pass for a clean one.
        """
        try:
            with self._lock:
                return self._observed, self._admitted
        except Exception:  # noqa: BLE001 - a command path is the caller
            self._seal.faulted = True
            return None, None

    def seal(self) -> None:
        """Refuse every later append, and count each: ``determinism_seal``."""
        seal_under(self._lock, self._seal)

    def capture(self) -> LedgerCapture:
        """Entries and figures from ONE acquisition; see ``LedgerCapture``.
        O(entries) under the lock: a post-leg read."""
        with self._lock:
            return LedgerCapture(
                entries=tuple(self._entries),
                observed=self._observed,
                admitted=self._admitted,
                dropped=self._dropped,
                failed=self._seal.faulted,
                refused=self._seal.refused,
            )


class PassLog(Protocol):
    """Where a pass and its sample are recorded: the command log, named here
    only by this shape, so this module cannot reach the command loop's class.
    """

    def note_pass(
        self, iteration: int, observed: int | None, admitted: int | None
    ) -> None: ...

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None: ...


class PassObserver:
    """The command worker's observer while tracing: each pass sampled (D3).

    ``note_iteration`` takes ONE ``cutoff``, which releases the ledger's lock,
    and only then records the pass with it in the command log, so the two
    locks are never held together and the sample crosses as VALUES. The label
    it supports (Q3, agreed) is the latest admitted stamp known when the pass
    began, a lower bound. ``note_command`` passes straight through.
    """

    __slots__ = ("_ledger", "_commands")

    def __init__(self, ledger: AdmissionLedger, commands: PassLog) -> None:
        self._ledger = ledger
        self._commands = commands

    def note_iteration(self, iteration: int) -> None:
        observed, admitted = self._ledger.cutoff()
        self._commands.note_pass(iteration, observed, admitted)

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None:
        self._commands.note_command(iteration, command, raised)


__all__ = ["AdmissionLedger", "LedgerCapture", "PassLog", "PassObserver"]
