"""One recording is ONE critical section, in every evidence store.

Every store the trace keeps records under its own lock: the row journal and
the command loop's ledger now, the ATTITUDE admission ledger later. A sealed
capture means something only if two things hold of every recording in every
one of them, so both are written once, here.

**A fault is counted inside the section it happened in.** The journal's
recorders used to latch a fault in a SECOND acquisition, taken after the first
had released on its way out of the ``with``, and the command log built its
entry before taking its lock at all. Either way a window opened between the
fault and its latch, in which another thread could take the lock and read the
store as clean: a seal and a capture there would freeze evidence that had lost
a recording and say it had not. So the store's fault flag is set before the
lock is released, and before any latch work that can itself fail. It is a
plain attribute store, and nothing clears it. Everything done under the lock
is covered, the refusal's own count included: a refusal that could not be
counted, and said nothing, would read as one refusal fewer.

**An interrupt is passed on, and flagged.** A ``BaseException`` that is not
an error, a Ctrl+C or a ``SystemExit``, is never swallowed here: a recorder is
not the place to stop one. But the recording it cut short may have changed
its store part-way, the journal's watermark moved and its row never appended,
so it sets the flag before the release, like any fault. It latches no log:
latch work that raised would take the interrupt's place.

**After the seal, the contents never change.** A recording that takes the
lock after ``seal_under`` counts a refusal and returns. So a capture taken
after the seal reads the contents as they stood at it, with the fault of every
recording that took the lock first. If even the count fails, the flag says so,
and the store's log is left as it was.

Not claimed. A refusal count is an observation: zero refusals is not
quiescence, only the absence of a recording that reached a sealed lock. A
recording whose lock would not even open, or that is interrupted while it is
taken, is ordered against nothing; it sets the flag, the one thing it still
can, so every capture after it reads the store as failed. An interrupt that
lands before a recorder is called at all is the run's to report, not a
store's. And none of this says a leg finished.

The rule is D5 of the LANDING2 step-1 plan.
"""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager
from typing import Any, TypeVar

_T = TypeVar("_T")


class SealState:
    """One store's seal, its refusal count and its fault flag.

    Written under the store's own lock by ``record_under`` and ``seal_under``,
    with one exception: when a recording's lock will not open, its latch
    raises, or an interrupt passes through it, the fault flag is stored again
    without the lock, the one write left that cannot fail. Slotted because a
    store holds one, and the journal's lock-ordering argument needs every
    object it holds to refuse a new attribute (``determinism_journal``).
    """

    __slots__ = ("sealed", "refused", "faulted")

    def __init__(self) -> None:
        self.sealed = False
        self.refused = 0
        self.faulted = False


def record_under(
    lock: AbstractContextManager[Any],
    state: SealState,
    change: Callable[[], _T],
    latch: Callable[[], None] | None = None,
) -> _T | None:
    """Run ``change`` as ONE recording, refused once the store is sealed.

    Take the lock; if sealed, count a refusal and return; run ``change``, which
    builds and then appends; on any error, set the flag FIRST and only then run
    ``latch``, the store's own log if it keeps one; on an interrupt, set the
    flag and pass the interrupt on; release. Returns what ``change`` returned,
    or None when it was refused or faulted.

    It raises nothing but an interrupt, because a recording's caller is a
    command path. ``drain`` comes through here too, though an owner calls it,
    because it is a mutation like the others and the seal has to refuse it the
    same way.
    """
    try:
        with lock:
            return _in_section(state, change, latch)
    except Exception:  # noqa: BLE001 - a command path is the caller
        # The lock would not open, or the latch raised after the flag was set.
        # A plain store needs neither.
        state.faulted = True
        return None
    except BaseException:
        # An interrupt. From inside the section it is flagged already; from
        # the lock, it is flagged here, ordered against nothing.
        state.faulted = True
        raise


def _in_section(
    state: SealState,
    change: Callable[[], _T],
    latch: Callable[[], None] | None,
) -> _T | None:
    """What a recording does while it holds the lock, every fault in it
    counted there: the refusal's count as much as ``change``, and an
    interrupt as much as an error."""
    try:
        if state.sealed:
            state.refused += 1
            return None
        return change()
    except Exception:  # noqa: BLE001 - a command path is the caller
        state.faulted = True
        # Sealed, the fault was in the refusal's count, and a sealed store's
        # log is not touched: the flag alone says it, and every capture reads
        # the flag.
        if latch is not None and not state.sealed:
            latch()
        return None
    except BaseException:
        # Passed on, never swallowed, but flagged before the release: the
        # recording it cut short may have changed the store part-way. No
        # latch: one that raised would take the interrupt's place, and
        # ``record_under`` would swallow it as an error.
        state.faulted = True
        raise


def seal_under(lock: AbstractContextManager[Any], state: SealState) -> None:
    """Seal one store under its own lock: every later recording is refused.

    Takes that lock alone. An owner sealing several stores seals them one after
    another and never nested, which keeps the rule that no two of the trace's
    locks are ever held at once.
    """
    with lock:
        state.sealed = True


__all__ = ["SealState", "record_under", "seal_under"]
