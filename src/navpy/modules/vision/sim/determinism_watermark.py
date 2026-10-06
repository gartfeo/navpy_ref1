"""The source-time watermark rule: forward only, with zero tolerance.

W3 needs monotonic autopilot source time. A stamp that goes backwards or
repeats is not a small error to smooth over -- it means two decisions cannot be
ordered against each other, and a run whose decisions cannot be ordered cannot
be compared against another run. So a bad stamp is RECORDED and never allowed
to drag the watermark with it.

Extracted from ``RowJournal`` so the rule can be read on its own. It is NOT a
pure function and calling it one was wrong: it writes a violation row through
``log`` when the stamp does not advance. What it does not do is hold state or
take a lock -- the watermark it returns is the caller's to store, and the caller
holds the row lock throughout.
"""

from __future__ import annotations

from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    EVENT_VIOLATION,
    VIOLATION_SOURCE_REGRESSED,
    VIOLATION_SOURCE_REPEATED,
)


def advanced_watermark(
    log: Any,
    previous: int | None,
    epoch: int,
    micros: int,
) -> int | None:
    """The watermark after seeing ``micros``, and a violation row if it stalls.

    Returns ``micros`` when it moves the watermark forward, and ``previous``
    unchanged otherwise -- having first written to ``log`` WHY it could not
    move, distinguishing a stamp that went backwards from one that repeated.
    That write is a side effect, so this is an extracted rule rather than a pure
    function. The caller holds the row lock; this neither takes nor knows about
    it.
    """
    if previous is None or micros > previous:
        return micros
    reason = (
        VIOLATION_SOURCE_REGRESSED
        if micros < previous
        else VIOLATION_SOURCE_REPEATED
    )
    log.note_violation(
        (EVENT_VIOLATION, int(epoch), reason, (micros, previous))
    )
    return previous


__all__ = ["advanced_watermark"]
