"""D8's rules 5 to 9: the ATTITUDE ledger, joined to the rows and the passes.

Each rule reads the subscription's evidence (the teardown's ``admission``,
from ``pixel_pn_admission_ledger``) for the rulings the ledger was owed,
``first``..``end``, and holds the ledger's entries, the worker's pass samples
and the association rows to them. The rules are D8 of
the LANDING2 step-1 plan, and
``determinism_eligibility`` judges a case by all thirteen. Each takes a case
the reader accepted and returns its reasons, none when it holds. None
raises: a figure that is missing or malformed fails its rule, and a reason
prints only figures the case holds, never one worked out from them, since
first - 1 of the widest first the reader takes is too wide to print.

Two readings, both on the conservative side. A bound the evidence cannot
give fails every rule that needs one, so a case without its subscription's
bounds fails 7 and 9 as well as 5 and 6. And an association row that names no
ruling is not an admitted ledger entry, so it fails rule 9: with the ledger
subscribed before the source started (rule 5), every ATTITUDE the source
acted on was admitted, and handed to the ledger, first.
"""

from __future__ import annotations

from bisect import bisect_right
from collections import Counter
from collections.abc import Mapping, Sequence
from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_FENCED,
    ASSOCIATION_RULING,
    EVENT_ASSOCIATION,
    EVENT_OUTPUT,
    EVENT_SUBSCRIPTION,
    OUTPUT_WORKER_ITERATION,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
    SUBSCRIPTION_RULING,
)
from navpy.modules.vision.sim.determinism_reader import CaseEvidence

# A ledger stamp is whole milliseconds of boot time; a row's, microseconds.
_MICROS_PER_MS = 1_000


def admission_of(case: CaseEvidence) -> Mapping[str, Any]:
    """The subscription's evidence, or nothing when there is none to read."""
    finalization = case.manifest["finalization"]
    admission = (
        finalization.get("admission") if type(finalization) is dict else None
    )
    return admission if type(admission) is dict else {}


def bounds_of(case: CaseEvidence) -> tuple[int, int] | None:
    """(first, end): the rulings the ledger was owed, when both are known."""
    admission = admission_of(case)
    first, end = admission.get("first"), admission.get("end")
    if type(first) is int and type(end) is int:
        return first, end
    return None


def _first(label: str, found: Sequence[Any]) -> list[str]:
    """One reason for many findings: how many, and the first."""
    return [f"{label}: {len(found)}, the first {found[0]!r}"] if found else []


def tap_live(case: CaseEvidence) -> list[str]:
    """5. The tap was live, and the ledger subscribed before the source
    started: its subscription was in place with no window row yet in the
    journal."""
    admission = admission_of(case)
    if not admission:
        return ["there is no admission evidence"]
    reasons = []
    if admission.get("live") is not True:
        reasons.append(f"the tap was not live: {admission.get('error')}")
    rows = admission.get("window_rows")
    if type(rows) is not int or rows != 0:
        reasons.append(
            f"the ledger subscribed with {rows!r} window rows in the journal"
        )
    return reasons


def owed_rulings(case: CaseEvidence) -> list[str]:
    """6. No delivery faulted, counted or flagged, and the ledger holds
    exactly the rulings first..end, in order."""
    admission = admission_of(case)
    reasons = []
    faults, faulted = admission.get("faults"), admission.get("faulted")
    if type(faults) is not int or faults != 0:
        reasons.append(f"deliveries faulted: {faults!r}")
    if faulted is not False:
        reasons.append(f"the subscription's faulted flag is {faulted!r}")
    bounds = bounds_of(case)
    if bounds is None:
        return reasons + ["the rulings the ledger was owed are unknown"]
    first, end = bounds
    held = [entry[0] for entry in case.capture.ledger.entries]
    # Counted before numbered: an end no list could hold is judged, not made.
    owed = max(0, end - first + 1)
    if len(held) != owed or any(ruling != first + at for at, ruling in enumerate(held)):
        reasons.append(f"the ledger does not hold exactly rulings {first}..{end}")
    return reasons


def pass_cutoffs(case: CaseEvidence) -> list[str]:
    """7. The passes are 1..N, N the iterations. Each observed cutoff, None
    read as first - 1, never falls below the one before and lies in
    [first - 1, end]; each admitted cutoff is the last ACCEPTED ruling at or
    below it, or None when there is none."""
    commands = case.capture.commands
    reasons = []
    identities = [sample[0] for sample in commands.passes]
    # Counted before numbered, as rule 6 is; the iterations are a count.
    if len(identities) != commands.iterations or any(
        identity != at for at, identity in enumerate(identities, 1)
    ):
        reasons.append(f"the passes are not exactly 1..{commands.iterations}")
    bounds = bounds_of(case)
    if bounds is None:
        return reasons + ["the cutoffs cannot be placed: the rulings owed are unknown"]
    first, end = bounds
    accepted = sorted(entry[0] for entry in case.capture.ledger.entries if entry[2])
    decreasing, outside, wrong = [], [], []
    previous = first - 1
    for iteration, observed, admitted in commands.passes:
        cutoff = first - 1 if observed is None else observed
        if cutoff < previous:
            decreasing.append(iteration)
        if not first - 1 <= cutoff <= end:
            outside.append(iteration)
        previous = max(previous, cutoff)
        below = bisect_right(accepted, cutoff)
        if admitted != (accepted[below - 1] if below else None):
            wrong.append(iteration)
    return (
        reasons
        + _first("passes whose observed cutoff decreases", decreasing)
        + _first(
            f"passes observed below the ruling before {first}, or past {end}",
            outside,
        )
        + _first("passes whose admitted cutoff is not the last accepted", wrong)
    )


def pass_references(case: CaseEvidence) -> list[str]:
    """8. Every OUTPUT row's iteration, None aside, and every command entry's
    names a pass, and the command entries' iterations strictly increase. An
    OUTPUT with no iteration ran outside every pass: its pass label is
    unknown, which the label table allows."""
    capture = case.capture
    passes = {sample[0] for sample in capture.commands.passes}
    outputs = [
        row[OUTPUT_WORKER_ITERATION]
        for row in capture.rows
        if row[0] == EVENT_OUTPUT and row[OUTPUT_WORKER_ITERATION] is not None
    ]
    iterations = [entry[0] for entry in capture.commands.entries]
    reasons = _first(
        "output rows naming no pass", [i for i in outputs if i not in passes]
    ) + _first(
        "command entries naming no pass", [i for i in iterations if i not in passes]
    )
    if any(later <= earlier for earlier, later in zip(iterations, iterations[1:])):
        reasons.append("the command entries' iterations do not strictly increase")
    return reasons


def association_window(case: CaseEvidence) -> list[str]:
    """9. One OPENED row, then one CLOSED, their rulings s0 and s1 (None read
    as first - 1) in order within [first - 1, end]. The association rows'
    rulings strictly increase and each is an admitted ledger entry; every
    ruling admitted strictly inside (s0, s1) has exactly one row; and every
    committed or fenced row carries its ruling's stamp."""
    capture = case.capture
    stamps = {
        ruling: stamp
        for ruling, stamp, accepted, _ in capture.ledger.entries
        if accepted
    }
    associations = [
        (index, row)
        for index, row in enumerate(capture.rows)
        if row[0] == EVENT_ASSOCIATION
    ]
    rulings = [row[ASSOCIATION_RULING] for _, row in associations]
    reasons = []
    if any(
        earlier is None or later is None or later <= earlier
        for earlier, later in zip(rulings, rulings[1:])
    ):
        reasons.append("the association rows' rulings do not strictly increase")
    reasons += _first(
        "association rows naming no admitted ruling",
        [ruling for ruling in rulings if ruling not in stamps],
    )
    reasons += _first("committed or fenced rows not stamp consistent", [
        index
        for index, row in associations
        if row[2] in (ASSOCIATION_COMMITTED, ASSOCIATION_FENCED)
        and row[ASSOCIATION_RULING] in stamps
        and not _consistent(row[3], stamps[row[ASSOCIATION_RULING]])
    ])
    window = _window(case)
    if isinstance(window, str):
        return reasons + [window]
    s0, s1 = window
    named = Counter(rulings)
    return reasons + _first(
        "rulings admitted inside the window without exactly one row",
        [(ruling, named[ruling]) for ruling in stamps
         if s0 < ruling < s1 and named[ruling] != 1],
    )


def _consistent(micros: int | None, stamp_ms: int | None) -> bool:
    return (
        micros is not None
        and stamp_ms is not None
        and micros == stamp_ms * _MICROS_PER_MS
    )


def _window(case: CaseEvidence) -> tuple[int, int] | str:
    """(s0, s1), or why the window cannot be placed."""
    rows = [row for row in case.capture.rows if row[0] == EVENT_SUBSCRIPTION]
    outcomes = [row[2] for row in rows]
    if outcomes != [SUBSCRIPTION_OPENED, SUBSCRIPTION_CLOSED]:
        return f"the window's rows are {outcomes}, not OPENED then CLOSED"
    bounds = bounds_of(case)
    if bounds is None:
        return "the window cannot be placed: the rulings owed are unknown"
    first, end = bounds
    named = [row[SUBSCRIPTION_RULING] for row in rows]
    s0, s1 = (first - 1 if ruling is None else ruling for ruling in named)
    if not first - 1 <= s0 <= s1 <= end:
        return (
            f"the window's rulings {named[0]!r}..{named[1]!r} are not in order"
            f" from the ruling before {first} to {end}"
        )
    return s0, s1


__all__ = [
    "admission_of",
    "association_window",
    "bounds_of",
    "owed_rulings",
    "pass_cutoffs",
    "pass_references",
    "tap_live",
]
