"""D8: whether a traced case is ELIGIBLE for a comparison verdict.

A case is ELIGIBLE only if all thirteen rules of D8 hold
(the LANDING2 step-1 plan). Anything else is
INSPECTABLE, with every failed rule as a reason, and never a clean
comparison. Rule 1, the manifest vouching for its files, is the reader's
(``determinism_reader``): evidence it refuses is judged on that alone
(``refusal``), since no other rule can be judged on files that do not vouch
for themselves. ``judge`` holds a case the reader accepted to rules 2 to 13;
5 to 9, which join the ledger to the rows and the passes, are in
``determinism_ledger_rules``. No rule raises: a figure that is missing or
malformed fails its rule, with the reason.

Rule 12 reads more than the plan's letter, on the conservative side: it also
fails when the manifest counts a failure held for raising (``held`` above
zero) or says there is no summary, since either means the evidence
step failed (delivery step 6's review). VIOLATION rows are the rows' own
account of a stamp that stalled, and no rule reads them.
"""

from __future__ import annotations

import re
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    EVENT_LIFECYCLE,
    LIFECYCLE_CLOSED,
    SLOT_PAIRS,
)
from navpy.modules.vision.sim.determinism_evidence import (
    EVIDENCE_STEP,
    SOURCE_CLOSE_STEP,
    WORKER_STOPPED,
)
from navpy.modules.vision.sim.determinism_ledger_rules import (
    association_window,
    owed_rulings,
    pass_cutoffs,
    pass_references,
    tap_live,
)
from navpy.modules.vision.sim.determinism_reader import CaseEvidence, read_case
from navpy.modules.vision.sim.determinism_slots import slot_index
from navpy.modules.vision.sim.determinism_trace_decode import EvidenceRefused
from navpy.modules.vision.sim.determinism_truth_evidence import raw_truth_reasons

ELIGIBLE = "ELIGIBLE"
INSPECTABLE = "INSPECTABLE"
_SHA256 = re.compile("[0-9a-f]{64}")


@dataclass(frozen=True)
class Reason:
    """One failed rule, and why."""

    rule: int
    text: str


@dataclass(frozen=True)
class Verdict:
    """ELIGIBLE with no reason, or INSPECTABLE with every failed rule's."""

    status: str
    reasons: tuple[Reason, ...]

    @property
    def eligible(self) -> bool:
        return self.status == ELIGIBLE

    @property
    def failed_rules(self) -> tuple[int, ...]:
        return tuple(sorted({reason.rule for reason in self.reasons}))


def refusal(error: EvidenceRefused) -> Verdict:
    """Evidence the reader refused: rule 1 fails, and no other is judged."""
    return Verdict(
        INSPECTABLE, (Reason(1, f"{error} (remaining rules are not judged)"),)
    )


def judge(case: CaseEvidence) -> Verdict:
    """Original D8 rules and, for v2, raw-truth capture rule 14."""
    reasons = tuple(
        Reason(rule, text) for rule, check in _RULES for text in check(case)
    )
    return Verdict(INSPECTABLE if reasons else ELIGIBLE, reasons)


def case_verdict(directory: Path) -> Verdict:
    """The verdict on the case whose files are in ``directory``."""
    try:
        case = read_case(directory)
    except EvidenceRefused as error:
        return refusal(error)
    return judge(case)


def _finalization(case: CaseEvidence) -> dict[str, Any]:
    finalization = case.manifest["finalization"]
    return finalization if type(finalization) is dict else {}


def _period(case: CaseEvidence) -> list[str]:
    """2. period_us > 0."""
    period = case.capture.period_us
    return [] if period > 0 else [f"period_us is {period}"]


def _slots(case: CaseEvidence) -> list[str]:
    """3. Every slot is its stamp // period_us, watermark slots included,
    and None exactly when its stamp is."""
    period = case.capture.period_us
    wrong = [
        (index, slot)
        for index, row in enumerate(case.capture.rows)
        for stamp, slot in SLOT_PAIRS[row[0]]
        if row[slot] != slot_index(row[stamp], period)
    ]
    if not wrong:
        return []
    return [
        f"slots that are not their stamp // period_us: {len(wrong)}, the "
        f"first row {wrong[0][0]}, field {wrong[0][1]}"
    ]


def _stores(case: CaseEvidence) -> list[str]:
    """4. Every store is complete: no overflow, failure, drain, unreadable
    payload or callback fault in the journal; no dropped, failed or
    unreadable command; no overflow, failure or gap in the ledger."""
    capture = case.capture
    status, commands, ledger = capture.status, capture.commands, capture.ledger
    figures = (
        ("the journal overflowed", status.overflowed),
        ("the journal failed", status.failed),
        ("the journal was drained", status.drained),
        ("a payload was unreadable", status.payload_unreadable),
        (f"callback faults: {status.callback_faults}", status.callback_faults),
        (f"commands dropped: {commands.dropped}", commands.dropped),
        ("the command log failed", commands.failed),
        (f"commands unreadable: {commands.unreadable}", commands.unreadable),
        (f"ledger entries dropped: {ledger.dropped}", ledger.dropped),
        ("the ledger failed", ledger.failed),
        ("the ledger has a gap", ledger.gapped),
    )
    reasons = [text for text, broken in figures if broken]
    if not reasons and not capture.complete:
        reasons.append("the capture is not complete")
    return reasons


def _clock(case: CaseEvidence) -> list[str]:
    """10. No ledger discontinuity and no stampless admission."""
    ledger = case.capture.ledger
    reasons = []
    if ledger.discontinuous:
        reasons.append("an admitted stamp steps back: the clock is discontinuous")
    if ledger.stampless:
        reasons.append(f"admissions without a stamp: {ledger.stampless}")
    return reasons


def _quiet(case: CaseEvidence) -> list[str]:
    """11. No refusal observed through the capture, and the worker's stop
    confirmed."""
    capture = case.capture
    reasons = [
        f"the {name} refused {count} recordings once sealed"
        for name, count in (
            ("journal", capture.status.refused),
            ("command log", capture.commands.refused),
            ("ledger", capture.ledger.refused),
        )
        if count
    ]
    worker = _finalization(case).get("worker")
    if worker != WORKER_STOPPED:
        reasons.append(f"the worker's stop is {worker!r}, not confirmed")
    return reasons


def _ended(case: CaseEvidence) -> list[str]:
    """12. The ending epoch is known, with a LIFECYCLE_CLOSED row at it, and
    neither source.close nor the evidence step failed."""
    manifest, finalization = case.manifest, _finalization(case)
    reasons = []
    epoch = finalization.get("epoch")
    if type(epoch) is not int:
        reasons.append(f"the ending epoch is unknown: {epoch!r}")
    elif not any(
        row[0] == EVENT_LIFECYCLE and row[1] == epoch and row[2] == LIFECYCLE_CLOSED
        for row in case.capture.rows
    ):
        reasons.append(f"no LIFECYCLE_CLOSED row at the ending epoch {epoch}")
    errors = finalization.get("teardown_errors")
    if type(errors) is not list or not all(
        type(entry) is dict and type(entry.get("step")) is str for entry in errors
    ):
        reasons.append(f"the teardown's step errors cannot be read: {errors!r}")
    else:
        reasons += [
            f"{entry['step']} failed: {entry.get('error')}"
            for entry in errors
            if entry["step"] in (SOURCE_CLOSE_STEP, EVIDENCE_STEP)
        ]
    if manifest["held"]:
        reasons.append(f"the evidence step failed: {manifest['held']} held")
    if manifest["summary"]["error"] is not None:
        reasons.append(
            f"the evidence step failed: no summary, {manifest['summary']['error']}"
        )
    return reasons


def _hashed(value: Any) -> bool:
    return type(value) is dict and _is_sha256(value.get("sha256"))


def _is_sha256(value: Any) -> bool:
    return type(value) is str and _SHA256.fullmatch(value) is not None


def _identified(case: CaseEvidence) -> list[str]:
    """13. The harness's identity and the case's name and definition sha256
    are present, and the child's endpoint hashes are equal."""
    identity = case.manifest["identity"]
    if type(identity) is not dict:
        return [f"the case has no identity: {identity!r}"]
    reasons = []
    if not _hashed(identity.get("harness")):
        reasons.append(f"no harness identity: {identity.get('harness')!r}")
    named = identity.get("case")
    named = named if type(named) is dict else {}
    if type(named.get("name")) is not str or not named.get("name"):
        reasons.append(f"the case has no name: {named.get('name')!r}")
    if not _is_sha256(named.get("definition_sha256")):
        reasons.append(
            f"no definition sha256: {named.get('definition_sha256')!r}"
        )
    endpoints = identity.get("endpoints")
    endpoints = endpoints if type(endpoints) is dict else {}
    start, teardown = endpoints.get("start"), endpoints.get("teardown")
    if not (_hashed(start) and _hashed(teardown)):
        reasons.append(f"an endpoint has no hash: {start!r}, {teardown!r}")
    elif start["sha256"] != teardown["sha256"]:
        reasons.append("the endpoint hashes differ")
    return reasons


_RULES: tuple[tuple[int, Callable[[CaseEvidence], list[str]]], ...] = (
    (2, _period),
    (3, _slots),
    (4, _stores),
    (5, tap_live),
    (6, owed_rulings),
    (7, pass_cutoffs),
    (8, pass_references),
    (9, association_window),
    (10, _clock),
    (11, _quiet),
    (12, _ended),
    (13, _identified),
    (14, raw_truth_reasons),
)


__all__ = [
    "ELIGIBLE",
    "INSPECTABLE",
    "Reason",
    "Verdict",
    "case_verdict",
    "judge",
    "refusal",
]
