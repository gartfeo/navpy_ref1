"""Build a traced case's evidence in memory, the way the writer builds it.

The capture is sealed and made into the trace file's bytes and the
manifest's by the writer's own functions (``determinism_evidence``), then read
back by the reader (``determinism_reader``). So every checksum and count is
valid by construction, and a test that needs a record no recorder writes edits
the CAPTURE first, never the bytes. Nothing here writes a file.

``eligible_trace`` is the leg every eligibility test departs from: it meets
all thirteen rules of D8 of the LANDING2 step-1 plan.
"""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import replace
from types import SimpleNamespace
from typing import Any

from navpy.modules.vision.sim.determinism_admission_ledger import PassObserver
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    LIFECYCLE_ACTIVATED,
    LIFECYCLE_CLOSED,
    OUTPUT_EMPTY,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_evidence import (
    SUMMARY_NAME,
    evidence_manifest,
    file_record,
    sealed_capture,
    trace_lines,
    trace_record,
)
from navpy.modules.vision.sim.determinism_reader import (
    CaseEvidence,
    read_evidence,
)
from navpy.modules.vision.sim.determinism_trace import (
    DeterminismTrace,
    TraceCapture,
)
from tests.modules.vision.truth_packet_factory import truth_packet

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.
IDENTITY = {
    "harness": {"sha256": "a" * 64, "files": 12},
    "case": {"name": "speed-1-run-1", "definition_sha256": "d" * 64},
    "given_error": None,
    "endpoints": {
        "start": {"sha256": "e" * 64, "files": 3},
        "teardown": {"sha256": "e" * 64, "files": 3},
    },
    "run": {"pid": 1},
}


def stamp_ms(number: int) -> int:
    """The boot stamp of the ATTITUDE ruled ``number``: 20 ms apart."""
    return 10_000 + 20 * number


def ruling(
    number: int,
    stamp: int | None,
    accepted: bool = True,
    reason: str | None = None,
) -> SimpleNamespace:
    """What the router's tap hands the ledger: one ruling on one ATTITUDE."""
    return SimpleNamespace(
        ruling=number, time_boot_ms=stamp, accepted=accepted, reason=reason
    )


def admit(trace: DeterminismTrace, number: int) -> None:
    """Ruling ``number`` admitted, then the association it caused, as the
    router's tap and then the source's callback record them."""
    stamp = stamp_ms(number)
    trace.ledger.append(ruling(number, stamp))
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=stamp / 1000
    )


def eligible_trace(
    period_us: int = PERIOD_US, *, capacity: int | None = None
) -> DeterminismTrace:
    """One leg that meets every rule. The ledger is owed rulings 1..3, all
    admitted; the window opens before any and closes on the third; each has
    its association row; the worker passes twice, the second issuing a
    command and dispatching; the leg closes at epoch 1. Only
    ``period_us`` 0 breaks a rule (D8.2). ``capacity`` is the three
    stores', the environment's by default."""
    trace = DeterminismTrace(period_us, capacity=capacity)
    passes = PassObserver(trace.ledger, trace.command_log)
    trace.record_subscription(epoch=0, outcome=SUBSCRIPTION_OPENED)
    trace.record_lifecycle(
        epoch=0, outcome=LIFECYCLE_ACTIVATED, resulting_epoch=1
    )
    passes.note_iteration(1)
    admit(trace, 1)
    admit(trace, 2)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED, message=truth_packet())
    passes.note_iteration(2)
    trace.command_log.note_command(2, None)
    trace.record_output(
        epoch=1, outcome=OUTPUT_EMPTY, taken_at_us=trace.watermark_us()
    )
    admit(trace, 3)
    trace.record_subscription(epoch=1, outcome=SUBSCRIPTION_CLOSED)
    trace.record_lifecycle(
        epoch=1, outcome=LIFECYCLE_CLOSED, resulting_epoch=2
    )
    return trace


def admission(**fields: Any) -> dict[str, Any]:
    """What the ledger's subscription says it was owed, as the child's link
    reports it (``pixel_pn_admission_ledger``): by default, ``eligible_trace``'s."""
    return {
        "live": True,
        "first": 1,
        "end": 3,
        "faults": 0,
        "faulted": False,
        "error": None,
        "window_rows": 0,
        **fields,
    }


def finalization(**fields: Any) -> dict[str, Any]:
    """The teardown's finalization, with the epoch the writer adds to it."""
    return {
        "worker": "stopped",
        "mavlink_quiescence": "unverified",
        "teardown_errors": [],
        "admission": admission(),
        "epoch": 1,
        **fields,
    }


def evidence_bytes(
    capture: TraceCapture,
    *,
    identity: Any = IDENTITY,
    held: int = 0,
    summary_error: str | None = None,
    **fields: Any,
) -> tuple[bytes, bytes]:
    """The manifest's bytes and the trace file's, as the writer makes them
    from ``capture``. ``fields`` replace the finalization's."""
    lines = trace_lines(capture)
    manifest = evidence_manifest(
        capture,
        trace=trace_record(lines, None),
        summary=file_record(SUMMARY_NAME, summary_error),
        identity=identity,
        finalization=finalization(**fields),
        error=None,
        held=held,
    )
    return json.dumps(manifest, indent=2).encode("utf-8"), lines.data


def case_of(capture: TraceCapture, **fields: Any) -> CaseEvidence:
    """``capture`` read back the way the reader reads a case's files."""
    return read_evidence(*evidence_bytes(capture, **fields))


def eligible_capture() -> TraceCapture:
    """``eligible_trace``, sealed and captured as the writer does."""
    return sealed_capture(eligible_trace())


def with_rows(capture: TraceCapture, rows: Iterable[tuple]) -> TraceCapture:
    """``capture`` holding ``rows`` instead, and a row count to match."""
    held = tuple(rows)
    return replace(
        capture, rows=held, status=replace(capture.status, rows=len(held))
    )


def edited_case(capture: TraceCapture, **replaced: Any) -> CaseEvidence:
    """``capture``'s case with manifest keys replaced as a hand edit would
    replace them: for values the writer cannot make, since it copies every
    mapping it is given."""
    manifest_data, trace_data = evidence_bytes(capture)
    manifest = json.loads(manifest_data)
    manifest.update(replaced)
    return read_evidence(
        json.dumps(manifest, indent=2).encode("utf-8"), trace_data
    )


__all__ = [
    "IDENTITY",
    "PERIOD_US",
    "admission",
    "admit",
    "case_of",
    "edited_case",
    "eligible_capture",
    "eligible_trace",
    "evidence_bytes",
    "finalization",
    "ruling",
    "stamp_ms",
    "with_rows",
]
