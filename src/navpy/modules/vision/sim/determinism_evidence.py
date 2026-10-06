"""D7: a traced case's evidence, in the form it leaves the process.

Three files, all made from ONE capture: ``determinism_trace.jsonl``, every
record every store holds; ``determinism_summary.json``, the ended leg reduced;
and ``determinism_manifest.json``, which says what the other two are. This
module is the part with no file I/O: the seal and the capture, how a record is
encoded, and what the manifest holds. ``scripts/pixel_pn_determinism_evidence``
writes the files. The rule is D7 of
the LANDING2 step-1 plan.

**One capture, after every seal.** ``sealed_capture`` seals the journal, the
command log and the ledger, one after another and never nested, and only then
captures the three (D5). A recording that reaches a sealed store changes
nothing and is counted, so all three files describe the same contents however
late a callback runs. Not claimed: a refusal count is an observation, and zero
refusals is not quiescence.

**Every record, exactly.** A trace line is one canonical JSON object,
``{"fields": ..., "index": ..., "kind": ...}``: keys sorted, no spaces, ASCII,
one per line (``trace_line``). The rows come first, then the worker's passes,
its command entries and the ledger's entries, each numbered from 0 within its
kind. A record is a tuple and becomes an array, bytes become
``{"hex": ...}``, and None or an exact int, str or bool stays as it is. So
decoding a line gives back the record it came from, and the reader holds each
line to this form by making it again. A float, a mapping, a list, a bytearray
or a subclass of an exact type has no such form; none is ever recorded, so
one in a store is a defect, and the encode fails rather than coerce it into a
record of something else (``encode_field``).

**The manifest vouches, or says why not.** It gives the trace file's sha256,
line count and count per kind only when that file was written. It carries
every status field of every store, derived verdicts included, read off the
record types themselves so that a field a store gains later cannot be left
out (``store_status``). It also carries the gate's variables as the
environment holds them, the case's identity (D7b), the teardown's
finalization, and how many failures its writer held for raising when the
manifest was made (``held``): above zero, the evidence step fails (D8.12).
Whether the case is ELIGIBLE is D8, and the reader's to say
(``determinism_eligibility``): the writer never claims it, so ``eligibility``
is always None, and the reader refuses a manifest that says anything else.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from navpy.modules.vision.sim.determinism_trace import (
    DeterminismTrace,
    TraceCapture,
)
from navpy.modules.vision.sim.determinism_trace_gate import (
    DETERMINISM_TRACE_CAPACITY_ENV,
    DETERMINISM_TRACE_ENV,
)

TRACE_NAME = "determinism_trace.jsonl"
SUMMARY_NAME = "determinism_summary.json"
MANIFEST_NAME = "determinism_manifest.json"
MANIFEST_SCHEMA = "navpy.determinism_evidence"
MANIFEST_VERSION = 2
# The manifest's keys, in the order they are written. The reader refuses any
# other set, or order.
MANIFEST_KEYS = (
    "schema",
    "version",
    "trace",
    "summary",
    "period_us",
    "capacity",
    "gate",
    "identity",
    "finalization",
    "stores",
    "eligibility",
    "error",
    "held",
)
# A trace line's kind: the store its record came from.
KIND_ROW = "row"
KIND_PASS = "pass"
KIND_COMMAND = "command"
KIND_LEDGER = "ledger"
KINDS = (KIND_ROW, KIND_PASS, KIND_COMMAND, KIND_LEDGER)
# The environment variables that decide whether a case is traced, and how much.
GATE_VARIABLES = (DETERMINISM_TRACE_ENV, DETERMINISM_TRACE_CAPACITY_ENV)
# The teardown's words that D8 reads: the worker's confirmed stop, and the two
# steps whose failure makes a case ineligible (D8.11, D8.12). Spelled here, not
# in the teardown's script, so the teardown and the reader share one spelling.
WORKER_STOPPED = "stopped"
SOURCE_CLOSE_STEP = "source.close"
EVIDENCE_STEP = "determinism_evidence"
# The store fields that hold records. The records are the trace file's; the
# manifest counts them.
_RECORD_FIELDS = frozenset({"entries", "passes"})


def sealed_capture(trace: DeterminismTrace) -> TraceCapture:
    """Seal every store, each under its own lock alone, then capture once.

    D5's order: the journal, the command log, the ledger. The capture reads
    each store's refusal count with its contents.
    """
    trace.journal.seal()
    trace.command_log.seal()
    trace.ledger.seal()
    return trace.capture()


def encode_field(value: Any) -> Any:
    """One recorded value as JSON that decodes back to exactly it.

    Raises TypeError for a value with no exact form (module docstring).
    """
    kind = type(value)
    if value is None or kind is bool or kind is int or kind is str:
        return value
    if kind is bytes:
        return {"hex": value.hex()}
    if kind is tuple:
        return [encode_field(item) for item in value]
    raise TypeError(f"no exact encoding for a {type.__repr__(kind)}")


@dataclass(frozen=True)
class TraceLines:
    """The trace file's bytes, and what the manifest says of them."""

    data: bytes
    sha256: str
    lines: int
    kinds: dict[str, int]


def trace_lines(capture: TraceCapture) -> TraceLines:
    """Every record of every store, one canonical line each, in store order.

    Raises TypeError, from ``encode_field``, when a record holds a value
    with no exact encoding.
    """
    stores = (
        (KIND_ROW, capture.rows),
        (KIND_PASS, capture.commands.passes),
        (KIND_COMMAND, capture.commands.entries),
        (KIND_LEDGER, capture.ledger.entries),
    )
    lines = [
        trace_line(kind, index, record)
        for kind, records in stores
        for index, record in enumerate(records)
    ]
    data = "".join(lines).encode("ascii")
    return TraceLines(
        data=data,
        sha256=hashlib.sha256(data).hexdigest(),
        lines=len(lines),
        kinds={kind: len(records) for kind, records in stores},
    )


def trace_line(kind: str, index: int, record: Any) -> str:
    """One record as its line, newline included: the only form a reader
    accepts for it."""
    return json.dumps(
        {"fields": encode_field(record), "index": index, "kind": kind},
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ) + "\n"


def trace_record(lines: TraceLines | None, error: str | None) -> dict[str, Any]:
    """The trace file as the manifest vouches for it: its figures only once
    it was written, and nothing but ``error`` when it was not."""
    if lines is None or error is not None:
        return {
            "file": TRACE_NAME,
            "sha256": None,
            "lines": None,
            "kinds": None,
            "error": error,
        }
    return {
        "file": TRACE_NAME,
        "sha256": lines.sha256,
        "lines": lines.lines,
        "kinds": dict(lines.kinds),
        "error": None,
    }


def file_record(name: str, error: str | None) -> dict[str, Any]:
    """A file the manifest names, and why it lacks its content, if it does."""
    return {"file": name, "error": error}


def store_status(capture: TraceCapture) -> dict[str, Any]:
    """Every status field and derived verdict of every store, JSON-native.

    Read off each record type's fields and properties, so a field a store
    gains later is carried with no change here. The records themselves are
    the trace file's, so here they are counted, and a mapping becomes its
    [key, value] pairs, in order. Every value goes through ``encode_field``.
    """
    return {
        "journal": _status_of(capture.status),
        "commands": _status_of(capture.commands),
        "ledger": _status_of(capture.ledger),
        "complete": capture.complete,
    }


def _status_of(record: Any) -> dict[str, Any]:
    names = [field.name for field in dataclasses.fields(record)] + [
        name
        for name, member in vars(type(record)).items()
        if isinstance(member, property)
    ]
    return {name: _status_value(name, getattr(record, name)) for name in names}


def _status_value(name: str, value: Any) -> Any:
    if name in _RECORD_FIELDS:
        return len(value)
    if isinstance(value, Mapping):
        return [
            [encode_field(key), encode_field(item)]
            for key, item in value.items()
        ]
    return encode_field(value)


def evidence_manifest(
    capture: TraceCapture | None,
    *,
    trace: Mapping[str, Any],
    summary: Mapping[str, Any],
    identity: Mapping[str, Any] | None,
    finalization: Mapping[str, Any],
    error: str | None,
    held: int,
) -> dict[str, Any]:
    """The manifest: what the other files are, and what they came from.

    ``capture`` None means none was taken, and ``error`` says why; the
    manifest then claims no figure of any store. ``held`` is how many
    failures the writer held for raising when it made this manifest.
    """
    return {
        "schema": MANIFEST_SCHEMA,
        "version": MANIFEST_VERSION,
        "trace": dict(trace),
        "summary": dict(summary),
        "period_us": None if capture is None else capture.period_us,
        "capacity": None if capture is None else capture.status.capacity,
        "gate": {name: os.environ.get(name) for name in GATE_VARIABLES},
        "identity": None if identity is None else dict(identity),
        "finalization": dict(finalization),
        "stores": None if capture is None else store_status(capture),
        # D8's verdict is the reader's alone, never the writer's.
        "eligibility": None,
        "error": error,
        "held": held,
    }


__all__ = [
    "EVIDENCE_STEP",
    "GATE_VARIABLES",
    "KINDS",
    "KIND_COMMAND",
    "KIND_LEDGER",
    "KIND_PASS",
    "KIND_ROW",
    "MANIFEST_KEYS",
    "MANIFEST_NAME",
    "MANIFEST_SCHEMA",
    "MANIFEST_VERSION",
    "SOURCE_CLOSE_STEP",
    "SUMMARY_NAME",
    "TRACE_NAME",
    "TraceLines",
    "WORKER_STOPPED",
    "encode_field",
    "evidence_manifest",
    "file_record",
    "sealed_capture",
    "store_status",
    "trace_line",
    "trace_lines",
    "trace_record",
]
