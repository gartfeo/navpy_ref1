"""Read a traced case back: the manifest, vouching for the trace file (D8.1).

``read_case`` reads the manifest and trace file (D7 of
the LANDING2 step-1 plan); ``read_evidence`` reads
their bytes. The manifest is held to its keys, in order, its
schema, its version and ``held``, which is required and never read as zero
(delivery step 6's review). It must claim no verdict. The trace file must be
the one it vouches for: written, with its sha256, its line count and its
count per kind. A manifest that cannot be read, a missing one included, is
refused, since nothing that failed while it was made or written can be
counted in it.

Then the capture is rebuilt: its records from the trace file, every other
figure from the manifest's store status, read as its declared type exactly.
Known count/latch contradictions are refused; this is not exhaustive. No store
may hold past the shared capacity, nor drop a record unless full
(``status_contradiction``); its latches pass ``validate_capture``; the ledger's
and command log's figures are ones their records allow (``contradiction``);
``store_status`` of the result is the manifest's, value for value, type for
type, in order. A figure no record can show false stays the manifest's word: a
failure, a drain, a callback fault, a refusal, a drop count. Nothing is
repaired: the first failure is the refusal (``EvidenceRefused``).

Not read here: D8's rules 2 to 13, which are ``determinism_eligibility``'s,
and the summary file, which no rule reads.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

from navpy.modules.vision.sim.determinism_admission_ledger import LedgerCapture
from navpy.modules.vision.sim.determinism_capture_validation import validate_capture
from navpy.modules.vision.sim.determinism_command_log import CommandCapture
from navpy.modules.vision.sim.determinism_evidence import (
    KIND_COMMAND,
    KIND_LEDGER,
    KIND_PASS,
    KIND_ROW,
    KINDS,
    MANIFEST_KEYS,
    MANIFEST_NAME,
    MANIFEST_SCHEMA,
    MANIFEST_VERSION,
    SUMMARY_NAME,
    TRACE_NAME,
    store_status,
)
from navpy.modules.vision.sim.determinism_row_log import (
    TraceStatus,
    status_contradiction,
)
from navpy.modules.vision.sim.determinism_trace import TraceCapture
from navpy.modules.vision.sim.determinism_trace_decode import (
    DecodedTrace,
    EvidenceRefused,
    decode_field,
    decode_trace,
    strict_json,
)

_TRACE_KEYS = ("file", "sha256", "lines", "kinds", "error")
_SUMMARY_KEYS = ("file", "error")
_STORE_KEYS = ("journal", "commands", "ledger", "complete")


@dataclass(frozen=True)
class CaseEvidence:
    """A case as read: its manifest, and the capture its files describe."""

    manifest: Mapping[str, Any]
    capture: TraceCapture


def read_case(directory: Path) -> CaseEvidence:
    """The case in ``directory``, from its manifest and its trace file.

    Raises EvidenceRefused, a missing or unreadable manifest included.
    """
    directory = Path(directory)
    manifest = _read(directory / MANIFEST_NAME, "the manifest")
    if manifest is None:
        raise EvidenceRefused(f"the manifest is missing: {MANIFEST_NAME}")
    return read_evidence(manifest, _read(directory / TRACE_NAME, "the trace file"))


def _read(path: Path, name: str) -> bytes | None:
    try:
        return path.read_bytes()
    except FileNotFoundError:
        return None
    except OSError as error:
        raise EvidenceRefused(f"{name} cannot be read: {error!r}") from None


def read_evidence(manifest_data: bytes, trace_data: bytes | None) -> CaseEvidence:
    """A case from its files' bytes, ``trace_data`` None when that file is
    missing. Raises EvidenceRefused (module docstring)."""
    try:
        manifest = _manifest(manifest_data)
        records = _records(manifest["trace"], trace_data, manifest["version"])
        return CaseEvidence(manifest=manifest, capture=_capture(manifest, records))
    except RecursionError:
        raise EvidenceRefused("a value is nested deeper than can be read") from None


def _require(condition: bool, refusal: str) -> None:
    if not condition:
        raise EvidenceRefused(refusal)


def _count(value: Any) -> bool:
    return type(value) is int and value >= 0


def _manifest(data: bytes) -> dict[str, Any]:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        raise EvidenceRefused("the manifest is not UTF-8") from None
    manifest = strict_json(text)
    _require(
        type(manifest) is dict and tuple(manifest) == MANIFEST_KEYS,
        f"the manifest's keys are not {MANIFEST_KEYS}",
    )
    _require(
        type(manifest["schema"]) is str and manifest["schema"] == MANIFEST_SCHEMA,
        f"the schema is not {MANIFEST_SCHEMA}",
    )
    _require(
        type(manifest["version"]) is int
        and manifest["version"] in (1, MANIFEST_VERSION),
        f"the version is neither 1 nor {MANIFEST_VERSION}",
    )
    _require(
        _count(manifest["held"]),
        "held is not a count of the failures the writer held for raising",
    )
    _require(
        manifest["eligibility"] is None,
        "the manifest claims a verdict, which only a reader gives",
    )
    if manifest["error"] is not None:
        raise EvidenceRefused(f"no capture was taken: {manifest['error']}")
    summary = manifest["summary"]
    _require(
        type(summary) is dict
        and tuple(summary) == _SUMMARY_KEYS
        and summary["file"] == SUMMARY_NAME
        and (summary["error"] is None or type(summary["error"]) is str),
        "the summary's record is not one",
    )
    return manifest


def _records(trace: Any, data: bytes | None, version: int) -> DecodedTrace:
    _require(
        type(trace) is dict
        and tuple(trace) == _TRACE_KEYS
        and trace["file"] == TRACE_NAME,
        "the trace file's record is not one",
    )
    if trace["error"] is not None:
        raise EvidenceRefused(f"the trace file was not written: {trace['error']}")
    if data is None:
        raise EvidenceRefused(f"the trace file is missing: {TRACE_NAME}")
    kinds = trace["kinds"]
    _require(
        _count(trace["lines"])
        and type(kinds) is dict
        and tuple(kinds) == KINDS
        and all(_count(count) for count in kinds.values()),
        "the trace file's counts are not counts of its kinds",
    )
    _require(
        hashlib.sha256(data).hexdigest() == trace["sha256"],
        "the trace file is not the one the manifest vouches for",
    )
    decoded = decode_trace(data, version=version)
    counted = {
        KIND_ROW: len(decoded.rows),
        KIND_PASS: len(decoded.passes),
        KIND_COMMAND: len(decoded.commands),
        KIND_LEDGER: len(decoded.ledger),
    }
    _require(
        sum(counted.values()) == trace["lines"],
        "the trace file's line count is not the manifest's",
    )
    _require(counted == kinds, "a kind's count is not the manifest's")
    return decoded


def _exact(kind: type) -> Callable[[Any], Any]:
    def read(value: Any) -> Any:
        _require(type(value) is kind, f"not a {kind.__name__}: {value!r}")
        return value

    return read


def _optional(read: Callable[[Any], Any]) -> Callable[[Any], Any]:
    return lambda value: None if value is None else read(value)


def _counted(value: Any) -> Any:
    _require(_count(value), f"not a count: {value!r}")
    return value


def _mapping(value: Any) -> Mapping[Any, Any]:
    _require(
        type(value) is list
        and all(type(pair) is list and len(pair) == 2 for pair in value),
        f"not [key, value] pairs: {value!r}",
    )
    return MappingProxyType(
        {decode_field(key): decode_field(item) for key, item in value}
    )


# How a store field is read from its figure, by the annotation it is declared
# with. A field declared any other way is refused, so a figure a store gains
# is not read as anything until the reader is taught it.
_FIELD_READERS: Mapping[str, Callable[[Any], Any]] = MappingProxyType({
    "int": _counted,
    "bool": _exact(bool),
    "int | None": _optional(_counted),
    "tuple | None": _optional(lambda value: _exact(tuple)(decode_field(value))),
    "Mapping[Any, tuple]": _mapping,
})


def _rebuilt(record_type: type, figures: Any, records: Mapping[str, Any]) -> Any:
    """One store's record: its records as read, every other field from its
    figure."""
    name = record_type.__name__
    _require(type(figures) is dict, f"no figures for the {name}")
    values: dict[str, Any] = {}
    for field in dataclasses.fields(record_type):
        if field.name in records:
            values[field.name] = records[field.name]
            continue
        read = _FIELD_READERS.get(str(field.type))
        _require(
            read is not None and field.name in figures,
            f"the {name}'s {field.name} cannot be read",
        )
        values[field.name] = read(figures[field.name])
    return record_type(**values)


def _capture(manifest: Mapping[str, Any], decoded: DecodedTrace) -> TraceCapture:
    stores = manifest["stores"]
    _require(type(manifest["period_us"]) is int, "period_us is not an int")
    _require(
        type(stores) is dict and tuple(stores) == _STORE_KEYS,
        "the stores' status is not one",
    )
    capture = TraceCapture(
        period_us=manifest["period_us"],
        rows=decoded.rows,
        status=_rebuilt(TraceStatus, stores["journal"], {}),
        commands=_rebuilt(
            CommandCapture,
            stores["commands"],
            {"entries": decoded.commands, "passes": decoded.passes},
        ),
        ledger=_rebuilt(LedgerCapture, stores["ledger"], {"entries": decoded.ledger}),
    )
    shared = capture.ledger.bounded, capture.commands.bounded
    wrong = status_contradiction(capture.status, capture.rows, shared)
    _require(wrong is None, f"the journal's figures are false: {wrong}")
    validate_capture(capture)
    for name, store in (("ledger", capture.ledger), ("command log", capture.commands)):
        wrong = store.contradiction()
        _require(wrong is None, f"the {name}'s figures are false: {wrong}")
    _require(
        capture.status.commands_incomplete == capture.commands.incomplete,
        "the journal's word on the command log is not the command log's",
    )
    _require(
        _counted(manifest["capacity"]) == capture.status.capacity,
        "the capacity is not the journal's",
    )
    _require(
        json.dumps(store_status(capture)) == json.dumps(stores),
        "the manifest's store figures are not what its records make",
    )
    return capture


__all__ = ["CaseEvidence", "read_case", "read_evidence"]
