"""Read the trace file back, strictly: every record, or a refusal.

The inverse of ``determinism_evidence.trace_lines``, for the offline reader
(D7 and D8.1 of the LANDING2 step-1 plan). Each
line is decoded, checked against the layout its kind and its event give it
(``determinism_events``), and then made again with ``trace_line``: unless the
two are the same text, the line is refused. So a file is read only if it is
exactly what the writer makes from some records, and what the reader gets
back is those records. Nothing is repaired: a refusal names the first line
that fails, and why.

JSON is read strictly (``strict_json``). NaN and the infinities, which
Python's reader accepts, are refused, and so is a number it would read as
infinite, 1e999, and a key given twice, which it would quietly collapse to
the last. So is a line nested deeper than can be read.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTCOMES,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
    EVENT_VIOLATION,
)
from navpy.modules.vision.sim.determinism_evidence import (
    KIND_COMMAND,
    KIND_LEDGER,
    KIND_PASS,
    KIND_ROW,
    KINDS,
    trace_line,
)
from navpy.modules.vision.sim.determinism_truth_sample import valid_record


class EvidenceRefused(ValueError):
    """The evidence cannot be read as what it says it is (D8.1)."""


@dataclass(frozen=True)
class DecodedTrace:
    """The trace file's records, by kind, each in file order."""

    rows: tuple[tuple, ...]
    passes: tuple[tuple, ...]
    commands: tuple[tuple, ...]
    ledger: tuple[tuple, ...]


def strict_json(text: str) -> Any:
    """``json.loads``, refusing NaN, the infinities, a number too large to
    be finite and a key given twice.

    Raises EvidenceRefused for anything that is not strict JSON.
    """
    try:
        return json.loads(
            text,
            parse_constant=_no_constant,
            parse_float=_finite,
            object_pairs_hook=_unique,
        )
    except EvidenceRefused:
        raise
    except (ValueError, RecursionError) as error:
        raise EvidenceRefused(f"not JSON: {error}") from error


def _no_constant(name: str) -> Any:
    raise EvidenceRefused(f"{name} is not strict JSON")


def _finite(text: str) -> float:
    """A number read as Python reads it, unless it is too large to be
    finite: Python's reader takes 1e999 as infinity."""
    number = float(text)
    if not math.isfinite(number):
        raise EvidenceRefused(f"{text} is not a finite number")
    return number


def _unique(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = dict(pairs)
    if len(result) != len(pairs):
        raise EvidenceRefused("a key is given twice")
    return result


def decode_field(value: Any) -> Any:
    """The recorded value that ``encode_field`` made ``value`` from.

    Raises EvidenceRefused for JSON no record encodes to, a float among it.
    """
    kind = type(value)
    if value is None or kind is bool or kind is int or kind is str:
        return value
    if kind is list:
        return tuple(decode_field(item) for item in value)
    if kind is dict and list(value) == ["hex"] and type(value["hex"]) is str:
        try:
            return bytes.fromhex(value["hex"])
        except ValueError:
            raise EvidenceRefused(f"not hex: {value['hex']!r}") from None
    raise EvidenceRefused(f"no record holds a {kind.__name__}")


def _int(value: Any) -> bool:
    return type(value) is int


def _optional_int(value: Any) -> bool:
    return value is None or type(value) is int


def _optional_bytes(value: Any) -> bool:
    return value is None or type(value) is bytes


def _optional_str(value: Any) -> bool:
    return value is None or type(value) is str


def _bool(value: Any) -> bool:
    return type(value) is bool


def _pair(value: Any) -> bool:
    """A VIOLATION's detail: the stamp that stalled, the watermark it met."""
    return (
        type(value) is tuple
        and len(value) == 2
        and all(type(item) is int for item in value)
    )


_Check = Callable[[Any], bool]
# What each layout holds after its event, epoch and outcome, as
# ``determinism_events`` documents it.
_ROW_TAILS: MappingProxyType[str, tuple[_Check, ...]] = MappingProxyType({
    EVENT_ASSOCIATION: (_optional_int,) * 3,
    EVENT_TRUTH: (_optional_int,) * 2,
    EVENT_STAGE: (_optional_int,) * 4 + (_optional_bytes,),
    EVENT_DECIMATE: (_optional_int,) * 4,
    EVENT_OUTPUT: (_optional_int,) * 5,
    EVENT_LIFECYCLE: (_int, _optional_int, _optional_int),
    EVENT_SUBSCRIPTION: (_optional_int,),
    EVENT_VIOLATION: (_pair,),
})
# (iteration, observed, admitted); (iteration, digest); and
# (ruling, time_boot_ms, accepted, reason).
_PASS = (_int, _optional_int, _optional_int)
_COMMAND = (_int, _optional_bytes)
_LEDGER = (_int, _optional_int, _bool, _optional_str)


def _fits(record: Any, checks: tuple[_Check, ...]) -> bool:
    return (
        type(record) is tuple
        and len(record) == len(checks)
        and all(check(field) for check, field in zip(checks, record))
    )


def _row_fits(record: Any) -> bool:
    if type(record) is not tuple or len(record) < 3:
        return False
    event, epoch, outcome = record[:3]
    if type(event) is not str or event not in _ROW_TAILS:
        return False
    tail = record[3:]
    if event == EVENT_TRUTH and len(record) == 6:
        if not valid_record(record[5]):
            return False
        tail = record[3:5]
    return (
        type(epoch) is int
        and type(outcome) is str
        and outcome in EVENT_OUTCOMES[event]
        and _fits(tail, _ROW_TAILS[event])
    )


_FITS: MappingProxyType[str, _Check] = MappingProxyType({
    KIND_ROW: _row_fits,
    KIND_PASS: lambda record: _fits(record, _PASS),
    KIND_COMMAND: lambda record: _fits(record, _COMMAND),
    KIND_LEDGER: lambda record: _fits(record, _LEDGER),
})


def decode_trace(data: bytes, *, version: int | None = None) -> DecodedTrace:
    """Every record the trace file holds, by kind.

    Raises EvidenceRefused for a file that is not exactly what
    ``trace_lines`` makes from some records. A supplied version additionally
    pins TRUTH's layout. Standalone decoding without it accepts either layout.
    """
    if version is not None and (type(version) is not int or version not in (1, 2)):
        raise EvidenceRefused("unsupported evidence version")
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        raise EvidenceRefused("the trace file is not ASCII") from None
    if text and not text.endswith("\n"):
        raise EvidenceRefused("the trace file ends mid-line")
    records: dict[str, list[Any]] = {kind: [] for kind in KINDS}
    current = 0  # the kind being read: a line never goes back to an earlier one
    for number, line in enumerate(text.split("\n")[:-1]):
        kind, index, record = _decoded(line + "\n", number)
        position = KINDS.index(kind)
        if position < current:
            raise EvidenceRefused(
                f"line {number}: a {kind} record after the {KINDS[current]}s"
            )
        current = position
        if index != len(records[kind]):
            raise EvidenceRefused(
                f"line {number}: {kind} {index}, where "
                f"{len(records[kind])} was next"
            )
        if not _FITS[kind](record):
            raise EvidenceRefused(f"line {number}: not a {kind}: {record!r}")
        if version is not None and kind == KIND_ROW and record[0] == EVENT_TRUTH:
            if len(record) != (5 if version == 1 else 6):
                raise EvidenceRefused(f"line {number}: TRUTH row does not match the evidence version")
        records[kind].append(record)
    return DecodedTrace(*(tuple(records[kind]) for kind in KINDS))


def _decoded(line: str, number: int) -> tuple[str, int, Any]:
    """One line's kind, index and record, or a refusal naming the line."""
    try:
        value = strict_json(line)
        if type(value) is not dict or set(value) != {"fields", "index", "kind"}:
            raise EvidenceRefused("not a trace line")
        kind, index = value["kind"], value["index"]
        if type(kind) is not str or kind not in KINDS or type(index) is not int:
            raise EvidenceRefused(f"no kind {kind!r}, or no index {index!r}")
        record = decode_field(value["fields"])
        if trace_line(kind, index, record) != line:
            raise EvidenceRefused("not the line its record makes")
    except EvidenceRefused as error:
        raise EvidenceRefused(f"line {number}: {error}") from None
    except RecursionError:
        raise EvidenceRefused(
            f"line {number}: nested deeper than can be read"
        ) from None
    return kind, index, record


__all__ = [
    "DecodedTrace",
    "EvidenceRefused",
    "decode_field",
    "decode_trace",
    "strict_json",
]
