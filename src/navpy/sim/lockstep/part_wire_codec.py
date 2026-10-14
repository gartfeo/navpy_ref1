"""Strict canonical-JSON bodies for the ``NVPART01`` wire.

A body is UTF-8 JSON with sorted keys, no whitespace, ASCII only and no
NaN or infinity: exactly one byte string per value, so golden vectors pin
it. Decoding is strict: a duplicate key, a non-canonical spelling, a
missing or extra key, or a value of the wrong type is a protocol error.
Bytes travel as lowercase hex; an id is ``[origin ordinal, origin sequence]``.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from typing import Any

#: An id ``(origin ordinal, origin sequence)``; the operator is ordinal 0.
RequestId = tuple[int, int]


class PartWireError(ValueError):
    """A protocol error on ``NVPART01``: a fault, never a different outcome."""


def dumps(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False,
    ).encode("ascii")


def loads(body: bytes) -> Any:
    try:
        value = json.loads(
            body.decode("ascii"), object_pairs_hook=_unique_keys, parse_constant=_no_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PartWireError(f"malformed body: {error}") from error
    if dumps(value) != body:
        raise PartWireError("body is not canonical JSON")
    return value


def _unique_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    keys = [key for key, _ in pairs]
    if len(set(keys)) != len(keys):
        raise PartWireError(f"duplicate key in {keys}")
    return dict(pairs)


def _no_constant(name: str) -> None:
    raise PartWireError(f"{name} is not allowed")


def fields(value: object, names: Sequence[str], what: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PartWireError(f"{what} must be an object")
    if set(value) != set(names):
        raise PartWireError(f"{what} keys {sorted(value)}, expected {sorted(names)}")
    return value


def integer(value: object, what: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise PartWireError(f"{what} must be an integer >= {minimum}, got {value!r}")
    return value


def text(value: object, what: str) -> str:
    if not isinstance(value, str):
        raise PartWireError(f"{what} must be a string")
    return value


def flag(value: object, what: str) -> bool:
    if type(value) is not bool:
        raise PartWireError(f"{what} must be a boolean")
    return value


def hex_bytes(value: object, what: str) -> bytes:
    data = text(value, what)
    try:
        decoded = bytes.fromhex(data)
    except ValueError as error:
        raise PartWireError(f"{what} is not hex") from error
    if decoded.hex() != data:  # one spelling only: lowercase, no whitespace
        raise PartWireError(f"{what} is not canonical lowercase hex")
    return decoded


def request_id(value: object, what: str = "id") -> RequestId:
    if not isinstance(value, list) or len(value) != 2:
        raise PartWireError(f"{what} must be [ordinal, sequence]")
    return integer(value[0], f"{what} ordinal"), integer(value[1], f"{what} sequence", minimum=1)


def optional_id(value: object, what: str = "id") -> RequestId | None:
    return None if value is None else request_id(value, what)


def items(value: object, what: str, decode: Callable[[object], Any]) -> tuple[Any, ...]:
    if not isinstance(value, list):
        raise PartWireError(f"{what} must be a list")
    return tuple(decode(item) for item in value)


def in_id_order(ids: Sequence[RequestId], what: str) -> None:
    """Every id list is strictly ascending: id order, no duplicates."""
    if any(later <= earlier for earlier, later in zip(ids, ids[1:])):
        raise PartWireError(f"{what} not in strict id order: {list(ids)}")


__all__ = [
    "PartWireError", "RequestId", "dumps", "fields", "flag", "hex_bytes", "in_id_order",
    "integer", "items", "loads", "optional_id", "request_id", "text",
]
