"""Strict token parsing shared by the eval's CSV evidence parsers."""

from __future__ import annotations

import math
import re


def seconds_of_day(value: str) -> float:
    match = re.fullmatch(r"(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?", value.strip())
    if match is None:
        raise ValueError(f"invalid log timestamp {value!r}")
    hour, minute, second = (int(match.group(index)) for index in range(1, 4))
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError(f"invalid log timestamp {value!r}")
    fraction = float(f"0.{match.group(4)}") if match.group(4) else 0.0
    return hour * 3600.0 + minute * 60.0 + second + fraction


def payload_fields(
    payload: str,
    fields: tuple[str, ...],
    event: str,
) -> dict[str, str]:
    pairs: list[tuple[str, str]] = []
    for token in payload.split(";"):
        if token.count("=") != 1:
            raise ValueError(f"malformed {event} token {token!r}")
        key, value = token.split("=", 1)
        pairs.append((key, value))
    keys = tuple(key for key, _value in pairs)
    if keys != fields:
        raise ValueError(f"{event} fields/order must be {fields}, got {keys}")
    return dict(pairs)


def canonical_integer(name: str, value: str, *, allow_zero: bool) -> int:
    if re.fullmatch(r"0|[1-9]\d*", value) is None:
        raise ValueError(f"{name} must be a canonical integer")
    number = int(value)
    if not allow_zero and number == 0:
        raise ValueError(f"{name} must be positive")
    return number


def finite_number(name: str, value: str, *, optional: bool = False) -> float | None:
    if optional and value == "":
        return None
    try:
        number = float(value)
    except ValueError as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def exact_boolean(name: str, value: str) -> bool:
    if value == "True":
        return True
    if value == "False":
        return False
    raise ValueError(f"{name} must be exactly True or False")


__all__ = [
    "canonical_integer",
    "exact_boolean",
    "finite_number",
    "payload_fields",
    "seconds_of_day",
]
