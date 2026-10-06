"""Small validation and canonicalization helpers for certificate modules."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any


def finite_number(value: Any) -> float | None:
    """Return a finite real value, excluding booleans, or ``None``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def require_finite(
    value: Any,
    *,
    name: str,
    minimum: float | None = None,
    maximum: float | None = None,
    minimum_inclusive: bool = True,
) -> float:
    number = finite_number(value)
    if number is None:
        raise TypeError(f"{name} must be a finite number, not {value!r}")
    if minimum is not None:
        too_small = number < minimum if minimum_inclusive else number <= minimum
        if too_small:
            operator = ">=" if minimum_inclusive else ">"
            raise ValueError(f"{name} must be {operator} {minimum:g}, not {number:g}")
    if maximum is not None and number > maximum:
        raise ValueError(f"{name} must be <= {maximum:g}, not {number:g}")
    return number


def canonical_digest(payload: Any) -> str:
    """Return a stable SHA-256 over strict canonical JSON."""
    text = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
        allow_nan=False,
    )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def failed_reasons(**named: str) -> dict[str, str]:
    return {name: reason for name, reason in named.items() if reason}
