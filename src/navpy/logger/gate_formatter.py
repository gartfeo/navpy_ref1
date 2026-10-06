"""Pure formatters for ObservableGate results into CSV-safe strings.

Avoids raw repr / object dumping. Sanitizes characters that would break
the event-payload encoding (; = , CR LF) inside gate names and reasons.
"""
from __future__ import annotations

import re
from typing import Sequence, Tuple

from navpy.modules.navigation.gates.observable_gate import GateResult, GateStatus


_UNSAFE = re.compile(r"[;=,\r\n]")


def sanitize(token: str) -> str:
    """Replace CSV/payload-unsafe characters with underscores."""
    if not token:
        return ""
    return _UNSAFE.sub("_", token)


def format_gate_result(result: GateResult) -> str:
    """Render a single GateResult as a compact string.

    Shape:
      PASS              -> "PASS"
      BLOCK / INVALID   -> "STATUS:sanitized_reason"
    """
    if result.status == GateStatus.PASS:
        return "PASS"
    reason = sanitize(result.reason) if result.reason is not None else ""
    return f"{result.status.value}:{reason}"


def format_gate_results_list(
    results: Sequence[Tuple[str, GateResult]],
) -> str:
    """Render an ordered list of (name, GateResult) pairs.

    Shape:  gate_a=PASS;gate_b=BLOCK:NOT_READY;gate_c=INVALID:NO_SIGNAL
    Gate names and reasons are sanitized against ';=,\\r\\n'.
    """
    return ";".join(
        f"{sanitize(name)}={format_gate_result(result)}"
        for name, result in results
    )
