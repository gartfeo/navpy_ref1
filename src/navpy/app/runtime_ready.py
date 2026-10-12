"""Machine-readable readiness signal for GCS-managed NavPy processes."""

from __future__ import annotations

import json
import os
from collections.abc import Mapping


NAVPY_RUNTIME_READY_MARKER = "NAVPY_RUNTIME_READY_V1"


def emit_runtime_ready(payload: Mapping[str, object]) -> None:
    """Publish one flushed stdout record after runtime startup health passes."""
    ready = dict(payload)
    ready["process_pid"] = os.getpid()
    encoded = json.dumps(ready, sort_keys=True, separators=(",", ":"))
    print(f"{NAVPY_RUNTIME_READY_MARKER} {encoded}", flush=True)


def parse_runtime_ready(line: str) -> dict[str, object] | None:
    """Parse an exact readiness record; ignore ordinary or near-match output."""
    prefix = f"{NAVPY_RUNTIME_READY_MARKER} "
    if not isinstance(line, str) or not line.startswith(prefix):
        return None
    try:
        payload = json.loads(line[len(prefix):])
    except (TypeError, ValueError):
        return None
    return payload if isinstance(payload, dict) else None


__all__ = [
    "NAVPY_RUNTIME_READY_MARKER",
    "emit_runtime_ready",
    "parse_runtime_ready",
]
