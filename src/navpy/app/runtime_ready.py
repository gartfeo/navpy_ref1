"""Machine-readable readiness signal for GCS-managed NavPy processes.

The record names the process by its identity fields: ``process_pid`` from the
OS by default. A lockstep participant sets its identity instead
(``{ordinal, generation}``, with ``process_pid`` the ordinal), so the record
repeats run to run.
"""

from __future__ import annotations

import json
import os
from collections.abc import Callable, Mapping


NAVPY_RUNTIME_READY_MARKER = "NAVPY_RUNTIME_READY_V1"

ProcessIdentity = Callable[[], Mapping[str, object]]


def os_process_identity() -> Mapping[str, object]:
    """Wall mode: the OS process id."""
    return {"process_pid": os.getpid()}


_identity: ProcessIdentity = os_process_identity


def set_process_identity(identity: ProcessIdentity) -> None:
    """Name this process in readiness records by ``identity``'s fields."""
    global _identity
    _identity = identity


def emit_runtime_ready(payload: Mapping[str, object]) -> None:
    """Publish one flushed stdout record after runtime startup health passes."""
    ready = dict(payload)
    ready.update(_identity())
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
    "ProcessIdentity",
    "emit_runtime_ready",
    "os_process_identity",
    "parse_runtime_ready",
    "set_process_identity",
]
