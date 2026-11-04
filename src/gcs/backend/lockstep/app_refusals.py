"""Requests from the app that a lockstep run refuses (``deterministic_dev_sim.md`` section 5).

The coordinator's admission asks :func:`refusal` for each relay request; a
refused one never enters the run and is answered ``501``, journaled like
any answer. The table is the step-10 inventory's rows that simulation does
not support yet (``gcs_request_inventory.md``, "Sim today"):

- ``GET /api/vehicles/{sys_id}/parameters``: the full-parameter read runs
  MAVFTP on wall timers, and serial-step never answers MAVFTP (todo 245);
- the two zip downloads, stamped with wall time (todo 241);
- the app's NavPy-sim start and stop, wall-only until step 13.

Anything that is not an HTTP request with a method and a target is refused
too: the relay sends nothing else.
"""

from __future__ import annotations

import json
import re
from typing import Any
from urllib.parse import unquote, urlsplit

#: ``(method, path pattern, reason)``; the pattern matches the decoded path without its query.
REFUSED: tuple[tuple[str, re.Pattern[str], str], ...] = tuple(
    (method, re.compile(pattern), reason) for method, pattern, reason in (
        ("GET", r"/api/vehicles/[^/]+/parameters", "the full-parameter read uses MAVFTP (todo 245)"),
        ("GET", r"/api/diagnostics/export", "a zip download (todo 241)"),
        ("GET", r"/api/logs/download/[^/]+", "a zip download (todo 241)"),
        ("POST", r"/api/control/navpy-sim/start", "NavPy-sim start is wall-only (step 13)"),
        ("POST", r"/api/control/navpy-sim/stop/[^/]+", "NavPy-sim stop is wall-only (step 13)"),
    ))
NOT_HTTP = "only HTTP requests enter the run"
NOT_ASCII = "a request target is ASCII (percent-encoded)"
STATUS = 501


def answer(reason: str) -> dict[str, Any]:
    return {"status": STATUS, "body": json.dumps(
        {"detail": f"hardware-only, not supported in simulation: {reason}"})}


def refusal(payload: dict[str, Any]) -> dict[str, Any] | None:
    """The answer that refuses ``payload``, or None to admit it."""
    method, target = payload.get("method"), payload.get("target")
    if payload.get("kind") != "http" or not isinstance(method, str) or not isinstance(target, str):
        return answer(NOT_HTTP)
    if not target.isascii():
        return answer(NOT_ASCII)
    # Decoded once, as the backend's scope decodes it (``lockstep_asgi._scope``).
    path = unquote(urlsplit(target).path)
    # Starlette answers HEAD with the GET route's handler.
    method = "GET" if method.upper() == "HEAD" else method.upper()
    for refused_method, pattern, reason in REFUSED:
        if method == refused_method and pattern.fullmatch(path):
            return answer(reason)
    return None


__all__ = ["NOT_ASCII", "NOT_HTTP", "REFUSED", "STATUS", "answer", "refusal"]
