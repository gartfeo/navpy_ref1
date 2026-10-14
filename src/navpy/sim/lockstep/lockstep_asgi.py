"""The in-process ASGI call: one HTTP request through an app, no server, no sockets.

``lockstep_slice.md``, "Backend participant", Requests:

- the target splits into the path (percent-decoded, as a server passes it)
  and the raw query string;
- headers pass as given; a JSON body is sent as ``application/json``;
- the status and body pass through, error statuses too: an app that raises
  after it started a response (Starlette's server-error middleware sends
  the 500 first) answers with what it sent;
- a sim-time deadline (``asyncio.timeout`` on the loop's runtime clock)
  raises ``TimeoutError``; a cancel of the awaiting task propagates as
  ``CancelledError``;
- the wire request id rides in the scope's ``extensions``.

A request delivered at tick k runs from tick k; its response is the
``DONE`` of the tick its handler finishes in.
"""

from __future__ import annotations

import asyncio
import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any
from urllib.parse import unquote, urlsplit

from navpy.sim.lockstep.part_wire_codec import RequestId

#: The ``extensions`` key carrying the lockstep request facts.
EXTENSION = "navpy.lockstep"
JSON_CONTENT_TYPE = "application/json"
_NO_JSON = object()


@dataclass(frozen=True)
class AsgiRequest:
    method: str
    #: Path with an optional ``?query``.
    target: str
    headers: tuple[tuple[str, str], ...] = ()
    #: A JSON body, sent as ``application/json`` (``body`` must then be empty).
    json: Any = _NO_JSON
    body: bytes = b""
    #: Sim seconds the request may take, from its start; None for no deadline.
    deadline_s: float | None = None
    request_id: RequestId | None = None

    def encoded_body(self) -> tuple[bytes, tuple[tuple[str, str], ...]]:
        if self.json is _NO_JSON:
            return self.body, self.headers
        if self.body:
            raise ValueError("a request has a JSON body or raw bytes, not both")
        body = json.dumps(self.json, allow_nan=False).encode("utf-8")
        return body, self.headers + (("content-type", JSON_CONTENT_TYPE),)


@dataclass(frozen=True)
class AsgiResponse:
    status: int
    headers: tuple[tuple[str, str], ...]
    body: bytes

    def header(self, name: str) -> str | None:
        wanted = name.lower()
        return next((value for key, value in self.headers if key.lower() == wanted), None)


def _scope(request: AsgiRequest, body: bytes, headers: tuple[tuple[str, str], ...]) -> dict:
    parts = urlsplit(request.target)
    raw_headers = [(b"host", b"lockstep")] + [
        (name.lower().encode("latin-1"), value.encode("latin-1")) for name, value in headers
    ]
    if body:
        raw_headers.append((b"content-length", str(len(body)).encode("ascii")))
    lockstep = {"request_id": None if request.request_id is None else list(request.request_id)}
    return {
        "type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1",
        "method": request.method.upper(), "scheme": "http", "path": unquote(parts.path),
        "raw_path": parts.path.encode("ascii"), "query_string": parts.query.encode("ascii"),
        "root_path": "", "headers": raw_headers, "client": ("lockstep", 0),
        "server": ("lockstep", 80), "extensions": {EXTENSION: lockstep},
    }


async def _call(app: Callable[..., Any], scope: dict, body: bytes) -> AsgiResponse:
    sent = {"body": False}
    status, headers, chunks = 0, [], []

    async def receive() -> dict:
        if not sent["body"]:
            sent["body"] = True
            return {"type": "http.request", "body": body, "more_body": False}
        await asyncio.Event().wait()  # no disconnect in lockstep

    async def send(message: dict) -> None:
        nonlocal status
        if message["type"] == "http.response.start":
            status = message["status"]
            headers.extend((k.decode("latin-1"), v.decode("latin-1"))
                           for k, v in message.get("headers", ()))
        elif message["type"] == "http.response.body":
            chunks.append(message.get("body", b""))

    try:
        await app(scope, receive, send)
    except Exception:
        if not status:  # nothing was sent: a fault, not a response
            raise
    return AsgiResponse(status, tuple(headers), b"".join(chunks))


async def asgi_request(app: Callable[..., Any], request: AsgiRequest) -> AsgiResponse:
    """Run ``request`` through ``app``; see the module docstring."""
    body, headers = request.encoded_body()
    scope = _scope(request, body, headers)
    if request.deadline_s is None:
        return await _call(app, scope, body)
    async with asyncio.timeout(request.deadline_s):
        return await _call(app, scope, body)


__all__ = ["AsgiRequest", "AsgiResponse", "EXTENSION", "JSON_CONTENT_TYPE", "asgi_request"]
