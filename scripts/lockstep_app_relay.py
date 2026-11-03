"""The app relay of a lockstep run (``deterministic_dev_sim.md`` section 5, "A live relay").

It serves the GCS app on the slot's backend port, outside the baton, and
only forwards:

- ``/ws/telemetry``: the backend's broadcasts that the coordinator journals
  in each ``DONE`` (:meth:`AppRelay.broadcast`), sent to every open page.
  A new page first gets the latest vehicle state, merged from the
  broadcasts as the page merges them. Watching never enters the run.
  ``ping`` is answered here; ``manual_control`` is dropped and logged (the
  wire has no kind for it, todo 240).
- every other request: one ``{"kind": "http", ...}`` payload into the
  coordinator's admission (:class:`~navpy.sim.lockstep.coordinator_admission.RelayInbox`).
  The answer comes back by its ticket (:meth:`AppRelay.answer`), in any
  completion order, to the connection that asked. ``X-NavPy-Deadline-S``
  becomes the request's sim-time ``deadline_s``; no other header and no
  non-JSON body is carried (step 10: no route needs one).

A page that closed or reloaded gets nothing: the relay logs it and sends no
cancel, as a disconnect is not an operator action. Only an explicit
``POST /__relay/cancel/{X-GCS-Request-ID}`` of a pending request is a
cancel. The relay keeps no timer, retry or batching window: what it sends
the run is what it received, when the coordinator takes it.
"""

from __future__ import annotations

import asyncio
import itertools
import json
import logging
import math
import threading
from collections.abc import Callable
from typing import Any

import uvicorn

from navpy.sim.lockstep.coordinator_admission import RelayCancel, RelayInbox, RelayRequest

DEADLINE_HEADER = b"x-navpy-deadline-s"
CLIENT_ID_HEADER = b"x-gcs-request-id"
CANCEL_PREFIX = "/__relay/cancel/"
#: Messages queued for one page before the relay closes it (a page that stopped reading).
PAGE_QUEUE = 1000
#: HTTP status of each coordinator or backend outcome that is not a response.
CANCELLED_STATUS, DEADLINE_STATUS, ENDED_STATUS = 499, 504, 503

_log = logging.getLogger(__name__)


def http_reply(answer: object) -> tuple[int, bytes]:
    """The HTTP status and JSON body of one answer from the run."""
    if isinstance(answer, dict) and isinstance(answer.get("status"), int) \
            and isinstance(answer.get("body"), str):
        return answer["status"], answer["body"].encode("utf-8")
    if answer == {"cancelled": True}:
        return CANCELLED_STATUS, b'{"detail":"cancelled"}'
    if answer == {"deadline": True}:
        return DEADLINE_STATUS, b'{"detail":"sim-time deadline passed"}'
    return 502, json.dumps({"detail": f"unexpected answer {answer!r}"}).encode("utf-8")


class TelemetryState:
    """The latest vehicle state, merged as ``telemetryStore.js`` merges broadcasts."""

    def __init__(self) -> None:
        self._vehicles: dict[Any, dict] = {}
        self._last: dict[str, Any] | None = None

    def observe(self, message: dict[str, Any]) -> None:
        if message.get("type") != "telemetry" or not isinstance(message.get("vehicles"), list):
            return
        for vehicle in message["vehicles"]:
            self._vehicles[vehicle.get("sys_id")] = {**self._vehicles.get(vehicle.get("sys_id"), {}),
                                                     **vehicle}
        if isinstance(message.get("active_ids"), list):
            active = set(message["active_ids"])
            self._vehicles = {key: value for key, value in self._vehicles.items() if key in active}
        self._last = message

    def snapshot(self) -> str | None:
        if self._last is None:
            return None
        return json.dumps({**self._last, "vehicles": list(self._vehicles.values())})


class AppRelay:
    def __init__(self, target: int, *, inbox: RelayInbox | None = None) -> None:
        self.target = target
        self.inbox = inbox or RelayInbox()
        self._tickets = itertools.count(1)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        # Loop-thread state only.
        self._pending: dict[int, asyncio.Future] = {}
        self._gone: set[int] = set()
        self._client_ids: dict[str, int] = {}
        self._pages: dict[int, asyncio.Queue] = {}
        self._page_ids = itertools.count(1)
        self._telemetry = TelemetryState()
        self._ended = False
        self.port: int | None = None
        #: ``(event, detail)`` of what the relay did not forward (host log, tests).
        self.notes: list[tuple[str, object]] = []

    # -- the coordinator's side (RelayPort) ------------------------------------
    def take(self) -> tuple:
        return self.inbox.take()

    def answer(self, ticket: int, answer: object) -> None:
        self._call(self._resolve, ticket, answer)

    def broadcast(self, data: str) -> None:
        self._call(self._fan_out, data)

    def _call(self, function: Callable[..., None], *args: object) -> None:
        loop = self._loop
        if loop is None:
            return
        try:
            loop.call_soon_threadsafe(function, *args)
        except RuntimeError:  # the loop closed: the run outlived the relay
            pass

    # -- lifetime --------------------------------------------------------------
    def start(self, port: int, host: str = "127.0.0.1") -> None:
        config = uvicorn.Config(self, host=host, port=port, lifespan="off", log_level="warning",
                                ws="websockets-sansio")
        self._server = uvicorn.Server(config)
        self._loop = asyncio.new_event_loop()
        started = threading.Event()

        def serve() -> None:
            asyncio.set_event_loop(self._loop)
            self._loop.call_soon(started.set)
            try:
                self._loop.run_until_complete(self._server.serve())
            finally:
                self._loop.close()

        self._thread = threading.Thread(target=serve, name="lockstep-app-relay", daemon=True)
        self._thread.start()
        started.wait()
        while not self._server.started:
            if not self._thread.is_alive():
                raise RuntimeError(f"the app relay could not serve port {port}")
            self._thread.join(0.05)
        self.port = self._server.servers[0].sockets[0].getsockname()[1]  # port 0: the one bound

    def stop(self) -> None:
        """Answer every open request ``503`` (the run ended), close the pages, stop serving."""
        if self._thread is None:
            return
        done = threading.Event()
        self._call(self._end, done)
        done.wait(5.0)
        self._server.should_exit = True
        self._thread.join(10.0)
        self._thread = None

    def _end(self, done: threading.Event) -> None:
        self._ended = True
        for ticket in list(self._pending):
            self._resolve(ticket, {"status": ENDED_STATUS, "body": '{"detail":"lockstep run ended"}'})
        for page in self._pages.values():
            _close(page)
        done.set()

    # -- the loop's side -------------------------------------------------------
    def _resolve(self, ticket: int, answer: object) -> None:
        future = self._pending.pop(ticket, None)
        if ticket in self._gone:
            self._gone.discard(ticket)
            self.notes.append(("unreceived", ticket))
            _log.info("app relay: answer for request %s after its page closed", ticket)
        elif future is not None and not future.done():
            future.set_result(answer)
        for client_id, owned in list(self._client_ids.items()):
            if owned == ticket:
                del self._client_ids[client_id]

    def _fan_out(self, data: str) -> None:
        try:
            self._telemetry.observe(json.loads(data))
        except ValueError:
            pass
        for page_id, page in list(self._pages.items()):
            try:
                page.put_nowait(data)
            except asyncio.QueueFull:
                self.notes.append(("page_overflow", page_id))
                self._pages.pop(page_id, None)
                _close(page)

    async def __call__(self, scope: dict, receive: Callable, send: Callable) -> None:
        if scope["type"] == "websocket":
            await self._websocket(scope, receive, send)
        elif scope["type"] == "http":
            await self._http(scope, receive, send)

    async def _http(self, scope: dict, receive: Callable, send: Callable) -> None:
        method, path = scope["method"], scope["path"]
        if method == "POST" and path.startswith(CANCEL_PREFIX):
            await self._cancel(path[len(CANCEL_PREFIX):], send)
            return
        body = b""
        while True:
            message = await receive()
            if message["type"] == "http.disconnect":
                return
            body += message.get("body", b"")
            if not message.get("more_body"):
                break
        query = scope.get("query_string", b"").decode("latin-1")
        payload: dict[str, Any] = {"kind": "http", "method": method,
                                   "target": path + (f"?{query}" if query else "")}
        headers = dict(scope.get("headers", []))
        try:
            if body:
                payload["json"] = json.loads(body)
            if DEADLINE_HEADER in headers:
                deadline = float(headers[DEADLINE_HEADER])
                if not (math.isfinite(deadline) and deadline > 0):
                    raise ValueError(f"deadline {deadline}")
                payload["deadline_s"] = deadline
        except ValueError as error:  # never sent to the run
            await _reply(send, 400, json.dumps({"detail": f"not carried into the run: {error}"}).encode())
            return
        if self._ended:
            await _reply(send, ENDED_STATUS, b'{"detail":"lockstep run ended"}')
            return
        ticket = next(self._tickets)
        future = asyncio.get_running_loop().create_future()
        self._pending[ticket] = future
        client_id = headers.get(CLIENT_ID_HEADER)
        if client_id:
            self._client_ids[client_id.decode("latin-1")] = ticket
        self.inbox.submit(RelayRequest(ticket, payload))
        gone = asyncio.ensure_future(_disconnected(receive))
        await asyncio.wait({future, gone}, return_when=asyncio.FIRST_COMPLETED)
        if future.done():
            gone.cancel()
            status, data = http_reply(future.result())
            await _reply(send, status, data)
            return
        self._gone.add(ticket)  # a disconnect is not an operator action: no cancel
        self.notes.append(("disconnected", ticket))
        _log.info("app relay: page closed during request %s; no cancel sent", ticket)

    async def _cancel(self, client_id: str, send: Callable) -> None:
        ticket = self._client_ids.get(client_id)
        if ticket is None or ticket not in self._pending:
            await _reply(send, 404, b'{"detail":"no pending request with that id"}')
            return
        self.inbox.submit(RelayCancel(ticket))
        await _reply(send, 202, b'{"cancel":"queued"}')

    async def _websocket(self, scope: dict, receive: Callable, send: Callable) -> None:
        message = await receive()
        if message["type"] != "websocket.connect" or scope["path"] != "/ws/telemetry" or self._ended:
            await send({"type": "websocket.close", "code": 1008})
            return
        await send({"type": "websocket.accept"})
        page_id, page = next(self._page_ids), asyncio.Queue(PAGE_QUEUE)
        snapshot = self._telemetry.snapshot()
        if snapshot is not None:
            page.put_nowait(snapshot)
        self._pages[page_id] = page
        reader = asyncio.ensure_future(self._read_page(page_id, page, receive))
        try:
            while (data := await page.get()) is not None:
                await send({"type": "websocket.send", "text": data})
            await send({"type": "websocket.close", "code": 1001})
        except OSError:
            pass
        finally:
            reader.cancel()
            self._pages.pop(page_id, None)

    async def _read_page(self, page_id: int, page: asyncio.Queue, receive: Callable) -> None:
        while True:
            message = await receive()
            if message["type"] == "websocket.disconnect":
                self._pages.pop(page_id, None)
                _close(page)
                return
            text = message.get("text")
            if text == "ping":
                try:
                    page.put_nowait("pong")
                except asyncio.QueueFull:
                    pass  # the overflow closes the page
                continue
            try:
                kind = json.loads(text or "").get("type")
            except (ValueError, AttributeError):
                kind = None
            self.notes.append(("ws_dropped", kind))
            _log.info("app relay: WebSocket message %r not carried into the run (todo 240)", kind)


def _close(page: asyncio.Queue) -> None:
    """End a page's sender: its queue's last item is None."""
    while True:
        try:
            page.put_nowait(None)
            return
        except asyncio.QueueFull:
            page.get_nowait()


async def _disconnected(receive: Callable) -> None:
    while (await receive())["type"] != "http.disconnect":
        pass


async def _reply(send: Callable, status: int, body: bytes) -> None:
    await send({"type": "http.response.start", "status": status,
                "headers": [(b"content-type", b"application/json"),
                            (b"content-length", str(len(body)).encode())]})
    await send({"type": "http.response.body", "body": body})


__all__ = ["AppRelay", "CANCEL_PREFIX", "CANCELLED_STATUS", "DEADLINE_HEADER", "DEADLINE_STATUS",
           "ENDED_STATUS", "TelemetryState", "http_reply"]
