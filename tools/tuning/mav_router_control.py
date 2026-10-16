"""The router's loopback control port: JSON {op: close|open, client, id} requests for the endpoint barrier
(mav_router.py; design hitl_final_approach.md section 2)."""
from __future__ import annotations

import json
import socket
import threading
from typing import Callable

from tools.tuning.mp_mirror import LOOPBACK, MAX_DATAGRAM

EndpointOp = Callable[[str], dict]


class ControlServer:
    """Answers each request datagram to its sender; close_endpoint/open_endpoint do the work."""

    def __init__(self, port: int, clients: set[str], close_endpoint: EndpointOp, open_endpoint: EndpointOp) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LOOPBACK, port))
        self._clients = clients
        self._ops: dict[str, EndpointOp] = {"close": close_endpoint, "open": open_endpoint}
        self._stopping = threading.Event()
        self.thread = threading.Thread(target=self._serve, name="router-control", daemon=True)

    def _serve(self) -> None:
        while not self._stopping.is_set():
            try:
                data, peer = self.sock.recvfrom(MAX_DATAGRAM)
            except ConnectionResetError:
                continue
            except OSError:
                return
            if self._stopping.is_set():
                return
            reply = self.request(data)
            try:
                self.sock.sendto(json.dumps(reply).encode(), peer)
            except OSError:
                pass

    def request(self, data: bytes) -> dict:
        try:
            request = json.loads(data.decode())
            op, name = request["op"], request["client"]
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            return {"op": "error", "error": "bad request"}
        if not isinstance(request, dict) or not isinstance(op, str) or not isinstance(name, str):
            return {"op": "error", "error": "bad request"}
        if name not in self._clients:
            return {"op": "error", "error": f"unknown client {name!r}", "id": request.get("id")}
        handler = self._ops.get(op)
        if handler is None:
            return {"op": "error", "error": f"unknown op {op!r}", "id": request.get("id")}
        reply = handler(name)
        reply["id"] = request.get("id")
        return reply

    def wake(self) -> None:
        """Stop serving: the next recvfrom returns at once (call before joining the thread)."""
        self._stopping.set()
        try:
            self.sock.sendto(b"", self.sock.getsockname())
        except OSError:
            pass

    def close(self) -> None:
        self.sock.close()
