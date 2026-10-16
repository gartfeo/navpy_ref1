"""Client of the board link router's endpoint barrier (HITL only; docs/design/hitl_final_approach.md section 2).

The router (tools/tuning/mav_router.py) owns the board's USB link and serves each client a loopback UDP endpoint.
Its control port answers a JSON request {op: "close", client, id} once it refuses the client's packets, has dropped
those still queued and no write of them is in progress; the reply repeats the id. navpy closes its own endpoint before
its terminal record, the runner closes navpy's before any RTL; closing a closed endpoint is answered as well.
"""

from __future__ import annotations

import json
import socket
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt

# The router answers after at most one USB write in progress (milliseconds). The bound is for a router that is
# stuck or gone: past it the barrier has failed and the run counts as failed.
BARRIER_TIMEOUT_S = 2.0
# Loopback UDP can still drop a datagram under load: the request is sent again this often until the bound.
BARRIER_RETRY_S = 0.25
MAX_REPLY = 65535


@dataclass(frozen=True)
class BarrierResult:
    ack: dict | None
    error: str | None = None

    @property
    def ok(self) -> bool:
        return self.ack is not None


def parse_address(text: str) -> tuple[str, int]:
    """HOST:PORT of the router's control port."""
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"router control address {text!r} is not HOST:PORT")
    return host, int(port)


class RouterBarrier:
    """Close one client's router endpoint and wait, bounded, for the router's ack."""

    def __init__(self, address: tuple[str, int], client: str, timeout_s: float = BARRIER_TIMEOUT_S) -> None:
        self.address = address
        self.client = client
        self.timeout_s = timeout_s

    def close_endpoint(self) -> BarrierResult:
        request_id = rt.uuid4().hex
        request = json.dumps({"op": "close", "client": self.client, "id": request_id}).encode()
        deadline = rt.monotonic() + self.timeout_s
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            while (left := deadline - rt.monotonic()) > 0:
                try:
                    sock.sendto(request, self.address)
                except OSError as exc:
                    return BarrierResult(None, f"request not sent: {exc}")
                reply = self._reply(sock, request_id, rt.monotonic() + min(BARRIER_RETRY_S, left))
                if reply is not None:
                    if reply.get("op") == "closed" and reply.get("client") == self.client:
                        return BarrierResult(reply)
                    return BarrierResult(None, f"router refused: {reply.get('error', reply)}")
        return BarrierResult(None, f"no ack within {self.timeout_s:g} s")

    @staticmethod
    def _reply(sock: socket.socket, request_id: str, until: float) -> dict | None:
        """The reply to this request, or None at `until` (other datagrams and port-unreachable resets are skipped)."""
        while (left := until - rt.monotonic()) > 0:
            sock.settimeout(left)
            try:
                data, _ = sock.recvfrom(MAX_REPLY)
            except (socket.timeout, ConnectionResetError):  # Windows: no router listening yet
                continue
            try:
                reply = json.loads(data.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(reply, dict) and reply.get("id") == request_id:
                return reply
        return None


__all__ = ["BARRIER_RETRY_S", "BARRIER_TIMEOUT_S", "BarrierResult", "RouterBarrier", "parse_address"]
