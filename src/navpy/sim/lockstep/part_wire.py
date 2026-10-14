"""The participant wire (``NVPART01``): exact layout, strict decoding.

TCP loopback between the coordinator and each participant
(``lockstep_slice.md``, "Wire NVPART01"). Each message is length-prefixed
(``<I``, :mod:`navpy.sim.lockstep.part_socket`) and is::

    header  <8sHB   magic "NVPART01", version, type        = 11 bytes
    body            canonical JSON (part_wire_codec)

Types: ``REGISTER`` (1) and ``DONE`` (2), participant to coordinator;
``TICK`` (3), coordinator to participant. Every id list is in strict id
order; events and log lines are streams, in delivery order. A lifecycle
record's ``id`` is its commit id, drawn in commit order from the operator
side's sequence (origin 0, shared with the coordinator's own spawns and the
operator's requests), so ``lifecycle[]`` is in id order too; the request it
answers, if any, is its ``request``. A wrong magic,
version or type, a malformed or non-canonical body, or a missing or
duplicate tick is a protocol error.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

from navpy.sim.lockstep import part_wire_codec as c
from navpy.sim.lockstep.part_wire_codec import PartWireError, RequestId

MAGIC = b"NVPART01"
VERSION = 1
HEADER = struct.Struct("<8sHB")
REGISTER, DONE, TICK = 1, 2, 3
#: Lifecycle record kinds (``lifecycle[]``): a spawn's result, a stop for its
#: target, a stop's result for its requester, readiness, and retirement.
LIFECYCLE_KINDS = ("spawn", "stop", "stop_result", "ready", "exited")

Ports = tuple[tuple[str, bytes], ...]


@dataclass(frozen=True)
class Register:
    capability: str
    role: str
    ordinal: int
    spawn_id: RequestId
    generation: int


@dataclass(frozen=True)
class Request:
    id: RequestId
    payload: Any


@dataclass(frozen=True)
class Response:
    id: RequestId
    payload: Any


@dataclass(frozen=True)
class Event:
    kind: str
    payload: Any


@dataclass(frozen=True)
class SpawnRequest:
    id: RequestId
    role: str
    ordinal: int


@dataclass(frozen=True)
class StopRequest:
    id: RequestId
    ordinal: int
    generation: int


@dataclass(frozen=True)
class Lifecycle:
    kind: str
    #: The commit id (origin 0, commit order).
    id: RequestId
    #: The request this record answers or carries (spawn, stop), else the spawn id.
    request: RequestId | None
    ordinal: int
    generation: int
    result: str | None = None
    payload: Any = None


@dataclass(frozen=True)
class Done:
    k: int
    ports: Ports = ()
    responses: tuple[Response, ...] = ()
    events: tuple[Event, ...] = ()
    spawn_requests: tuple[SpawnRequest, ...] = ()
    stop_requests: tuple[StopRequest, ...] = ()
    log_lines: tuple[str, ...] = ()
    faults: tuple[str, ...] = ()
    #: ``None``, or the participant's end: ``flushed`` (its last message).
    ended: bool | None = None


@dataclass(frozen=True)
class Tick:
    k: int
    t_us: int
    ports: Ports = ()
    requests: tuple[Request, ...] = ()
    lifecycle: tuple[Lifecycle, ...] = ()
    cancels: tuple[RequestId, ...] = ()
    end: bool = False


def _id(value: RequestId | None) -> list[int] | None:
    return None if value is None else [value[0], value[1]]


def _ports(ports: Ports) -> list[list[str]]:
    return [[device, data.hex()] for device, data in ports]


def _body(message: Register | Done | Tick) -> tuple[int, dict[str, Any]]:
    if isinstance(message, Register):
        return REGISTER, {
            "capability": message.capability, "role": message.role, "ordinal": message.ordinal,
            "spawn_id": _id(message.spawn_id), "generation": message.generation,
        }
    if isinstance(message, Done):
        return DONE, {
            "k": message.k, "ports": _ports(message.ports),
            "responses": [{"id": _id(r.id), "payload": r.payload} for r in message.responses],
            "events": [{"kind": e.kind, "payload": e.payload} for e in message.events],
            "spawn_requests": [{"id": _id(s.id), "role": s.role, "ordinal": s.ordinal}
                               for s in message.spawn_requests],
            "stop_requests": [{"id": _id(s.id), "ordinal": s.ordinal, "generation": s.generation}
                              for s in message.stop_requests],
            "log_lines": list(message.log_lines), "faults": list(message.faults),
            "ended": None if message.ended is None else {"flushed": message.ended},
        }
    return TICK, {
        "k": message.k, "t_us": message.t_us, "ports": _ports(message.ports),
        "requests": [{"id": _id(r.id), "payload": r.payload} for r in message.requests],
        "lifecycle": [
            {"kind": e.kind, "id": _id(e.id), "request": _id(e.request), "ordinal": e.ordinal,
             "generation": e.generation, "result": e.result, "payload": e.payload}
            for e in message.lifecycle
        ],
        "cancels": [_id(i) for i in message.cancels], "end": message.end,
    }


def encode(message: Register | Done | Tick) -> bytes:
    kind, body = _body(message)
    decoded = decode(HEADER.pack(MAGIC, VERSION, kind) + c.dumps(body))
    if decoded != message:  # the encoder never emits what the decoder refuses
        raise PartWireError(f"{type(message).__name__} does not round-trip: {message}")
    return HEADER.pack(MAGIC, VERSION, kind) + c.dumps(body)


def decode(data: bytes) -> Register | Done | Tick:
    if len(data) < HEADER.size:
        raise PartWireError("truncated header")
    magic, version, kind = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise PartWireError(f"bad magic {magic!r}")
    if version != VERSION:
        raise PartWireError(f"version {version}, expected {VERSION}")
    decoder = _DECODERS.get(kind)
    if decoder is None:
        raise PartWireError(f"unknown message type {kind}")
    return decoder(c.loads(data[HEADER.size:]))


def _decode_register(body: object) -> Register:
    b = c.fields(body, ("capability", "role", "ordinal", "spawn_id", "generation"), "REGISTER")
    return Register(
        c.text(b["capability"], "capability"), c.text(b["role"], "role"),
        c.integer(b["ordinal"], "ordinal", minimum=1), c.request_id(b["spawn_id"], "spawn_id"),
        c.integer(b["generation"], "generation"),
    )


def _decode_ports(value: object) -> Ports:
    def port(item: object) -> tuple[str, bytes]:
        if not isinstance(item, list) or len(item) != 2:
            raise PartWireError("a port is [device, hex]")
        return c.text(item[0], "device"), c.hex_bytes(item[1], "port bytes")

    ports = c.items(value, "ports", port)
    devices = [device for device, _ in ports]
    if devices != sorted(set(devices)):
        raise PartWireError(f"ports not in strict device order: {devices}")
    return ports


def _ordered(entries: tuple[Any, ...], what: str) -> tuple[Any, ...]:
    c.in_id_order([entry.id for entry in entries], what)
    return entries


def _decode_done(body: object) -> Done:
    b = c.fields(body, ("k", "ports", "responses", "events", "spawn_requests", "stop_requests",
                        "log_lines", "faults", "ended"), "DONE")
    ended = b["ended"]
    if ended is not None:
        ended = c.flag(c.fields(ended, ("flushed",), "ended")["flushed"], "flushed")
    return Done(
        c.integer(b["k"], "k", minimum=1), _decode_ports(b["ports"]),
        _ordered(c.items(b["responses"], "responses", lambda v: Response(
            c.request_id(c.fields(v, ("id", "payload"), "response")["id"]), v["payload"])),
            "responses"),
        c.items(b["events"], "events", lambda v: Event(
            c.text(c.fields(v, ("kind", "payload"), "event")["kind"], "event kind"), v["payload"])),
        _ordered(c.items(b["spawn_requests"], "spawn_requests", _decode_spawn), "spawn_requests"),
        _ordered(c.items(b["stop_requests"], "stop_requests", _decode_stop), "stop_requests"),
        c.items(b["log_lines"], "log_lines", lambda v: c.text(v, "log line")),
        c.items(b["faults"], "faults", lambda v: c.text(v, "fault")),
        ended,
    )


def _decode_spawn(value: object) -> SpawnRequest:
    v = c.fields(value, ("id", "role", "ordinal"), "spawn request")
    return SpawnRequest(c.request_id(v["id"]), c.text(v["role"], "role"),
                        c.integer(v["ordinal"], "ordinal", minimum=1))


def _decode_stop(value: object) -> StopRequest:
    v = c.fields(value, ("id", "ordinal", "generation"), "stop request")
    return StopRequest(c.request_id(v["id"]), c.integer(v["ordinal"], "ordinal", minimum=1),
                       c.integer(v["generation"], "generation"))


def _decode_lifecycle(value: object) -> Lifecycle:
    v = c.fields(value, ("kind", "id", "request", "ordinal", "generation", "result", "payload"),
                 "lifecycle")
    kind = c.text(v["kind"], "lifecycle kind")
    if kind not in LIFECYCLE_KINDS:
        raise PartWireError(f"unknown lifecycle kind {kind!r}")
    result = None if v["result"] is None else c.text(v["result"], "result")
    return Lifecycle(kind, c.request_id(v["id"]), c.optional_id(v["request"], "request"),
                     c.integer(v["ordinal"], "ordinal"),
                     c.integer(v["generation"], "generation"), result, v["payload"])


def _decode_tick(body: object) -> Tick:
    b = c.fields(body, ("k", "t_us", "ports", "requests", "lifecycle", "cancels", "end"), "TICK")
    cancels = c.items(b["cancels"], "cancels", c.request_id)
    c.in_id_order(cancels, "cancels")
    return Tick(
        c.integer(b["k"], "k", minimum=1), c.integer(b["t_us"], "t_us"), _decode_ports(b["ports"]),
        _ordered(c.items(b["requests"], "requests", lambda v: Request(
            c.request_id(c.fields(v, ("id", "payload"), "request")["id"]), v["payload"])),
            "requests"),
        _ordered(c.items(b["lifecycle"], "lifecycle", _decode_lifecycle), "lifecycle"), cancels,
        c.flag(b["end"], "end"),
    )


_DECODERS = {REGISTER: _decode_register, DONE: _decode_done, TICK: _decode_tick}


def check_tick(k: int, expected: int) -> None:
    """Stream rule: a ``TICK`` or ``DONE`` carries exactly the next tick."""
    if k != expected:
        raise PartWireError(f"tick {k}, expected {expected} (missing or duplicate)")


__all__ = [
    "DONE", "Done", "Event", "HEADER", "LIFECYCLE_KINDS", "Lifecycle", "MAGIC", "PartWireError",
    "Ports", "REGISTER", "Register", "Request", "RequestId", "Response", "SpawnRequest",
    "StopRequest", "TICK", "Tick", "VERSION", "check_tick", "decode", "encode",
]
