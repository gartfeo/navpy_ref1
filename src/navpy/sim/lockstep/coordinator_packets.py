"""Packet records: every MAVLink packet that crosses the coordinator, from emission to end.

``lockstep_fleet.md``, Evidence the gates need (packet records). Each
complete packet gets an id ``(origin, generation, device, emission seq)``
when it is emitted, before any queue: a SITL's (``sitl<i>``, generation 1)
when frame k is read, a participant's (its ordinal and generation) when its
``DONE k`` is applied, a router copy's (``router``, 1, the destination
device) when it is routed. A tick record's ``packets`` list holds that
tick's events, in order (ids are lists):

- ``["emitted", id, [start, end], sha256, msgid, [src sys, comp], [target sys, comp], fields?]``:
  the byte range in the origin stream; the target from the router entry
  table (-1: none); ``fields`` for a NAVLINK packet: its NavPy UID
  ``uid = [sender, boot_id, msg_seq]``, ``task_id`` and a ``SWARM_ACK``'s
  ``ack = {ref: [boot_id, msg_seq, msg type], status}``, where present.
- ``["copy", id, parent]``: a router copy; it keeps the parent's bytes.
- ``["admitted", id, bytes]``: bytes of the packet reply k carried;
  ``["admitted", id, bytes, j]``: bytes of its prefix that reply j < k
  carried before the packet was complete (point-to-point bytes move unframed).

and the terminal dispositions, exactly one per id:

- ``["delivered", id, receiver]``: to a SITL port (its device) at the reply
  that admitted the last byte, or to a participant (``[ordinal, generation]``)
  in ``TICK k``, written only when that member is admitted and live and
  every byte of the packet went to it;
- ``["discarded", id, reason]``: ``unadmitted``, ``retired`` or ``drain``;
- ``["router_dropped", id]``: no endpoint accepted it;
- ``["routed", id, copies]``: a source the router copied;
- ``["undelivered", id, bytes admitted, bytes pending]``: queued at parking.

A second disposition, or an id that never ends, is a coordinator fault.
"""

from __future__ import annotations

from collections.abc import Hashable

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.coordinator_journal import sha256
from navpy.sim.lockstep.coordinator_mavlink import NAVLINK_IDS, NO_TARGET
from navpy.sim.lockstep.coordinator_types import CoordinatorFault
from navpy.sim.lockstep.router_framer import NO_TARGET as NO_FIELD
from navpy.sim.lockstep.router_framer import RouterPacket, Segment

PacketId = tuple[Hashable, int, str, int]
ROUTER_ORIGIN = "router"
DISCARD_REASONS = ("unadmitted", "retired", "drain")
TERMINALS = ("delivered", "discarded", "router_dropped", "routed", "undelivered")
_DECODER = mavlink.MAVLink(None)


def sitl_origin(index: int) -> str:
    """SITL ``index`` (1-based) as a packet origin."""
    return f"sitl{index}"


def packet_key(packet: RouterPacket) -> str:
    """The tally key ``<id>:<src_sys>.<src_comp>><target>`` (``coordinator_mavlink``),
    target from the entry table."""
    system, component = packet.source
    target = packet.target[0]
    return f"{packet.msgid}:{system}.{component}>{NO_TARGET if target == NO_FIELD else target}"


def navlink_fields(packet: RouterPacket) -> dict[str, object]:
    """The NavPy UID, task id and ACK reference of a NAVLINK packet, where present."""
    if packet.msgid not in NAVLINK_IDS:
        return {}
    try:
        message = _DECODER.decode(bytearray(packet.raw))
    except (mavlink.MAVError, KeyError, ValueError, IndexError, TypeError):
        return {}
    values = message.to_dict()
    fields: dict[str, object] = {}
    if values.get("boot_id") or values.get("msg_seq"):
        fields["uid"] = [packet.source[0], values["boot_id"], values["msg_seq"]]
    if "task_id" in values:
        fields["task_id"] = values["task_id"]
    if "ref_msg_seq" in values:
        fields["ack"] = {"ref": [values["ref_boot_id"], values["ref_msg_seq"], values["ref_msg_type"]],
                         "status": values["status"]}
    return fields


def _wire(packet_id: PacketId) -> list[object]:
    return list(packet_id)


class PacketBook:
    def __init__(self) -> None:
        self._seqs: dict[tuple[Hashable, int, str], int] = {}
        #: Open ids (emitted, no terminal disposition yet) and their tally keys.
        self._open: dict[PacketId, str] = {}
        self._events: list[list[object]] = []

    def emit(self, origin: Hashable, generation: int, device: str, segment: Segment) -> PacketId:
        packet = segment.packet
        if packet is None:
            raise ValueError("only a packet is emitted; skipped bytes have no id")
        packet_id = self._next(origin, generation, device)
        event = ["emitted", _wire(packet_id), [segment.start, segment.start + len(segment.data)],
                 sha256(segment.data), packet.msgid, list(packet.source), list(packet.target)]
        fields = navlink_fields(packet)
        if fields:
            event.append(fields)
        self._events.append(event)
        self._open[packet_id] = packet_key(packet)
        return packet_id

    def copy(self, parent: PacketId, device: str) -> PacketId:
        packet_id = self._next(ROUTER_ORIGIN, 1, device)
        self._events.append(["copy", _wire(packet_id), _wire(parent)])
        self._open[packet_id] = self._open[parent]
        return packet_id

    def admitted(self, packet_id: PacketId, count: int, tick: int | None = None) -> None:
        """``count`` bytes admitted by this tick's reply, or by reply ``tick`` (bytes
        of a packet's prefix an earlier reply took before it was complete)."""
        event: list[object] = ["admitted", _wire(packet_id), count]
        if tick is not None:
            event.append(tick)
        self._events.append(event)

    def end(self, packet_id: PacketId, disposition: str, *detail: object) -> str:
        """Record ``packet_id``'s terminal disposition; returns its tally key."""
        key = self._open.pop(packet_id, None)
        if key is None:
            raise CoordinatorFault("packet_disposition",
                                   f"packet {packet_id} {disposition}: it has no open emission")
        self._events.append([disposition, _wire(packet_id), *detail])
        return key

    def take(self) -> list[list[object]]:
        """The events since the last take, for the tick (or parked) record."""
        events, self._events = self._events, []
        return events

    def open_ids(self) -> list[PacketId]:
        return list(self._open)

    def _next(self, origin: Hashable, generation: int, device: str) -> PacketId:
        key = (origin, generation, device)
        self._seqs[key] = self._seqs.get(key, 0) + 1
        return origin, generation, device, self._seqs[key]


__all__ = [
    "DISCARD_REASONS", "PacketBook", "PacketId", "ROUTER_ORIGIN", "TERMINALS", "navlink_fields",
    "packet_key", "sitl_origin",
]
