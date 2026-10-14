"""Participant-bound bytes of the router path: bytes move by the byte rules, records follow.

``lockstep_fleet.md``, Router and delivery and Evidence. A tick's bytes for
a device (SITL i's serial0 egress for NavPy i, the router's copies for the
backend) go in that device's ``TICK k`` when an admitted, live member owns
it, else they are discarded and counted (``unadmitted`` or ``retired``, the
slice's Joining rule) - whatever their framing: a packet's prefix is
delivered or discarded in the tick it arrives, never held back.

The journal decoding only observes the stream. Each tick's byte range gets
its fate; a packet, emitted once complete, ends by the fates of all its
bytes: ``delivered`` to a receiver ``[ordinal, generation]`` only if every
byte went to that same member, else ``discarded`` with the reason of its
first byte that did not (``retired`` if its bytes went to two generations).
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_members import Member
from navpy.sim.lockstep.coordinator_packets import PacketBook, PacketId

#: ``("delivered", [ordinal, generation])`` or ``("discarded", reason)``.
Fate = tuple[str, object]


@dataclass
class _Block:
    #: The stream offset of ``data[0]``.
    start: int
    data: bytearray = field(default_factory=bytearray)
    #: Packets completed this tick: ``(id, start, end)`` stream offsets.
    packets: list[tuple[PacketId, int, int]] = field(default_factory=list)
    #: No later packet starts before this offset: older fates may go.
    floor: int = 0


class InboundLedger:
    def __init__(self) -> None:
        self._blocks: dict[str, _Block] = {}
        self._fates: dict[str, list[tuple[int, int, Fate]]] = {}
        #: The next stream offset of each device fed whole packets (the router's copies).
        self._whole: dict[str, int] = {}

    def add(self, device: str, start: int, data: bytes, packets: list[tuple[PacketId, int, int]],
            floor: int) -> None:
        """A stream's bytes this tick (from ``start``) and the packets they completed."""
        if data or packets:
            self._blocks[device] = _Block(start, bytearray(data), list(packets), floor)

    def add_whole(self, device: str, packet_id: PacketId, data: bytes) -> None:
        """One whole packet (a router copy), behind what the device got this tick."""
        start = self._whole.get(device, 0)
        self._whole[device] = start + len(data)
        block = self._blocks.setdefault(device, _Block(start))
        block.data += data
        block.packets.append((packet_id, start, start + len(data)))
        block.floor = start + len(data)

    def settle(self, record: TickRecord, owners: Mapping[str, Member],
               reason: Callable[[str], str], book: PacketBook) -> dict[str, bytes]:
        """Deliver or discard this tick's bytes; end the packets they completed."""
        delivered: dict[str, bytes] = {}
        for device, block in self._blocks.items():
            member = owners.get(device)
            if member is None:
                record.discard(device, len(block.data))
                fate: Fate = ("discarded", reason(device))
            else:
                delivered[device] = bytes(block.data)
                fate = ("delivered", [member.ordinal, member.spawn.generation])
            fates = self._fates.setdefault(device, [])
            fates.append((block.start, block.start + len(block.data), fate))
            for packet_id, start, end in block.packets:
                book.end(packet_id, *_disposition(fates, start, end))
            self._fates[device] = [entry for entry in fates if entry[1] > block.floor]
        self._blocks = {}
        return delivered


def _disposition(fates: list[tuple[int, int, Fate]], start: int, end: int) -> Fate:
    covering = [fate for a, b, fate in fates if a < end and b > start]
    refused = [fate for fate in covering if fate[0] != "delivered"]
    if refused:
        return refused[0]
    receivers = {tuple(fate[1]) for fate in covering}
    return covering[0] if len(receivers) == 1 else ("discarded", "retired")


__all__ = ["InboundLedger"]
