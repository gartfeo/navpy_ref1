"""Reads a run journal's packet records back: dispositions, receivers, latency.

The checks the run gate needs from ``coordinator_packets``'s records
(``lockstep_fleet.md``, Evidence; the gates themselves are step 10g):

- every packet id is defined once (one ``emitted`` or ``copy``), has
  exactly one terminal disposition, and only a defined id has any;
- a delivery to a participant names a member ``[ordinal, generation]``
  that ``TICK k`` went to (the tick record's ``members``, which on a routed
  fleet carry the generation): bytes for an unadmitted or retired receiver,
  of any generation, can only be ``discarded``;
- a delivery to a SITL port follows admissions that add up to the packet's
  length.

:meth:`PacketJournal.latency` counts a packet's ticks from its root
emission (following router copies to their source, so source-side
carry-over counts) to its delivery.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

from navpy.sim.lockstep.coordinator_packets import TERMINALS

Id = tuple[Any, ...]


@dataclass
class _Packet:
    tick: int
    length: int
    parent: Id | None = None
    admitted: int = 0
    ends: list[tuple[int, list[Any]]] = field(default_factory=list)


class PacketJournal:
    def __init__(self, records: Iterable[Mapping[str, Any]]) -> None:
        self._packets: dict[Id, _Packet] = {}
        #: Per tick, the ``(ordinal, generation)`` of each member ``TICK k`` went to.
        self._members: dict[int, set[tuple[int, Any]]] = {}
        self._problems: list[str] = []
        for record in records:
            if "packets" in record:
                self._members[record["k"]] = {(m["ordinal"], m.get("generation"))
                                              for m in record.get("members", ())}
                self._read(record["k"], record["packets"])
            elif "parked" in record and "packets" in record["parked"]:
                self._read(record["parked"]["tick"], record["parked"]["packets"])

    def problems(self) -> list[str]:
        problems = list(self._problems)
        for packet_id, packet in self._packets.items():
            if len(packet.ends) != 1:
                problems.append(f"{list(packet_id)} has {len(packet.ends)} terminal dispositions")
        return problems

    def disposition(self, packet_id: Id) -> list[Any]:
        (_tick, event), = self._packets[tuple(packet_id)].ends
        return event

    def latency(self, packet_id: Id) -> int:
        """Ticks from the root emission to the delivery."""
        packet = self._packets[tuple(packet_id)]
        (tick, event), = packet.ends
        if event[0] != "delivered":
            raise ValueError(f"{list(packet_id)} was not delivered: {event}")
        root = packet
        while root.parent is not None:
            root = self._packets[root.parent]
        return tick - root.tick

    def _read(self, k: int, events: list[list[Any]]) -> None:
        for event in events:
            kind, packet_id = event[0], tuple(event[1])
            if kind in ("emitted", "copy") and packet_id in self._packets:
                self._problems.append(f"tick {k}: {kind} {list(packet_id)} is already defined")
                continue
            if kind == "emitted":
                start, end = event[2]
                self._packets[packet_id] = _Packet(k, end - start)
            elif kind == "copy":
                parent = self._packets.get(tuple(event[2]))
                if parent is None:
                    self._problems.append(f"copy {list(packet_id)} of an unknown packet")
                    continue
                self._packets[packet_id] = _Packet(k, parent.length, tuple(event[2]))
            elif kind == "admitted" and packet_id in self._packets:
                self._packets[packet_id].admitted += event[2]
            elif kind == "arrived" and packet_id in self._packets:
                if self._packets[packet_id].ends:
                    self._problems.append(f"tick {k}: {list(packet_id)} arrived after it ended")
            elif kind in TERMINALS:
                self._end(k, packet_id, event)
            else:
                self._problems.append(f"tick {k}: {kind} for an unknown packet {list(packet_id)}")

    def _end(self, k: int, packet_id: Id, event: list[Any]) -> None:
        packet = self._packets.get(packet_id)
        if packet is None:
            self._problems.append(f"tick {k}: {event[0]} for an unknown packet {list(packet_id)}")
            return
        packet.ends.append((k, event))
        if event[0] != "delivered":
            return
        receiver = event[2]
        if isinstance(receiver, list):  # a participant [ordinal, generation]
            if tuple(receiver) not in self._members.get(k, ()):
                self._problems.append(f"tick {k}: {list(packet_id)} delivered to {receiver}, "
                                      f"a member TICK {k} did not go to")
        elif packet.admitted != packet.length:
            self._problems.append(f"tick {k}: {list(packet_id)} delivered with {packet.admitted} "
                                  f"of {packet.length} bytes admitted")


__all__ = ["PacketJournal"]
