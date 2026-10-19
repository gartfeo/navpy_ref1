"""NAVLINK traced hop by hop through a fleet run's packet records (step 10f).

``lockstep_fleet.md``, "Evidence" (Correlation) and the 10f acceptance: a
NAVLINK packet NavPy i sends crosses five hops to NavPy j, each a packet
record (``coordinator_packets``):

1. NavPy i's emission on ``lockstep:serial0.<i>``, delivered to SITL i's
   serial0 (the reply that admits its last byte);
2. ArduPilot i's re-emission on its serial1 (``sitl<i>.serial1``);
3. the router's copy of it to ``sitl<j>.serial1``, delivered to SITL j;
4. ArduPilot j's re-emission on its serial0 (``lockstep:serial0.<j>``);
5. that packet's delivery to NavPy j (participant ordinal j) in a ``TICK``.

Router copies chain by ``parent``. An ArduPilot re-emission is a new SITL
emission, matched by the NavPy UID ``(sender, boot_id, msg_seq)`` and the
bytes (sha256): NavPy sends one message more than once under the same UID
(ack resends, task rebroadcasts), and each send has its own MAVLink
sequence, so the bytes tell the sends apart. ArduPilot forwards a packet
unchanged (measured in 10f); a forward that altered it stops the trace
short. A re-emission comes after the delivery it answers: a SITL reads
reply k before it steps, so its answer is in frame k+1 or later. Two sends
of one UID with the same bytes (an unchanged retry after the MAVLink
sequence wrapped) cannot be told apart downstream: their traces are marked
``ambiguous`` (a gate must not count them as delivered).

A trace records each hop's tick and its latency: ticks from NavPy i's
emission to NavPy j's delivery. SITL i is sysid i and NavPy ordinal i
(``FleetLayout``). A trace that stops short names the hop it stopped at. A
packet is traced to the peers it is for: every other NavPy when it has no
target (-1) or a broadcast one (0), else only its target (ArduPilot and the
router forward a targeted packet only toward its target; one for the sender
itself or a non-NavPy system has no trace).
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

from navpy.sim.lockstep.coordinator_mavlink import NAVLINK_IDS
from navpy.sim.lockstep.coordinator_types import serial0_device, serial1_device

Id = tuple[Any, ...]
#: The hops of a complete trace, in order.
HOPS = ("navpy_emitted", "sitl_serial0", "ap_serial1", "router_copy", "ap_serial0", "navpy_delivered")
#: Targets that address every peer: none (the router table's -1) or broadcast.
_EVERY_PEER = (-1, 0)


@dataclass(frozen=True)
class _Emission:
    tick: int
    sha256: str
    msgid: int
    uid: tuple | None
    target: int


@dataclass(frozen=True)
class NavlinkTrace:
    uid: tuple
    msgid: int
    sender: int
    receiver: int
    #: ``(hop, tick)`` in :data:`HOPS` order, as far as the trace got.
    hops: tuple[tuple[str, int], ...]
    #: The sender emitted this UID with these bytes more than once.
    ambiguous: bool = False
    #: The radio link model lost the router copy (``loss`` or ``outage``): an injected effect.
    lost: str | None = None

    @property
    def complete(self) -> bool:
        return len(self.hops) == len(HOPS)

    @property
    def latency_ticks(self) -> int | None:
        """Ticks from NavPy ``sender``'s emission to NavPy ``receiver``'s delivery."""
        return self.hops[-1][1] - self.hops[0][1] if self.complete else None


class _Records:
    def __init__(self, records: Iterable[Mapping[str, Any]]) -> None:
        self.emitted: dict[Id, _Emission] = {}
        self.copies: dict[Id, list[Id]] = {}
        self.delivered: dict[Id, tuple[int, Any]] = {}
        #: Router copies the radio link lost, and why.
        self.lost: dict[Id, str] = {}
        #: ``(origin, device, uid, sha256)`` -> the unclaimed ids emitted there, in order.
        self._unclaimed: dict[tuple[Any, str, tuple, str], list[Id]] = {}
        for record in records:
            if "packets" in record:
                self._read(record["k"], record["packets"])
            elif "parked" in record and "packets" in record["parked"]:
                self._read(record["parked"]["tick"], record["parked"]["packets"])

    def _read(self, k: int, events: list[list[Any]]) -> None:
        for event in events:
            kind, packet_id = event[0], tuple(event[1])
            if kind == "emitted":
                fields = event[7] if len(event) > 7 else {}
                uid = tuple(fields["uid"]) if "uid" in fields else None
                self.emitted[packet_id] = _Emission(k, event[3], event[4], uid, event[6][0])
                if uid is not None:
                    key = (packet_id[0], packet_id[2], uid, event[3])
                    self._unclaimed.setdefault(key, []).append(packet_id)
            elif kind == "copy":
                self.copies.setdefault(tuple(event[2]), []).append(packet_id)
            elif kind == "delivered":
                receiver = event[2]
                self.delivered[packet_id] = (k, tuple(receiver) if isinstance(receiver, list) else receiver)
            elif kind == "lost":
                self.lost[packet_id] = event[2]

    def sends(self, sender: int, emission: _Emission) -> int:
        """How often NavPy ``sender`` emitted ``emission``'s UID with its bytes."""
        return len(self._unclaimed.get((sender, serial0_device(sender), emission.uid, emission.sha256), ()))

    def claim(self, sitl: int, device: str, emission: _Emission, after: int) -> Id | None:
        """SITL ``sitl``'s earliest unclaimed re-emission of ``emission`` (its UID
        and bytes) on ``device`` after tick ``after``, now claimed."""
        found = self._unclaimed.get((f"sitl{sitl}", device, emission.uid, emission.sha256), [])
        for index, packet_id in enumerate(found):
            if self.emitted[packet_id].tick > after:
                return found.pop(index)
        return None


def trace_navlink(records: Iterable[Mapping[str, Any]], fleet: Iterable[int]) -> list[NavlinkTrace]:
    """Every NAVLINK packet a NavPy of ``fleet`` emitted, traced to each other
    NavPy of ``fleet`` it is for: one :class:`NavlinkTrace` per (packet,
    receiver), in emission order, then receiver order."""
    sys_ids = tuple(fleet)
    book = _Records(records)
    sent = [(packet_id, emission) for packet_id, emission in book.emitted.items()
            if packet_id[0] in sys_ids and packet_id[2] == serial0_device(packet_id[0])
            and emission.msgid in NAVLINK_IDS and emission.uid is not None]
    ambiguous = {packet_id for packet_id, emission in sent if book.sends(packet_id[0], emission) > 1}
    traces = []
    for packet_id, emission in sent:
        sender = packet_id[0]
        hops, up = _upstream(book, packet_id, emission, sender)
        for receiver in sys_ids:
            if receiver != sender and emission.target in (*_EVERY_PEER, receiver):
                traces.append(_trace(book, emission, sender, receiver, up, list(hops), packet_id in ambiguous))
    return traces


def _upstream(book: _Records, packet_id: Id, emission: _Emission,
              sender: int) -> tuple[list[tuple[str, int]], Id | None]:
    """The hops every receiver shares: the emission, its delivery to SITL
    ``sender`` and ArduPilot's re-emission on serial1 (claimed once)."""
    hops: list[tuple[str, int]] = [("navpy_emitted", emission.tick)]
    delivered = book.delivered.get(packet_id)
    if delivered is None or delivered[1] != serial0_device(sender):
        return hops, None
    hops.append(("sitl_serial0", delivered[0]))
    up = book.claim(sender, serial1_device(sender), emission, delivered[0])
    if up is not None:
        hops.append(("ap_serial1", book.emitted[up].tick))
    return hops, up


def _trace(book: _Records, emission: _Emission, sender: int, receiver: int, up: Id | None,
           hops: list[tuple[str, int]], ambiguous: bool) -> NavlinkTrace:
    def done(lost: str | None = None) -> NavlinkTrace:
        return NavlinkTrace(emission.uid, emission.msgid, sender, receiver, tuple(hops), ambiguous, lost)

    if up is None:
        return done()
    copies = [c for c in book.copies.get(up, ()) if c[2] == serial1_device(receiver)]
    if len(copies) == 1 and copies[0] in book.lost:
        return done(book.lost[copies[0]])
    if len(copies) != 1 or book.delivered.get(copies[0], (None, None))[1] != serial1_device(receiver):
        return done()
    arrived = book.delivered[copies[0]][0]
    hops.append(("router_copy", arrived))
    down = book.claim(receiver, serial0_device(receiver), emission, arrived)
    if down is None:
        return done()
    hops.append(("ap_serial0", book.emitted[down].tick))
    final = book.delivered.get(down)
    if final is None or not (isinstance(final[1], tuple) and final[1][0] == receiver):
        return done()
    hops.append(("navpy_delivered", final[0]))
    return done()


__all__ = ["HOPS", "NavlinkTrace", "trace_navlink"]
