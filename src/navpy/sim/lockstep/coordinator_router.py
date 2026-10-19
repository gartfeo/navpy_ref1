"""The coordinator's ports: the router in the tick path, the queues, the packet records.

``lockstep_fleet.md``, Router and delivery; every run takes this path, the
one-SITL slice included (step 10c). The router's endpoints are
``sitl1 .. sitlN`` (SITL i's serial1, owned by the coordinator) and
``backend`` (the backend's ``Manifest.router`` device), in that order.

Bytes and records are kept apart. Point-to-point bytes (SITL i's serial0
to NavPy i, NavPy i's egress to SITL i's serial0) move as they come, by the
byte rules alone (``coordinator_inbound``, ``coordinator_queues``; each
SITL-bound byte counted once under its phase, ``coordinator_streams``); a
persistent :class:`~navpy.sim.lockstep.router_framer.RouterFramer` per
stream only observes them, so a packet split across ticks is emitted once
complete and its records follow its bytes. The router, like mavlink-router,
forwards only whole packets: on serial1 and the backend's endpoint the
framer is the router's own decoder. Within tick k:

1. :meth:`RoutedPorts.frames`: per SITL in order, serial0 egress is
   emitted for NavPy i's ``TICK k``; serial1 egress is emitted and routed,
   in endpoint order, then byte order. A copy for ``sitl<j>`` joins SITL j's
   serial1 queue; one for ``backend`` the backend's ``TICK k``.
2. :meth:`RoutedPorts.inbound`: what an admitted, live member owns is
   delivered in its ``TICK k``; the rest is discarded (``unadmitted`` or
   ``retired``) and counted, by the slice's Joining rule.
3. :meth:`RoutedPorts.outbound`, per ``DONE`` in ordinal order: NavPy i's
   egress joins SITL i's serial0 queue; the backend's is routed as source
   ``backend`` and joins the serial1 queues behind step 1's bytes. Drain
   egress is never delivered (``discarded``, ``drain``).
4. :meth:`RoutedPorts.reply`: reply k admits from each queue; each tick
   record carries its packet events and the bytes still pending per port.

Parking (:meth:`RoutedPorts.park`) journals the parked frame's packets
(never routed: ``discarded``, ``drain``, and their skipped bytes count
under ``from_sitl``) and every queued byte as ``undelivered``, then drops
the queues. :meth:`RoutedPorts.closing` requires every packet to have
ended and journals an incomplete frame a stream still holds.

The link models (the coordinator's ``links``, ``link_model``) plug in at these seams:
SITL i's serial0 egress passes the serial link before step 1's framing;
NavPy i's egress joins its queue with a ready tick and the port's line rate
(``coordinator_queues``); every router copy goes through its sender's radio,
which steps at the start of step 1 and after each routing. Each tick record
then notes ``link_held``; parking ends what the links still hold
(``discarded``, ``link``; ``in_flight``).
"""

from __future__ import annotations

from collections.abc import Hashable, Mapping, Sequence
from typing import Optional

from navpy.sim.lockstep.coordinator_inbound import InboundLedger
from navpy.sim.lockstep.coordinator_journal import TickRecord, sha256
from navpy.sim.lockstep.coordinator_links import Delivery, PortLinks
from navpy.sim.lockstep.coordinator_mavlink import (
    DRAIN_TO_SITL,
    FROM_SITL,
    ROUTED_OUT,
    ROUTER_DROPPED,
    ROUTER_LINK,
    ROUTER_SKIPPED_BYTES,
    MavlinkTally,
)
from navpy.sim.lockstep.coordinator_members import Member, Roster
from navpy.sim.lockstep.coordinator_packets import PacketBook, PacketId, packet_key, sitl_origin
from navpy.sim.lockstep.coordinator_queues import ReplyQueues
from navpy.sim.lockstep.coordinator_streams import StreamAccount
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, Manifest
from navpy.sim.lockstep.link_model import LinkModels
from navpy.sim.lockstep.router_entries import EntryTable, load_entry_table
from navpy.sim.lockstep.router_framer import RouterFramer, RouterPacket
from navpy.sim.lockstep.router_model import ReferenceRouter
from navpy.sim.lockstep.step_wire import Frame

BACKEND_ENDPOINT = "backend"
#: A SITL's generation in its packet ids: one boot per run.
SITL_GENERATION = 1

#: One framed piece of a stream: its packet id (None: skipped bytes), bytes,
#: packet, and its offset in the stream.
Piece = tuple[Optional[PacketId], bytes, Optional[RouterPacket], int]


def router_endpoints(fleet_size: int) -> list[str]:
    return [sitl_origin(index) for index in range(1, fleet_size + 1)] + [BACKEND_ENDPOINT]


class RoutedPorts:
    def __init__(self, manifest: Manifest, roster: Roster, table: EntryTable | None = None,
                 links: LinkModels | None = None) -> None:
        self._sitls = manifest.sitl_ports
        self._router_device = manifest.router
        endpoints = router_endpoints(len(self._sitls))
        self._router = ReferenceRouter(endpoints)
        self._table = table or load_entry_table()
        self._roster = roster
        self._framers: dict[tuple[Hashable, int, str], RouterFramer] = {}
        self._book = PacketBook()
        #: The link models' seams; each router endpoint's device, in endpoint order.
        self._links = PortLinks(links, self._sitls, dict(zip(
            endpoints, [ports[1] for ports in self._sitls] + [manifest.router])), self._book)
        self._queues = ReplyQueues(manifest.sitl_devices, self._links.rates)
        #: This tick's participant-bound bytes and their fates (step 1 to step 2).
        self._ledger = InboundLedger()
        self.tally = MavlinkTally((*manifest.sitl_devices, ROUTER_LINK))
        #: SITL-bound bytes, each counted once under the phase it was written in.
        self._account = StreamAccount(self._queues, self._book, self.tally)

    def frames(self, k: int, record: TickRecord, egress: Mapping[str, bytes]) -> None:
        self._deliver(self._links.step(k, record.data["t_us"]))  # before this frame's copies
        for index, (serial0, serial1) in enumerate(self._sitls, start=1):
            origin = sitl_origin(index)
            data = self._links.to_navpy(k, serial0, egress.get(serial0, b""))
            framer = self._framer(origin, SITL_GENERATION, serial0)
            start = framer.fed
            pieces = self._emit(origin, SITL_GENERATION, serial0, data)
            self._count(record.data, serial0, FROM_SITL, pieces)
            packets = [(packet_id, at, at + len(piece))
                       for packet_id, piece, _packet, at in pieces if packet_id is not None]
            self._ledger.add(serial0, start, data, packets, framer.held_start)
            pieces = self._emit(origin, SITL_GENERATION, serial1, egress.get(serial1, b""))
            self._count(record.data, serial1, FROM_SITL, [p for p in pieces if p[0] is not None])
            self._route(k, record, origin, serial1, pieces)

    def inbound(self, k: int, record: TickRecord, owners: Mapping[str, Member]) -> dict[str, bytes]:
        return self._ledger.settle(record, owners,
                                   lambda device: no_receiver_reason(self._roster.members, device),
                                   self._book)

    def outbound(self, k: int, member: Member, device: str, data: bytes, record: TickRecord, *,
                 draining: bool) -> None:
        stream = (member.ordinal, member.spawn.generation)
        if draining:
            record.drain_egress(member.ordinal, device, data)
        if device != self._router_device:
            ready = None if draining else self._links.ready(k, device, data)
            self._queue_stream(record, stream, device, data, k if draining else None, ready)
        elif draining:
            pieces = self._emit(*stream, device, data)
            self._end_all(record.data, ROUTER_LINK, DRAIN_TO_SITL, pieces, "drain")
        else:
            self._route(k, record, BACKEND_ENDPOINT, ROUTER_LINK, self._emit(*stream, device, data))

    def reply(self, k: int, record: TickRecord, frames: Sequence[Frame]) -> dict[str, bytes]:
        ingress: dict[str, bytes] = {}
        for devices, frame in zip(self._sitls, frames):
            for device, port in zip(devices, frame.ports):
                ingress[device], progress = self._queues.admit(device, port.fifo_bytes, k,
                                                               frame.source_us)
                # Unclassified bytes (a frame not yet complete) count once framing resolves them.
                self._account.admitted(record.data, device, progress)
        record.note("pending", self._queues.pending())
        if self._links.on:
            record.note("link_held", self._links.held())
        record.note("packets", self._book.take())
        return ingress

    def drained(self, record: TickRecord) -> None:
        record.note("packets", self._book.take())

    def park(self, parked: dict, egress: Mapping[str, bytes]) -> None:
        for index, devices in enumerate(self._sitls, start=1):
            for device in devices:
                held = self._links.park_serial(device)  # still in the link: before the parked frame
                if held:
                    parked.setdefault("link_held", {})[device] = {"bytes": len(held), "sha256": sha256(held)}
                    pieces = self._emit(sitl_origin(index), SITL_GENERATION, device, held)
                    self._end_all(parked, device, FROM_SITL, pieces, "link")
                pieces = self._emit(sitl_origin(index), SITL_GENERATION, device, egress.get(device, b""))
                self._end_all(parked, device, FROM_SITL, pieces, "drain")
        self._links.park_radio()
        undelivered, leftovers = self._queues.park()
        self._account.park(leftovers)
        parked["undelivered"] = undelivered
        parked["packets"] = self._book.take()

    def closing(self) -> dict[str, object]:
        unended = self._book.open_ids()
        if unended:
            raise CoordinatorFault("packet_disposition",
                                   f"{len(unended)} packets never ended, first {unended[0]}")
        self._account.held({(device, (origin, generation)): framer.fed
                            for (origin, generation, device), framer in self._framers.items()
                            if not isinstance(origin, str)})
        counts: dict[str, object] = {"mavlink_counts": self.tally.totals(),
                                     "stream_bytes": self._account.summary()}
        held = {f"{origin}/{generation}/{device}": {"bytes": len(framer.held),
                                                     "sha256": sha256(framer.held)}
                for (origin, generation, device), framer in self._framers.items() if framer.held}
        if held:  # an incomplete frame at the end: journaled, never dropped silently
            counts["held"] = held
        return counts

    def _framer(self, origin: Hashable, generation: int, device: str) -> RouterFramer:
        framer = self._framers.get((origin, generation, device))
        if framer is None:
            framer = self._framers[(origin, generation, device)] = RouterFramer(self._table)
        return framer

    def _emit(self, origin: Hashable, generation: int, device: str, data: bytes) -> list[Piece]:
        return [(None if segment.packet is None
                 else self._book.emit(origin, generation, device, segment),
                 segment.data, segment.packet, segment.start)
                for segment in self._framer(origin, generation, device).feed(data)]

    def _queue_stream(self, record: TickRecord, stream: tuple[int, int], device: str,
                      data: bytes, drain_tick: int | None, ready: int | None) -> None:
        """A participant's bytes for its SITL serial0: queued as they come (in drain
        only recorded), admissible from reply ``ready`` (the serial link; None: now);
        framing classifies each range it resolves (``coordinator_streams``)."""
        framer = self._framer(*stream, device)
        base = self._account.write(device, stream, framer.fed, data, drain_tick, ready)
        for packet_id, piece, _packet, start in self._emit(*stream, device, data):
            self._account.resolve(record.data, device, packet_id, base + start, base + start + len(piece))
        self._account.settle(device, stream, base, base + framer.held_start)

    def _route(self, k: int, record: TickRecord, endpoint: str, link: str, pieces: list[Piece]) -> None:
        """Route ``endpoint``'s packets in byte order; ``link`` is its tally link. With the
        radio link model each copy goes through ``endpoint``'s radio first."""
        skipped = 0
        for packet_id, piece, packet, _start in pieces:
            if packet_id is None or packet is None:
                skipped += len(piece)
                continue
            destinations = self._router.route(endpoint, packet)
            if not destinations:
                key = self._book.end(packet_id, "router_dropped")
                self.tally.count(record.data, link, ROUTER_DROPPED, [key])
                continue
            copies = []
            for destination in destinations:
                device = self._links.endpoint_device[destination]
                copy = self._book.copy(packet_id, device)
                tally_link = ROUTER_LINK if device == self._router_device else device
                self.tally.count(record.data, tally_link, ROUTED_OUT, [packet_key(packet)])
                copies.append((destination, copy))
            self._deliver(self._links.send(k, record.data["t_us"], endpoint, piece, copies))
            self._book.end(packet_id, "routed", len(destinations))
        self.tally.count(record.data, link, ROUTER_SKIPPED_BYTES, skipped_bytes=skipped)

    def _deliver(self, deliveries: list[Delivery]) -> None:
        """Router copies reach their devices: the backend's ``TICK`` or a serial1 queue."""
        for device, copy, piece in deliveries:
            if device == self._router_device:
                self._ledger.add_whole(device, copy, piece)
            else:
                queue = self._queues[device]
                queue.append(piece, copy)
                queue.prune(queue.tail)  # whole packets: nothing registers later

    def _end_all(self, record: dict, link: str, direction: str, pieces: list[Piece], reason: str) -> None:
        keys = [self._book.end(packet_id, "discarded", reason)
                for packet_id, *_rest in pieces if packet_id is not None]
        self._count(record, link, direction, pieces, keys)

    def _count(self, record: dict, link: str, direction: str, pieces: list[Piece],
               keys: list[str] | None = None) -> None:
        if keys is None:
            keys = [packet_key(packet) for _id, _piece, packet, _at in pieces if packet is not None]
        skipped = sum(len(piece) for packet_id, piece, *_rest in pieces if packet_id is None)
        self.tally.count(record, link, direction, keys, skipped)


def no_receiver_reason(members: Sequence[Member], device: str) -> str:
    """Why a device's bytes have no receiver: its latest member retired, or none is admitted yet."""
    owners = [member for member in members if device in member.devices]
    return "retired" if owners and not owners[-1].live else "unadmitted"


__all__ = ["BACKEND_ENDPOINT", "RoutedPorts", "SITL_GENERATION", "no_receiver_reason",
           "router_endpoints"]
