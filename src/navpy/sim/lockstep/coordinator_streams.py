"""SITL-bound bytes, each counted once under the phase it was written in.

``lockstep_fleet.md``, Queues and Evidence. Bytes toward a SITL port move by
the byte rules alone (``coordinator_queues``): queued, admitted by a reply,
undelivered at parking, or written in drain. A participant's serial0 stream
is framed only to classify them, and framing may resolve a range ticks
later, even after parking. :meth:`StreamAccount.resolve` is the one place
a resolved range is classified, against the phases its bytes have had
since they were written:

- a packet gets an ``admitted`` record (with its reply's tick) per
  admitted range; a range still queued is tagged, so later replies report
  their bytes of it and the one taking its last byte delivers it; a packet
  that can no longer be delivered (it resolved after parking) is
  ``discarded{drain}``;
- skipped bytes count under their own phase: ``to_sitl`` if admitted,
  ``drain_to_sitl`` if written in drain, undelivered if parked; a range
  still queued is tagged :data:`~navpy.sim.lockstep.coordinator_queues.SKIPPED`
  and counted when a reply takes it or parking finds it;
- a remainder framing never resolves is ``held`` at closing, per phase.

:meth:`StreamAccount.summary` journals the bytes per port, phase
(``admitted``, ``undelivered``, ``drain``) and class (``packet``,
``skipped``, ``held``): they add up to the bytes written in each phase.
"""

from __future__ import annotations

from collections.abc import Hashable, Mapping

from navpy.sim.lockstep.coordinator_mavlink import DRAIN_TO_SITL, TO_SITL, MavlinkTally
from navpy.sim.lockstep.coordinator_packets import PacketBook, PacketId
from navpy.sim.lockstep.coordinator_queues import SKIPPED, Leftover, Progress, ReplyQueues


class StreamAccount:
    def __init__(self, queues: ReplyQueues, book: PacketBook, tally: MavlinkTally) -> None:
        self._queues = queues
        self._book = book
        self._tally = tally
        #: ``device -> phase -> class -> bytes``.
        self._bytes: dict[str, dict[str, dict[str, int]]] = {}
        #: Each participant stream's ``(device, stream) -> (queue base, unresolved floor)``.
        self._streams: dict[tuple[str, Hashable], tuple[int, int]] = {}

    def write(self, device: str, stream: Hashable, fed: int, data: bytes,
              drain_tick: int | None, ready: int | None = None) -> int:
        """A stream's bytes, queued (or, in drain, only recorded) and admissible from reply
        ``ready`` (None: now); returns its queue base."""
        queue = self._queues[device]
        base = queue.stream_base(stream, fed)
        if drain_tick is None:
            queue.append(data, ready=ready)
        else:
            queue.write_drain(len(data), drain_tick)
        return base

    def resolve(self, record: dict, device: str, packet_id: PacketId | None, start: int,
                end: int) -> None:
        """Framing resolved ``[start, end)`` as ``packet_id`` (None: skipped bytes)."""
        queue = self._queues[device]
        kind = "skipped" if packet_id is None else "packet"
        queued = False
        for phase, count in queue.resolve(start, end):
            if phase[0] == "queued":
                queued = True
                continue
            self._add(device, phase[0], kind, count)
            if phase[0] == "admitted" and packet_id is not None:
                self._book.admitted(packet_id, count, phase[1])
            elif phase[0] in ("admitted", "drain") and packet_id is None:
                direction = TO_SITL if phase[0] == "admitted" else DRAIN_TO_SITL
                self._tally.count(record, device, direction, skipped_bytes=count)
        if queued:
            queue.register(SKIPPED if packet_id is None else packet_id, start, end)
        elif packet_id is not None:  # resolved after parking: none of it can still go
            key = self._book.end(packet_id, "discarded", "drain")
            self._tally.count(record, device, DRAIN_TO_SITL, [key])

    def settle(self, device: str, stream: Hashable, base: int, floor: int) -> None:
        """``stream`` has no unresolved byte before ``floor``; phases older than
        every stream's floor on the port are dropped."""
        self._streams[(device, stream)] = (base, floor)
        self._queues[device].prune(min(f for (d, _s), (_b, f) in self._streams.items() if d == device))

    def admitted(self, record: dict, device: str, progress: list[Progress]) -> None:
        """Reply k took these bytes of tagged ranges."""
        keys, skipped = [], 0
        for step in progress:
            if step.packet == SKIPPED:
                skipped += step.count
                self._add(device, "admitted", "skipped", step.count)
                continue
            self._add(device, "admitted", "packet", step.count)
            self._book.admitted(step.packet, step.count)
            if step.complete:
                keys.append(self._book.end(step.packet, "delivered", device))
        self._tally.count(record, device, TO_SITL, keys, skipped)

    def park(self, leftovers: list[Leftover]) -> None:
        for left in leftovers:
            kind = "skipped" if left.packet == SKIPPED else "packet"
            self._add(left.device, "undelivered", kind, left.pending)
            if kind == "packet":
                self._book.end(left.packet, "undelivered", left.admitted, left.pending)

    def held(self, fed: Mapping[tuple[str, Hashable], int]) -> None:
        """At closing: every stream's unresolved remainder (``fed``: each stream's
        bytes written), per phase."""
        for (device, stream), (base, floor) in self._streams.items():
            end = base + fed[(device, stream)]
            for phase, count in self._queues[device].resolve(floor, end):
                self._add(device, phase[0], "held", count)

    def summary(self) -> dict[str, dict[str, dict[str, int]]]:
        return {device: {phase: dict(sorted(kinds.items())) for phase, kinds in sorted(phases.items())}
                for device, phases in sorted(self._bytes.items())}

    def _add(self, device: str, phase: str, kind: str, count: int) -> None:
        if count:
            kinds = self._bytes.setdefault(device, {}).setdefault(phase, {})
            kinds[kind] = kinds.get(kind, 0) + count


__all__ = ["StreamAccount"]
