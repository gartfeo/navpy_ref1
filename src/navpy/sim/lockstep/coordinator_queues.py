"""The coordinator's per-port queues toward the SITLs: carried over, never dropped.

``lockstep_fleet.md``, Queues (owner Q4). One FIFO per destination port
(SITL i serial0, SITL i serial1). Routing and NavPy egress append to the
tail; reply k takes ``n = min(len(queue), 0xFFFF, FIFO_CAP - fifo_bytes(frame k))``
bytes from the head, so old bytes always go before new ones and the firmware
FIFO can never overflow (nothing else enters it before reply k):

- ``0xFFFF`` is the wire's per-port block (``step_wire.MAX_INGRESS_PER_PORT``),
  the firmware's ``MAX_BLOCK`` (``NavPySerialStep.cpp:39``).
- :data:`FIFO_CAP` is the firmware ingress FIFO's usable size: a
  ``ByteBuffer{BUFFER_BYTES + 1}`` with ``BUFFER_BYTES = 1U << 20``
  (``SIM_NavPyStepPort.h:30``, ``SIM_NavPyStepPort.cpp:25``) keeps one byte
  free, so it holds ``BUFFER_BYTES``; a push beyond its space is
  ``ingress_fifo_full`` (``SIM_NavPyStepPort.cpp:44``). Pinned against the
  firmware source by ``test_coordinator_queues``.

A queue is a byte stream with absolute offsets: bytes move as they are
appended, whatever their framing (a packet's prefix may go out before its
suffix is written). Every byte carries its phase from the moment it is
written (:class:`PhaseLedger`): queued, admitted by reply j, undelivered at
parking, or written in drain tick k (drain bytes are never queued, but take
the next offsets, so a stream stays contiguous). Framing classifies a range
once it resolves it (:meth:`PortQueue.resolve` gives the range's phases); a
range still queued is tagged (:meth:`PortQueue.register`), and each
admission reports the bytes it took per tag (a packet id, or :data:`SKIPPED`
for bytes framing skipped), so a packet split across replies shows each
fragment's tick. What is still queued at parking is returned for the
journal (``undelivered``), then dropped.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Hashable
from dataclasses import dataclass

from navpy.sim.lockstep.coordinator_journal import sha256
from navpy.sim.lockstep.step_wire import MAX_INGRESS_PER_PORT

#: The firmware ingress FIFO's usable bytes (see the module doc).
FIFO_CAP = 1 << 20
#: One reply's bytes per port (the wire and the firmware ``MAX_BLOCK``).
MAX_BLOCK = MAX_INGRESS_PER_PORT
#: The tag of bytes framing skipped (they belong to no packet).
SKIPPED = ("skipped",)
#: A byte's phase: ``QUEUED``, ``("admitted", j)``, ``UNDELIVERED`` or ``("drain", k)``.
Phase = tuple
QUEUED: Phase = ("queued",)
UNDELIVERED: Phase = ("undelivered",)


@dataclass
class _Tagged:
    packet: Hashable
    start: int
    end: int


@dataclass(frozen=True)
class Progress:
    """Bytes of one tagged range admitted by one reply."""

    packet: Hashable
    count: int
    #: The reply took the range's last byte: a packet is delivered.
    complete: bool


@dataclass(frozen=True)
class Leftover:
    """A tagged range still (partly) queued at parking."""

    device: str
    packet: Hashable
    admitted: int
    pending: int


def admission_room(fifo_bytes: int) -> int:
    """What reply k may carry on a port whose frame k reported ``fifo_bytes``."""
    return max(0, min(MAX_BLOCK, FIFO_CAP - fifo_bytes))


class PhaseLedger:
    """Each byte's phase by offset, from its write until framing resolves it."""

    def __init__(self) -> None:
        #: ``[start, end, phase]``, ascending and contiguous.
        self._ranges: list[list] = []

    def write(self, start: int, count: int, phase: Phase) -> None:
        if count:
            self._ranges.append([start, start + count, phase])

    def relabel(self, start: int, end: int, old: Phase, new: Phase) -> None:
        """The bytes of ``[start, end)`` in phase ``old`` move to ``new``."""
        out: list[list] = []
        for a, b, phase in self._ranges:
            if phase != old or b <= start or a >= end:
                out.append([a, b, phase])
                continue
            cuts = sorted({a, b, max(a, start), min(b, end)})
            out.extend([x, y, new if start <= x and y <= end else phase]
                       for x, y in zip(cuts, cuts[1:]))
        self._ranges = out

    def resolve(self, start: int, end: int) -> list[tuple[Phase, int]]:
        """``(phase, bytes)`` of ``[start, end)``, in byte order (adjacent equal phases merged)."""
        parts: list[tuple[Phase, int]] = []
        for a, b, phase in self._ranges:
            if a < end and b > start:
                count = min(end, b) - max(start, a)
                if parts and parts[-1][0] == phase:
                    count += parts.pop()[1]
                parts.append((phase, count))
        return parts

    def prune(self, floor: int) -> None:
        """Bytes before ``floor`` are resolved: their phases are no longer needed."""
        self._ranges = [entry for entry in self._ranges if entry[1] > floor]


class PortQueue:
    def __init__(self, device: str = "") -> None:
        self._device = device
        self._data = bytearray()
        #: The absolute offset of the first queued byte.
        self._head = 0
        #: Tagged ranges not yet fully admitted, in stream order.
        self._tagged: deque[_Tagged] = deque()
        self.phases = PhaseLedger()
        #: The next offset a written byte gets (after parking, drain writes go on).
        self._end = 0
        #: The stream whose bytes are written now, and the queue offset of its offset 0.
        self._stream: tuple[Hashable, int] | None = None

    def stream_base(self, stream: Hashable, fed: int) -> int:
        """The queue offset of ``stream``'s first byte; ``fed`` of its bytes came before
        this write. One stream writes at a time (a participant generation), so its
        bytes are contiguous here."""
        if self._stream is None or self._stream[0] != stream:
            self._stream = (stream, self._end - fed)
        return self._stream[1]

    @property
    def tail(self) -> int:
        """The absolute offset the next queued byte gets."""
        return self._head + len(self._data)

    @property
    def pending(self) -> int:
        return len(self._data)

    def append(self, data: bytes, packet: Hashable | None = None) -> None:
        """Queue ``data``; ``packet`` tags all of it (a whole packet)."""
        start = self.tail
        self._data += data
        self.phases.write(start, len(data), QUEUED)
        self._end = self.tail
        if packet is not None and data:
            self.register(packet, start, self.tail)

    def write_drain(self, count: int, tick: int) -> None:
        """``count`` bytes written in drain tick ``tick``: never queued or admitted."""
        self.phases.write(self._end, count, ("drain", tick))
        self._end += count

    def resolve(self, start: int, end: int) -> list[tuple[Phase, int]]:
        return self.phases.resolve(start, end)

    def register(self, packet: Hashable, start: int, end: int) -> None:
        """Tag ``[start, end)``: later admissions report their bytes of it."""
        if end > self._head:
            self._tagged.append(_Tagged(packet, start, end))

    def prune(self, floor: int) -> None:
        self.phases.prune(floor)

    def admit(self, room: int, tick: int) -> tuple[bytes, list[Progress]]:
        count = min(room, len(self._data))
        start, end = self._head, self._head + count
        data = bytes(self._data[:count])
        del self._data[:count]
        self._head = end
        self.phases.relabel(start, end, QUEUED, ("admitted", tick))
        progress = []
        for tagged in self._tagged:
            if tagged.start >= end:
                break
            overlap = min(end, tagged.end) - max(start, tagged.start)
            if overlap > 0:
                progress.append(Progress(tagged.packet, overlap, tagged.end <= end))
        while self._tagged and self._tagged[0].end <= end:
            self._tagged.popleft()
        return data, progress

    def park(self) -> tuple[bytes, list[Leftover]]:
        """Everything still queued, then the queue is empty; those bytes' phase is
        now undelivered (a frame still incomplete may resolve in drain)."""
        data = bytes(self._data)
        leftovers = [Leftover(self._device, t.packet, max(0, self._head - t.start),
                              t.end - max(self._head, t.start)) for t in self._tagged]
        self.phases.relabel(self._head, self.tail, QUEUED, UNDELIVERED)
        self._head = self.tail
        self._data.clear()
        self._tagged.clear()
        return data, leftovers


class ReplyQueues:
    """Every SITL port's queue, by device."""

    def __init__(self, devices: tuple[str, ...]) -> None:
        self._queues = {device: PortQueue(device) for device in devices}

    def __getitem__(self, device: str) -> PortQueue:
        return self._queues[device]

    def admit(self, device: str, fifo_bytes: int, tick: int) -> tuple[bytes, list[Progress]]:
        return self._queues[device].admit(admission_room(fifo_bytes), tick)

    def pending(self) -> dict[str, int]:
        """Bytes still queued per port (every port, zero included)."""
        return {device: queue.pending for device, queue in self._queues.items()}

    def park(self) -> tuple[dict[str, dict[str, object]], list[Leftover]]:
        """The undelivered bytes per port (count and sha256; nonzero only) and the
        tagged ranges they belong to; the queues are then empty."""
        summary: dict[str, dict[str, object]] = {}
        leftovers: list[Leftover] = []
        for device, queue in self._queues.items():
            data, tagged = queue.park()
            if data:
                summary[device] = {"bytes": len(data), "sha256": sha256(data)}
            leftovers.extend(tagged)
        return summary, leftovers


__all__ = ["FIFO_CAP", "Leftover", "MAX_BLOCK", "PhaseLedger", "PortQueue", "Progress", "QUEUED",
           "ReplyQueues", "SKIPPED", "UNDELIVERED", "admission_room"]
