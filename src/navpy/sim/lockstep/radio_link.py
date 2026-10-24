"""Radio byte loss: the bytes a radio transmits, some lost, and what the loss broke.

``docs/design/swarm-task-assignment-ack.md``, "Fix 1: loss on the radio
only" (todo 114). A radio is a SITL's serial1 or the backend's router
device. Each byte it transmits is lost with probability ``rate`` before the
router frames it; serial0 (ArduPilot to NavPy, a short cable) is clean.
Draws are geometric gaps between lost bytes from
``model_stream(seed, "uart_loss", "radio.<endpoint>")`` (``sitl<i>``,
``backend``), so the lost bytes
depend only on the seed and the bytes, never on how they are chunked; a
zero rate draws nothing.

The coordinator does the deletion, so it also knows what it broke. Per
lossy stream (one router framer's stream) it frames the clean bytes too and
keeps each surviving byte's clean offset. A lossy emission is genuine when
its bytes map to one contiguous clean range that is exactly a clean
packet's range; any other emission (an embedded or stitched frame) is
false. A clean packet that can no longer get a genuine emission is broken.
The journal's ``radio`` list holds, per tick:

- ``["broken", device, tick, [start, end], sha256, msgid, [src sys, comp],
  [target sys, comp], fields]``: clean offsets, the tick the packet was
  written, ``fields`` as an ``emitted`` record's (``coordinator_packets``);
- ``["false", device, packet id]``: an emission that is no clean packet.

At the end of a stream every clean packet still waiting is broken: it is
complete in the clean stream, so only the loss kept it from the router (a
lost byte in it, or a frame the loss started that the router still holds,
``RoutedPorts.closing`` journals those bytes).
"""

from __future__ import annotations

import math
from bisect import bisect_right
from collections.abc import Hashable, Iterable, Iterator, Mapping
from dataclasses import dataclass
from random import Random
from typing import Any

from navpy.sim.lockstep.coordinator_journal import sha256
from navpy.sim.lockstep.coordinator_packets import PacketId, navlink_fields
from navpy.sim.lockstep.router_entries import EntryTable
from navpy.sim.lockstep.router_framer import RouterFramer, Segment
from navpy.sim.lockstep.sim_profile import Realization, model_stream
from navpy.sim.lockstep.sitl_models import UART_LOSS

#: The profile model whose draw is the radio's loss (``loss_pct``, a per-byte rate in %).
RADIO_LOSS_MODEL = UART_LOSS
#: The model name the journal's grid records the rate under.
RADIO_LOSS_RECORD = "radio_byte_loss"
#: A lossy stream: its router framer's key ``(origin, generation, device)``.
StreamKey = tuple[Hashable, int, str]
#: A lossy emission: its packet id, its offset in the lossy stream and its size.
Emission = tuple[PacketId, int, int]


class ByteLoss:
    """Which bytes of one radio's transmissions are lost."""

    def __init__(self, rate: float, draws: Random) -> None:
        if not 0.0 <= rate < 1.0:
            raise ValueError(f"radio byte loss rate {rate} is not in [0, 1)")
        self._rate = rate
        self._draws = draws
        #: Bytes still to survive before the next lost one (None: not drawn yet).
        self._gap: int | None = None

    def drop(self, data: bytes) -> tuple[bytes, list[int]]:
        """The surviving bytes, and the indices in ``data`` of the lost ones."""
        if self._rate == 0.0 or not data:
            return data, []
        lost: list[int] = []
        at = 0
        while True:
            if self._gap is None:
                self._gap = self._draw()
            if at + self._gap >= len(data):
                self._gap -= len(data) - at
                break
            at += self._gap
            lost.append(at)
            at += 1
            self._gap = None
        kept = bytearray()
        start = 0
        for index in lost:
            kept += data[start:index]
            start = index + 1
        kept += data[start:]
        return bytes(kept), lost

    def _draw(self) -> int:
        """Geometric: the surviving bytes before the next lost one."""
        return math.floor(math.log1p(-self._draws.random()) / math.log1p(-self._rate))


@dataclass(frozen=True)
class _Clean:
    end: int
    tick: int
    segment: Segment


class RadioStream:
    """One lossy stream: the clean stream framed beside it, and the offset map."""

    def __init__(self, device: str, loss: ByteLoss, table: EntryTable) -> None:
        self._device = device
        self._loss = loss
        self._clean = RouterFramer(table)
        #: ``(lossy start, clean start)`` of each run of surviving bytes; a lost byte starts a run.
        self._runs: list[tuple[int, int]] = [(0, 0)]
        self._lossy_fed = 0
        #: Clean packets with no genuine emission yet, by clean start (in order).
        self._pending: dict[int, _Clean] = {}

    def transmit(self, tick: int, data: bytes) -> bytes:
        """What survives of ``data``; frames the clean bytes first."""
        base = self._clean.fed
        for segment in self._clean.feed(data):
            if segment.packet is not None:
                self._pending[segment.start] = _Clean(segment.start + len(segment.data), tick, segment)
        kept, lost = self._loss.drop(data)
        for count, index in enumerate(lost):
            self._runs.append((self._lossy_fed + index - count, base + index + 1))
        self._lossy_fed += len(kept)
        return kept

    def observe(self, emissions: Iterable[Emission], held_start: int) -> list[list[object]]:
        """Match the lossy emissions; settle the clean packets now out of reach."""
        events: list[list[object]] = []
        for packet_id, start, size in emissions:
            first, run = self._clean_at(start)
            last, last_run = self._clean_at(start + size - 1)
            pending = self._pending.get(first)
            if run == last_run and pending is not None:
                # Contiguous from a clean packet's start: the same header, so the same size.
                assert pending.end == last + 1, f"{self._device}: emission {packet_id} is not its packet"
                del self._pending[first]
            else:
                events.append(["false", self._device, list(packet_id)])
        bound = self._clean_at(held_start)[0] if held_start < self._lossy_fed else self._clean.fed
        while self._pending and next(iter(self._pending)) < bound:
            events.append(self._broken(self._pending.pop(next(iter(self._pending)))))
        keep = self._clean_at(held_start)[1] if held_start < self._lossy_fed else len(self._runs) - 1
        del self._runs[:keep]
        return events

    def settle(self) -> list[list[object]]:
        """The end of the stream: every clean packet still waiting is broken."""
        events = [self._broken(pending) for pending in self._pending.values()]
        self._pending.clear()
        return events

    def _clean_at(self, lossy: int) -> tuple[int, int]:
        """A lossy offset's clean offset, and the index of its run."""
        index = bisect_right(self._runs, lossy, key=lambda run: run[0]) - 1
        lossy_start, clean_start = self._runs[index]
        return clean_start + lossy - lossy_start, index

    def _broken(self, pending: _Clean) -> list[object]:
        segment = pending.segment
        packet = segment.packet
        assert packet is not None
        return ["broken", self._device, pending.tick, [segment.start, pending.end], sha256(segment.data),
                packet.msgid, list(packet.source), list(packet.target), navlink_fields(packet)]


class RadioLinks:
    """The fleet's radios: SITL serial1 devices and the backend's router device."""

    def __init__(self, loss_pct: float, seed: int, endpoints: Mapping[str, str], table: EntryTable) -> None:
        """``endpoints``: each radio's device and its router endpoint, which names its draws."""
        self.loss_pct = loss_pct
        rate = self.rate = loss_pct / 100.0
        self._table = table
        self._losses = {device: ByteLoss(rate, model_stream(seed, RADIO_LOSS_MODEL, f"radio.{endpoint}"))
                        for device, endpoint in endpoints.items()}
        self._streams: dict[StreamKey, RadioStream] = {}
        self._events: list[list[object]] = []

    def lossy(self, device: str) -> bool:
        return self.rate > 0.0 and device in self._losses

    def transmit(self, key: StreamKey, tick: int, data: bytes) -> bytes:
        """What survives of ``data`` on stream ``key`` (its device must be :meth:`lossy`)."""
        stream = self._streams.get(key)
        if stream is None:
            device = key[2]
            stream = self._streams[key] = RadioStream(device, self._losses[device], self._table)
        return stream.transmit(tick, data)

    def observe(self, key: StreamKey, emissions: Iterable[Emission], held_start: int) -> None:
        self._events += self._streams[key].observe(emissions, held_start)

    def settle(self, devices: Iterable[str] | None = None) -> None:
        """End the streams of ``devices`` (every stream if None)."""
        ending = None if devices is None else set(devices)
        for key in list(self._streams):
            if ending is None or key[2] in ending:
                self._events += self._streams.pop(key).settle()

    def take(self) -> list[list[object]]:
        events, self._events = self._events, []
        return events


def radio_loss_pct(realization: Realization) -> float:
    """The radio's loss in % per byte: the run's ``uart_loss`` draw, 0 while it is off."""
    draw = realization.model(RADIO_LOSS_MODEL)
    return float(draw.params["loss_pct"]) if draw is not None and draw.on else 0.0


def journal_radio_loss_pct(records: Iterable[Mapping[str, Any]]) -> float:
    """The radio loss a run's journal records in its grid (0 for a journal from before it)."""
    for record in records:
        if "grid" in record:
            return float(record["grid"].get("radio_loss", {}).get("loss_pct", 0.0))
    return 0.0


def radio_events(records: Iterable[Mapping[str, Any]]) -> Iterator[list[Any]]:
    """Every ``radio`` event of a journal: tick and drain records, the parked record, the close."""
    for record in records:
        if "radio" in record:
            yield from record["radio"]
        elif "parked" in record:
            yield from record["parked"].get("radio", ())


__all__ = ["ByteLoss", "RADIO_LOSS_MODEL", "RADIO_LOSS_RECORD", "RadioLinks", "RadioStream",
           "journal_radio_loss_pct", "radio_events", "radio_loss_pct"]
