"""Link models: delay, jitter, loss, bandwidth and outage (step 3 of ``deterministic_dev_sim.md``).

Two models, applied by the coordinator's ports (``coordinator_router``,
``coordinator_queues``); product code never sees them:

- ``serial_link``, NavPy i to and from autopilot i (SITL i's serial0), a byte
  stream: each tick's bytes arrive ``delay + jitter`` ticks later, never
  before earlier bytes (no loss, no reordering). NavPy's bytes to the
  autopilot also share ``bytes_per_s``. The autopilot's bytes to NavPy are
  not rate limited: the SITL already sends more than a 115200 baud line
  carries (12.2 kB/s measured), and a real autopilot fits its streams to the
  UART's free space, which only the firmware can model (todo 84).
- ``radio_link``, the router path (every SITL's serial1 and the backend):
  each endpoint's radio sends its packets in order at ``bytes_per_s``; each
  router copy then is lost with probability ``loss``, or in an outage
  window, else it arrives ``delay + jitter`` ticks after it left, never
  before an earlier copy on the same link (source to destination).

Delays are whole ticks. Every draw comes from the model's own stream per link
(``sim_profile.model_stream``), values from ``random()`` alone, so a model or a
link never shifts another's draws. A rate gives each tick the bytes of
``(t_prev, t_k]`` at ``bytes_per_s``; bytes a tick does not use are gone (a
line cannot send faster later), only the fraction of a byte carries over. A
line's first tick carries nothing: it has no previous tick.

Outage windows are the model's mechanism; step 7 schedules them as faults,
so no profile draws one yet.
"""

from __future__ import annotations

import heapq
import math
import random
from collections import deque
from collections.abc import Hashable, Sequence
from dataclasses import dataclass, field

from navpy.sim.lockstep.sim_profile import Realization, model_stream

SERIAL_LINK = "serial_link"
RADIO_LINK = "radio_link"
_MICROS = 1_000_000


@dataclass(frozen=True)
class LinkSpec:
    delay_ticks: int
    jitter_ticks: int
    loss: float = 0.0
    #: The line rate; None is no limit.
    bytes_per_s: int | None = None
    #: Ticks ``[start, end)`` in which the link loses every packet.
    outages: tuple[tuple[int, int], ...] = ()

    def __post_init__(self) -> None:
        if self.delay_ticks < 0 or self.jitter_ticks < 0:
            raise ValueError(f"link delay and jitter must be >= 0 ticks: {self}")
        if not 0.0 <= self.loss <= 1.0:
            raise ValueError(f"link loss must be within [0, 1]: {self.loss}")
        if self.bytes_per_s is not None and self.bytes_per_s <= 0:
            raise ValueError(f"a link rate must be positive: {self.bytes_per_s}")

    def in_outage(self, tick: int) -> bool:
        return any(start <= tick < end for start, end in self.outages)


@dataclass(frozen=True)
class LinkModels:
    """The run's link models (None: off) and the run seed their streams draw from."""

    seed: int
    serial: LinkSpec | None = None
    radio: LinkSpec | None = None


def link_models(realization: Realization) -> LinkModels | None:
    """The run's draw as link specs; None while both link models are off."""
    serial, radio = realization.model(SERIAL_LINK), realization.model(RADIO_LINK)
    serial_spec = radio_spec = None
    if serial is not None and serial.on:
        serial_spec = LinkSpec(int(serial.params["delay_ticks"]), int(serial.params["jitter_ticks"]),
                               bytes_per_s=int(serial.params["bytes_per_s"]))
    if radio is not None and radio.on:
        radio_spec = LinkSpec(int(radio.params["delay_ticks"]), int(radio.params["jitter_ticks"]),
                              float(radio.params["loss"]), int(radio.params["bytes_per_s"]))
    if serial_spec is None and radio_spec is None:
        return None
    return LinkModels(realization.seed, serial_spec, radio_spec)


class LinkClock:
    """One directed link's arrival ticks: delay plus seeded jitter, never before an earlier one."""

    def __init__(self, spec: LinkSpec, stream: random.Random) -> None:
        self._spec = spec
        self._stream = stream
        self._last = 0

    def arrival(self, tick: int) -> int:
        span = self._spec.jitter_ticks
        jitter = min(span, math.floor(self._stream.random() * (span + 1))) if span else 0
        self._last = max(self._last, tick + self._spec.delay_ticks + jitter)
        return self._last


class ByteRate:
    """The bytes a line at ``bytes_per_s`` carries in each tick (see the module doc)."""

    def __init__(self, bytes_per_s: int) -> None:
        self._rate = bytes_per_s
        self._t_us: int | None = None
        #: The fraction of a byte carried over, in byte-microseconds.
        self._carry = 0

    def grant(self, t_us: int) -> int:
        if self._t_us is None:
            self._t_us = t_us
            return 0
        total = self._carry + self._rate * max(0, t_us - self._t_us)
        self._t_us = t_us
        self._carry = total % _MICROS
        return total // _MICROS


class ByteDelay:
    """A byte stream through a :class:`LinkClock`: each push arrives whole, in order."""

    def __init__(self, clock: LinkClock) -> None:
        self._clock = clock
        self._held: deque[tuple[int, bytes]] = deque()

    def push(self, tick: int, data: bytes) -> None:
        if data:
            self._held.append((self._clock.arrival(tick), data))

    def release(self, tick: int) -> bytes:
        out = bytearray()
        while self._held and self._held[0][0] <= tick:
            out += self._held.popleft()[1]
        return bytes(out)

    @property
    def pending(self) -> int:
        return sum(len(data) for _tick, data in self._held)

    def flush(self) -> bytes:
        """Every held byte (at parking); the line is then empty."""
        data = b"".join(data for _tick, data in self._held)
        self._held.clear()
        return data


@dataclass(frozen=True)
class Arrival:
    destination: str
    copy: Hashable
    data: bytes
    #: The tick the copy left its sender's radio.
    left: int


@dataclass
class _Outgoing:
    data: bytes
    #: ``(destination, copy id)`` in routing order.
    copies: list[tuple[str, Hashable]]
    paid: int = 0


@dataclass
class _Sender:
    rate: ByteRate | None
    queue: deque[_Outgoing] = field(default_factory=deque)
    credit: int = 0
    credit_tick: int | None = None


class RadioLinks:
    """The router path's radios: one sender per endpoint, one link per (sender, destination)."""

    def __init__(self, spec: LinkSpec, seed: int, endpoints: Sequence[str]) -> None:
        self._spec = spec
        self._seed = seed
        self._senders = {endpoint: _Sender(ByteRate(spec.bytes_per_s) if spec.bytes_per_s else None)
                         for endpoint in endpoints}
        self._clocks: dict[tuple[str, str], LinkClock] = {}
        self._losses: dict[tuple[str, str], random.Random] = {}
        #: ``(arrival tick, leave order, Arrival)``.
        self._air: list[tuple[int, int, Arrival]] = []
        self._order = 0

    def send(self, source: str, data: bytes, copies: list[tuple[str, Hashable]]) -> None:
        """One packet ``source`` routed, with its copies for each destination endpoint."""
        if copies:
            self._senders[source].queue.append(_Outgoing(data, list(copies)))

    def step(self, tick: int, t_us: int) -> tuple[list[Arrival], list[tuple[Hashable, str]]]:
        """Send what the radios can by ``tick``, then what arrives by it: the arrivals in
        (arrival, leave) order, and the copies lost (``loss`` or ``outage``) as they left."""
        lost: list[tuple[Hashable, str]] = []
        for source, sender in self._senders.items():
            if sender.rate is not None and sender.credit_tick != tick:
                sender.credit, sender.credit_tick = sender.rate.grant(t_us), tick
            while sender.queue:
                head = sender.queue[0]
                if sender.rate is not None:
                    paid = min(len(head.data) - head.paid, sender.credit)
                    head.paid += paid
                    sender.credit -= paid
                    if head.paid < len(head.data):
                        break
                sender.queue.popleft()
                lost += self._leave(source, head, tick)
        arrivals = []
        while self._air and self._air[0][0] <= tick:
            arrivals.append(heapq.heappop(self._air)[2])
        return arrivals, lost

    def held(self) -> dict[str, int]:
        """Bytes waiting in each sender's radio (nonzero only), and ``air``: copies in flight."""
        held = {source: sum(len(item.data) - item.paid for item in sender.queue)
                for source, sender in self._senders.items()}
        held = {source: count for source, count in held.items() if count}
        if self._air:
            held["air"] = len(self._air)
        return held

    def park(self) -> list[tuple[Hashable, int]]:
        """Every copy still in a radio or in flight, with its bytes; the radios are then empty."""
        left = [(copy, len(item.data)) for sender in self._senders.values()
                for item in sender.queue for _destination, copy in item.copies]
        left += [(arrival.copy, len(arrival.data)) for _tick, _order, arrival in sorted(self._air)]
        for sender in self._senders.values():
            sender.queue.clear()
        self._air.clear()
        return left

    def _leave(self, source: str, item: _Outgoing, tick: int) -> list[tuple[Hashable, str]]:
        lost = []
        for destination, copy in item.copies:
            link = (source, destination)
            dropped = self._loss(link).random() < self._spec.loss if self._spec.loss else False
            if self._spec.in_outage(tick):
                lost.append((copy, "outage"))
            elif dropped:
                lost.append((copy, "loss"))
            else:
                self._order += 1
                heapq.heappush(self._air, (self._clock(link).arrival(tick), self._order,
                                           Arrival(destination, copy, item.data, tick)))
        return lost

    def _clock(self, link: tuple[str, str]) -> LinkClock:
        if link not in self._clocks:
            self._clocks[link] = LinkClock(self._spec, model_stream(
                self._seed, RADIO_LINK, f"{link[0]}-{link[1]}.jitter"))
        return self._clocks[link]

    def _loss(self, link: tuple[str, str]) -> random.Random:
        if link not in self._losses:
            self._losses[link] = model_stream(self._seed, RADIO_LINK, f"{link[0]}-{link[1]}.loss")
        return self._losses[link]


def serial_clock(models: LinkModels, sitl: int, direction: str) -> LinkClock:
    """SITL ``sitl``'s serial0 clock ``to_navpy`` or ``to_sitl``, on its own stream."""
    if models.serial is None:
        raise ValueError("the serial link model is off")
    return LinkClock(models.serial, model_stream(models.seed, SERIAL_LINK, f"serial0.{sitl}.{direction}"))


__all__ = ["Arrival", "ByteDelay", "ByteRate", "LinkClock", "LinkModels", "LinkSpec", "RADIO_LINK",
           "RadioLinks", "SERIAL_LINK", "link_models", "serial_clock"]
