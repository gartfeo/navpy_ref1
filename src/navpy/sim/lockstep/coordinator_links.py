"""The link models at the coordinator's port seams (step 3 of ``deterministic_dev_sim.md``).

:class:`PortLinks` is what :class:`~navpy.sim.lockstep.coordinator_router.RoutedPorts`
asks at each seam; ``link_model`` holds the models themselves. Without link
models (``models`` None) every answer is the perfect link's: bytes pass the
same tick, NavPy's bytes are ready at once, no rate applies, and a router copy
is delivered as it is routed.

- SITL i's serial0 egress passes :meth:`to_navpy` before framing.
- NavPy i's egress is queued with :meth:`ready`'s tick; :attr:`rates` are the
  serial0 ports' line rates (``coordinator_queues``).
- Router copies go through :meth:`send`; :meth:`step` sends what the radios can
  and returns what arrives. A lost copy ends ``lost``; an arrived one gets an
  ``arrived`` event (``coordinator_packets``).
- :meth:`held` is the tick record's ``link_held``; :meth:`park_serial` and
  :meth:`park_radio` end what the links still hold at parking.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from navpy.sim.lockstep.coordinator_packets import PacketBook, PacketId
from navpy.sim.lockstep.link_model import ByteDelay, ByteRate, LinkClock, LinkModels, RadioLinks, \
    serial_clock

#: A router copy to deliver: its device, id and bytes.
Delivery = tuple[str, PacketId, bytes]


class PortLinks:
    def __init__(self, models: LinkModels | None, sitl_ports: Sequence[tuple[str, ...]],
                 endpoint_device: Mapping[str, str], book: PacketBook) -> None:
        self.on = models is not None
        self._book = book
        self.endpoint_device = endpoint_device
        self._to_navpy: dict[str, ByteDelay] = {}
        self._to_sitl: dict[str, LinkClock] = {}
        #: Each rate-limited port's line rate.
        self.rates: dict[str, ByteRate] = {}
        serial = models.serial if models is not None else None
        if serial is not None:
            for index, ports in enumerate(sitl_ports, start=1):
                self._to_navpy[ports[0]] = ByteDelay(serial_clock(models, index, "to_navpy"))
                self._to_sitl[ports[0]] = serial_clock(models, index, "to_sitl")
                if serial.bytes_per_s:
                    self.rates[ports[0]] = ByteRate(serial.bytes_per_s)
        self._radio = (RadioLinks(models.radio, models.seed, list(endpoint_device))
                       if models is not None and models.radio is not None else None)

    def to_navpy(self, k: int, device: str, data: bytes) -> bytes:
        """SITL ``device``'s egress of frame k in; what reaches NavPy's side by tick k out."""
        line = self._to_navpy.get(device)
        if line is None:
            return data
        line.push(k, data)
        return line.release(k)

    def ready(self, k: int, device: str, data: bytes) -> int | None:
        """The first reply that may admit NavPy's bytes of tick k (None: reply k)."""
        clock = self._to_sitl.get(device)
        return clock.arrival(k) if clock is not None and data else None

    def send(self, k: int, t_us: int, source: str, data: bytes,
             copies: list[tuple[str, PacketId]]) -> list[Delivery]:
        """One routed packet's copies (``(destination endpoint, id)``); returns what to deliver now."""
        if self._radio is None:
            return [(self.endpoint_device[destination], copy, data) for destination, copy in copies]
        self._radio.send(source, data, copies)
        return self.step(k, t_us)

    def step(self, k: int, t_us: int) -> list[Delivery]:
        if self._radio is None:
            return []
        arrivals, lost = self._radio.step(k, t_us)
        for copy, reason in lost:
            self._book.end(copy, "lost", reason)
        for arrival in arrivals:
            self._book.arrived(arrival.copy, arrival.left)
        return [(self.endpoint_device[a.destination], a.copy, a.data) for a in arrivals]

    def held(self) -> dict[str, dict[str, int]]:
        """What the links hold (nonzero only): bytes in each serial link toward NavPy, each
        radio's waiting bytes and the copies in flight (``air``). NavPy's bytes toward a
        SITL wait in its queue (the record's ``pending``)."""
        held: dict[str, dict[str, int]] = {}
        to_navpy = {device: line.pending for device, line in self._to_navpy.items() if line.pending}
        if to_navpy:
            held["to_navpy"] = to_navpy
        radio = self._radio.held() if self._radio is not None else {}
        if radio:
            held["radio"] = radio
        return held

    def park_serial(self, device: str) -> bytes:
        """The bytes still in ``device``'s serial link toward NavPy; the line is then empty."""
        line = self._to_navpy.get(device)
        return line.flush() if line is not None else b""

    def park_radio(self) -> None:
        """Every copy still in a radio or in flight ends ``in_flight``."""
        if self._radio is not None:
            for copy, count in self._radio.park():
                self._book.end(copy, "in_flight", count)


__all__ = ["Delivery", "PortLinks"]
