"""The SITL links: tick step 1 (read and check frame k, the parked one too) and the replies.

Per SITL (:class:`FrameReader`): EOF, a malformed frame or the wrong tick is
a coordinator fault. A nonzero out-of-loop advance count is a host fault.
The grid check: ``source_us`` must rise strictly, by at most the manifest's
``max_frame_gap_us`` (Δ_max), from that SITL's previous frame; today's
decoder accepts any header (``step_wire.py``), so this is where time is
checked.

The fleet (:class:`FleetFrames`, ``lockstep_fleet.md``, "Tick k, step 1"):
frame k is read from SITL 1, 2, ... in order, each checked as it arrives;
then the fleet clock check: every ``source_us`` must be equal, and t_k is
that value. A mismatch is ``CoordinatorFault("fleet_clock")``, journaled
with the values. So a SITL whose own grid is broken faults on the grid
first. Every read and reply draws on the tick's exchange deadline.
"""

from __future__ import annotations

import math
from typing import Protocol

from navpy.sim.lockstep.coordinator_deadline import FRAME, REPLY, ExchangeDeadline
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault
from navpy.sim.lockstep.step_wire import Frame, StepWireError, pack_reply, unpack_frame


class SitlLink(Protocol):
    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        """The next frame; ``timeout_s`` bounds the wait (the link's own budget too)."""

    def send_reply(self, reply: bytes, timeout_s: float = math.inf) -> None: ...


class FrameReader:
    """One SITL's link and its grid."""

    def __init__(self, link: SitlLink, max_frame_gap_us: int) -> None:
        self._link = link
        self._max_gap_us = max_frame_gap_us
        self._last_us: int | None = None

    def read(self, k: int, deadline: ExchangeDeadline | None = None) -> tuple[Frame, bytes]:
        deadline = deadline or ExchangeDeadline()
        try:
            raw = self._link.recv_frame(deadline.budget(FRAME, math.inf))
            frame = unpack_frame(raw, expected_tick=k)
        except (ConnectionError, OSError, StepWireError) as error:
            deadline.raise_if_spent(FRAME, error)
            raise CoordinatorFault("frame_invalid", f"SITL frame {k}: {error}", tick=k) from error
        deadline.check(FRAME)
        self.check(frame)
        return frame, raw

    def reply(self, k: int, ingress: tuple[bytes, ...],
              deadline: ExchangeDeadline | None = None) -> bytes:
        """Reply k; returns its raw bytes for the journal."""
        deadline = deadline or ExchangeDeadline()
        try:
            raw = pack_reply(k, ingress)
            self._link.send_reply(raw, deadline.budget(REPLY, math.inf))
        except (ConnectionError, OSError, StepWireError) as error:
            deadline.raise_if_spent(REPLY, error)
            raise CoordinatorFault("reply_failed", f"reply {k}: {error}", tick=k) from error
        deadline.check(REPLY)
        return raw

    def check(self, frame: Frame) -> None:
        if frame.out_of_loop_advances:
            raise HostFault("out_of_loop_advance",
                            f"out-of-loop clock advances: {frame.out_of_loop_advances}", tick=frame.tick)
        if self._last_us is not None:
            gap = frame.source_us - self._last_us
            if gap <= 0 or gap > self._max_gap_us:
                raise CoordinatorFault(
                    "grid", f"grid check at tick {frame.tick}: source_us step {gap} us outside "
                    f"(0, {self._max_gap_us}]", tick=frame.tick)
        self._last_us = frame.source_us


class FleetFrames:
    """The fleet's SITL links in SITL order, with their port devices."""

    def __init__(self, links: tuple[SitlLink, ...], ports: tuple[tuple[str, ...], ...],
                 max_frame_gap_us: int, deadline: ExchangeDeadline) -> None:
        if len(links) != len(ports):
            raise ValueError(f"{len(links)} SITL links for {len(ports)} SITLs in the manifest")
        self._readers = tuple(FrameReader(link, max_frame_gap_us) for link in links)
        self._ports = ports
        self._deadline = deadline

    def read(self, k: int) -> list[tuple[Frame, bytes]]:
        """Frame k of every SITL, each checked, then the fleet clock."""
        self._deadline.open(k)
        frames = [reader.read(k, self._deadline) for reader in self._readers]
        times = [frame.source_us for frame, _raw in frames]
        if len(set(times)) != 1:
            raise CoordinatorFault("fleet_clock", f"tick {k}: unequal source_us {times}", tick=k,
                                   context={"source_us": times})
        return frames

    def egress(self, frames: list[Frame]) -> dict[str, bytes]:
        """Each SITL port's egress by its device."""
        return {device: port.egress for frame, devices in zip(frames, self._ports)
                for device, port in zip(devices, frame.ports)}

    def reply(self, k: int, ingress: dict[str, bytes]) -> list[bytes]:
        """Reply k to SITL 1, 2, ...; the reply to SITL 1 anchors tick k+1's deadline."""
        raws = []
        for index, (reader, devices) in enumerate(zip(self._readers, self._ports)):
            raws.append(reader.reply(k, tuple(ingress.get(device, b"") for device in devices),
                                     self._deadline))
            if index == 0:
                self._deadline.anchor()
        return raws


__all__ = ["FleetFrames", "FrameReader", "SitlLink"]
