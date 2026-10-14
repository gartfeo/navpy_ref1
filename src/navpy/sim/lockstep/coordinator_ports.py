"""How a tick's SITL bytes reach the participants and theirs reach the SITLs.

Two strategies behind one interface (:class:`TickPorts`), chosen by the
manifest (:func:`make_ports`):

- :class:`DirectPorts`, the slice's links (``lockstep_slice.md``): each SITL
  port's egress goes to the participant owning its device, and the
  participants' bytes for a SITL port become reply k's ingress. A one-SITL
  run (and the 10a toy) keeps this path and its journal until step 10c
  moves the slice onto the router (owner Q1).
- :class:`~navpy.sim.lockstep.coordinator_router.RoutedPorts`, a routed fleet
  (``Manifest.router``, ``lockstep_fleet.md``, Router and delivery).

Per tick: :meth:`~TickPorts.frames` (step 1, frame k's egress), then
:meth:`~TickPorts.inbound` once the tick's members are admitted (what each
owned device gets in ``TICK k``; bytes for an unowned device are discarded
and counted), :meth:`~TickPorts.outbound` per ``DONE`` port block, and
:meth:`~TickPorts.reply` (reply k's ingress per SITL device). A drain tick
ends with :meth:`~TickPorts.drained`, the parked frame goes to
:meth:`~TickPorts.park`, and :meth:`~TickPorts.closing` gives the run's
closing counts record.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Protocol

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_mavlink import DRAIN_TO_SITL, FROM_SITL, TO_SITL, MavlinkTally
from navpy.sim.lockstep.coordinator_members import Member, Roster
from navpy.sim.lockstep.coordinator_router import RoutedPorts
from navpy.sim.lockstep.coordinator_types import Manifest
from navpy.sim.lockstep.step_wire import Frame


class TickPorts(Protocol):
    #: Tick records name each member's generation too (a routed fleet's receivers).
    member_generation: bool

    def frames(self, k: int, record: TickRecord, egress: Mapping[str, bytes]) -> None: ...

    def inbound(self, k: int, record: TickRecord, owners: Mapping[str, Member]) -> dict[str, bytes]: ...

    def outbound(self, k: int, member: Member, device: str, data: bytes, record: TickRecord, *,
                 draining: bool) -> None: ...

    def reply(self, k: int, record: TickRecord, frames: Sequence[Frame]) -> dict[str, bytes]: ...

    def drained(self, record: TickRecord) -> None: ...

    def park(self, parked: dict, egress: Mapping[str, bytes]) -> None: ...

    def closing(self) -> dict[str, object]: ...


class DirectPorts:
    """The slice's direct links; the tally decodes the bytes (``MavlinkTally.feed``)."""

    #: The slice's tick records name members by ordinal only (its journal bytes).
    member_generation = False

    def __init__(self, sitl_devices: tuple[str, ...]) -> None:
        self._sitl_devices = frozenset(sitl_devices)
        self.tally = MavlinkTally(sitl_devices)
        self._egress: Mapping[str, bytes] = {}
        self._ingress: dict[str, bytes] = {}

    def frames(self, k: int, record: TickRecord, egress: Mapping[str, bytes]) -> None:
        self.tally.feed(record.data, FROM_SITL, dict(egress))
        self._egress = egress

    def inbound(self, k: int, record: TickRecord, owners: Mapping[str, Member]) -> dict[str, bytes]:
        egress, self._egress = self._egress, {}
        for device, data in egress.items():
            if device not in owners:
                record.discard(device, len(data))
        return dict(egress)

    def outbound(self, k: int, member: Member, device: str, data: bytes, record: TickRecord, *,
                 draining: bool) -> None:
        if draining:
            record.drain_egress(member.ordinal, device, data)
        if device in self._sitl_devices:  # in drain: tallied, never delivered
            self._ingress[device] = self._ingress.get(device, b"") + data

    def reply(self, k: int, record: TickRecord, frames: Sequence[Frame]) -> dict[str, bytes]:
        ingress, self._ingress = self._ingress, {}
        self.tally.feed(record.data, TO_SITL, ingress)
        return ingress

    def drained(self, record: TickRecord) -> None:
        ingress, self._ingress = self._ingress, {}
        self.tally.feed(record.data, DRAIN_TO_SITL, ingress)

    def park(self, parked: dict, egress: Mapping[str, bytes]) -> None:
        self.tally.feed(parked, FROM_SITL, dict(egress))

    def closing(self) -> dict[str, object]:
        return {"mavlink_counts": self.tally.totals()}


def make_ports(manifest: Manifest, roster: Roster) -> TickPorts:
    if manifest.router is None:
        return DirectPorts(manifest.sitl_devices)
    return RoutedPorts(manifest, roster)


__all__ = ["DirectPorts", "TickPorts", "make_ports"]
