"""Test doubles for the router path's ports: a scripted tick harness, MAVLink sources.

:class:`PortsHarness` drives :class:`~navpy.sim.lockstep.coordinator_router.RoutedPorts`
through tick steps 1-4 as the coordinator does, without processes: each
:meth:`PortsHarness.tick` takes the SITLs' egress, which members are
admitted (their ``TICK k`` is noted in the record, as the exchange does),
the members' ``DONE`` egress and each port's FIFO fill, and journals the
tick record. NavPy i is ordinal i on ``lockstep:serial0.<i>``; the backend
is ordinal 4 on ``lockstep:router``.
"""

from __future__ import annotations

from dataclasses import dataclass

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_router import RoutedPorts
from navpy.sim.lockstep.coordinator_types import Manifest, RoleSpec
from navpy.sim.lockstep.mavlink_wire import encoder
from navpy.sim.lockstep.packet_journal import PacketJournal
from navpy.sim.lockstep.step_wire import Frame, PortState

SITLS = (1, 2, 3)
BACKEND = 4
ROUTER = "lockstep:router"


def serial0(sitl: int) -> str:
    return f"lockstep:serial0.{sitl}"


def serial1(sitl: int) -> str:
    return f"sitl{sitl}.serial1"


def routed_manifest() -> Manifest:
    roles = {"backend": RoleSpec("toy:backend", (ROUTER,)),
             **{f"navpy{i}": RoleSpec("toy:navpy", (serial0(i),)) for i in SITLS}}
    return Manifest(seed=1, epoch_us=0, max_frame_gap_us=20_000, drain_period_us=20_000,
                    mission_budget_us=10**9, roles=roles,
                    sitl_ports=tuple((serial0(i), serial1(i)) for i in SITLS), router=ROUTER)


@dataclass
class Spawn:
    ordinal: int
    generation: int = 1


@dataclass
class FakeMember:
    spawn: Spawn
    devices: tuple[str, ...]
    retired_at: int | None = None

    @property
    def ordinal(self) -> int:
        return self.spawn.ordinal

    @property
    def live(self) -> bool:
        return self.retired_at is None


class FakeRoster:
    def __init__(self) -> None:
        self.members: list[FakeMember] = []

    def add(self, ordinal: int, device: str, generation: int = 1) -> FakeMember:
        member = FakeMember(Spawn(ordinal, generation), (device,))
        self.members.append(member)
        return member


class Sources:
    """One encoder per MAVLink source (sequence numbers as on a real link)."""

    def __init__(self) -> None:
        self._links: dict = {}

    def send(self, source: tuple[int, int], message: str, *args) -> bytes:
        if source not in self._links:
            self._links[source] = encoder(*source)
        link, sink = self._links[source]
        getattr(link, f"{message}_send")(*args)
        return sink.drain()

    def heartbeat(self, system: int, component: int) -> bytes:
        return self.send((system, component), "heartbeat", 1, 3, 0, 0, 0)

    def param(self, source: tuple[int, int], system: int, component: int) -> bytes:
        return self.send(source, "param_request_read", system, component, b"X", -1)

    def assign(self, source: tuple[int, int], target: int, task_id: int = 4) -> bytes:
        return self.send(source, "task_assign_request", 7, 1, 1000, 5000, target, task_id, 1, 2, 1, 2, 3)

    def blob(self, source: tuple[int, int], seq: int) -> bytes:
        """A ``GPS_RTCM_DATA``: no target, 182 payload bytes, passes the SITL filter."""
        return self.send(source, "gps_rtcm_data", seq % 256, 180, bytes([seq % 251 + 1]) * 180)


class PortsHarness:
    def __init__(self) -> None:
        self.roster = FakeRoster()
        self.ports = RoutedPorts(routed_manifest(), self.roster)
        self.records: list[dict] = []
        self.parked: dict | None = None

    def tick(self, k: int, *, egress=None, admitted=(), done=(), fifo=None,
             phase: str = "mission") -> tuple[dict[str, bytes], dict[str, bytes]]:
        """Returns (what each owned device got in ``TICK k``, reply k's ingress per device)."""
        record = TickRecord(k, phase, 1_000_000 + k * 20_000)
        self.ports.frames(k, record, egress or {})
        owners = {device: member for member in admitted for device in member.devices}
        for member in sorted(admitted, key=lambda m: m.ordinal):
            record.member(member.ordinal, member.spawn.generation, b"", b"")
        delivered = self.ports.inbound(k, record, owners)
        for member, device, data in done:
            self.ports.outbound(k, member, device, data, record, draining=False)
        ingress = self.ports.reply(k, record, self.frames(k, fifo or {}))
        self.records.append(record.data)
        return delivered, ingress

    def drain(self, k: int, done) -> dict:
        record = TickRecord(k, "drain", 0)
        for member, device, data in done:
            self.ports.outbound(k, member, device, data, record, draining=True)
        self.ports.drained(record)
        self.records.append(record.data)
        return record.data

    def park(self, k: int, egress=None) -> dict:
        self.parked = {"tick": k}
        self.ports.park(self.parked, egress or {})
        self.records.append({"parked": self.parked})
        return self.parked

    @staticmethod
    def frames(k: int, fifo: dict[str, int]) -> list[Frame]:
        return [Frame((1, i), k, 1_000_000 + k * 20_000, 0, True,
                      (PortState(fifo.get(serial0(i), 0), 0, 0, b""),
                       PortState(fifo.get(serial1(i), 0), 0, 0, b"")))
                for i in SITLS]

    def journal(self) -> PacketJournal:
        return PacketJournal(self.records)

    def events(self, kind: str) -> list[tuple[int, list]]:
        """``(tick, event)`` of every packet event of ``kind``."""
        out = []
        for record in self.records:
            tick = record["parked"]["tick"] if "parked" in record else record["k"]
            events = record["parked"]["packets"] if "parked" in record else record["packets"]
            out.extend((tick, event) for event in events if event[0] == kind)
        return out
