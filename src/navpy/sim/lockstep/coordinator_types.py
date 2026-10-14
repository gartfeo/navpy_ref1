"""Inputs, budgets, faults and the operator slot of the lockstep coordinator.

Sim budgets are run inputs (``lockstep_slice.md``, "Run phases"); wall
budgets only supervise the host: missing one is a host fault, never a
different outcome.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Protocol

from navpy.sim.lockstep.baton import LockstepFault
from navpy.sim.lockstep.part_wire import Event, Lifecycle, RequestId, Response

#: The backend's router endpoint device (``lockstep_fleet.md``, Links and names).
ROUTER_DEVICE = "lockstep:router"


def serial0_device(sys_id: int) -> str:
    """SITL ``sys_id``'s serial0, NavPy ``sys_id``'s companion link."""
    return f"lockstep:serial0.{sys_id}"


def serial1_device(sys_id: int) -> str:
    """SITL ``sys_id``'s serial1, the coordinator's router endpoint ``sitl<i>``."""
    return f"sitl{sys_id}.serial1"


def fleet_sitl_ports(fleet_size: int) -> tuple[tuple[str, str], ...]:
    """:attr:`Manifest.sitl_ports` of SITLs 1..N under the naming scheme."""
    return tuple((serial0_device(i), serial1_device(i)) for i in range(1, fleet_size + 1))


#: The mission terminal marker: the first tick whose log lines contain it is k_T.
TERMINAL_MARKER = "SNAP(VISION-NAV"
#: Finalization budget after k_T, sim microseconds (until the operator returns).
FINALIZATION_BUDGET_US = 30_000_000
#: Drain budget after k_F, sim microseconds (until every participant ended).
DRAIN_BUDGET_US = 10_000_000
#: A participant stop's budget from its request, sim microseconds.
STOP_BUDGET_US = 10_000_000


class RunFault(LockstepFault):
    """A fault that ends the run.

    Only the class, ``kind`` (a stable code) and the logical context (tick,
    ordinal) are journaled; the message is host detail for the ``*_host``
    sidecar, since it may carry paths, ports or OS errors.
    """

    def __init__(self, kind: str, detail: str, *, tick: int | None = None,
                 ordinal: int | None = None, context: Mapping[str, object] | None = None) -> None:
        super().__init__(detail)
        self.kind = kind
        self.tick = tick
        self.ordinal = ordinal
        #: Logical facts the fault's gate needs (the fleet clock's ``source_us``
        #: values, the exchange budget's sub-wait); journaled only when given.
        self.context = dict(context or {})

    def journal_record(self, tick: int | None) -> dict[str, object]:
        record: dict[str, object] = {
            "fault": type(self).__name__, "kind": self.kind,
            "tick": self.tick if self.tick is not None else tick, "ordinal": self.ordinal}
        if self.context:
            record["context"] = self.context
        return record


class CoordinatorFault(RunFault):
    """The tick contract broke (relay, frame, protocol, EOF without an end)."""


class HostFault(RunFault):
    """A host-side failure: budgets, registration, exit codes, ownership."""


class ParticipantFault(RunFault):
    """A participant reported a fault in its ``DONE``."""


@dataclass(frozen=True)
class RoleSpec:
    """How a role's participant process is built."""

    #: The hooks factory (``module:attribute``) its environment record names.
    hooks: str
    #: The logical devices it owns: a SITL serial0 or the router device. Its
    #: hooks take the same devices from ``config`` (devices as data).
    devices: tuple[str, ...]
    config: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class Manifest:
    seed: int
    epoch_us: int
    #: Δ_max: the largest ``source_us`` step one frame may take (grid check).
    max_frame_gap_us: int
    #: Δ_drain: the synthetic drain tick period (the scheduler period).
    drain_period_us: int
    #: The scenario's mission sim-time budget, from the boot barrier.
    mission_budget_us: int
    roles: Mapping[str, RoleSpec]
    #: Each SITL's serial-port devices by port index (serial0, serial1), in
    #: SITL order (devices as data). Its length is the fleet size N; the slice is one.
    sitl_ports: tuple[tuple[str, ...], ...]
    #: The backend's router endpoint device (:data:`ROUTER_DEVICE`). Every
    #: SITL's serial1 is a router endpoint ``sitl<i>`` the coordinator owns,
    #: and SITL ingress goes through its queues (``lockstep_fleet.md``,
    #: Router and delivery).
    router: str
    #: Participants the coordinator spawns itself, admitted at the barrier:
    #: ``(role, ordinal)`` in spawn order.
    initial: tuple[tuple[str, int], ...] = ()
    #: The NavPy roles: only their log lines carry the terminal marker, and
    #: each must retire through an acked participant stop before drain (the
    #: operator stops companions in finalization).
    companion_roles: tuple[str, ...] = ()
    #: The expected fleet: the companion ordinals (NavPy i is sysid i) whose
    #: first terminal marker each must be latched before the mission ends
    #: ``ok``; taken from the scenario, never from the live set. k_T is the
    #: maximum of their first-SNAP ticks. Empty: the mission never completes
    #: (it ends as a timeout or ``operator_returned``).
    expected_fleet: tuple[int, ...] = ()
    #: The boot barrier's sim budget from frame 1: every SITL must have
    #: latched ``helpers_quiet`` within it, else ``CoordinatorFault("boot_barrier")``.
    #: None is no budget (the slice). Step 10d sets it from the recorded
    #: latest latch tick of a 3-SITL boot plus a documented margin.
    boot_budget_us: int | None = None

    def __post_init__(self) -> None:
        devices = self.sitl_devices
        if not self.sitl_ports or len(set(devices)) != len(devices):
            raise ValueError(f"the SITL ports must name distinct devices: {self.sitl_ports}")
        self._check_owners()

    def _check_owners(self) -> None:
        """A participant owns a SITL serial0 or the router endpoint, never a serial1."""
        if self.router in self.sitl_devices:
            raise ValueError(f"the router device {self.router!r} is a SITL port")
        ownable = {ports[0] for ports in self.sitl_ports} | {self.router}
        for role, spec in self.roles.items():
            foreign = sorted(set(spec.devices) - ownable)
            if foreign:
                raise ValueError(f"role {role!r} owns {foreign}: a participant owns a SITL "
                                 "serial0 or the router device")

    @property
    def sitl_devices(self) -> tuple[str, ...]:
        """Every SITL port's device, SITL by SITL, port by port."""
        return tuple(device for ports in self.sitl_ports for device in ports)


def backend_ordinal(fleet_size: int) -> int:
    """The backend's participant ordinal: after NavPy 1..N (the operator is 0)."""
    return fleet_size + 1


@dataclass(frozen=True)
class WallBudgets:
    """Host supervision only (seconds). Inside a tick each draws on the tick's
    exchange deadline as well (``coordinator_deadline``)."""

    register_s: float = 60.0
    done_s: float = 90.0
    exit_s: float = 30.0


@dataclass(frozen=True)
class OperatorRequest:
    target: int
    id: RequestId
    payload: object


@dataclass(frozen=True)
class OperatorCancel:
    target: int
    id: RequestId


@dataclass(frozen=True)
class OperatorInbox:
    """What tick k delivered to the operator's ports, in delivery order."""

    k: int
    t_us: int
    responses: tuple[tuple[int, Response], ...] = ()
    events: tuple[tuple[int, Event], ...] = ()
    log_lines: tuple[tuple[int, str], ...] = ()
    lifecycle: tuple[Lifecycle, ...] = ()
    #: The tick's absolute wall deadline (``time.monotonic()``) that every
    #: baton handoff and timer round draws on: the exchange deadline; None,
    #: each handoff keeps the baton's own budget.
    wall_deadline: float | None = None


class OperatorHost(Protocol):
    """Tick step 5: run the operator's baton to quiescence at ``t_k``.

    Returns the requests and cancels it made; they travel in ``TICK k+1``.
    ``done`` is true once the operator's run has returned.
    """

    done: bool

    def run_tick(self, inbox: OperatorInbox) -> tuple[OperatorRequest | OperatorCancel, ...]: ...

    def deliver_drain(self, inbox: OperatorInbox) -> None:
        """A drain tick's responses and events, after the run returned: the
        ports still account for them (late drops); nothing more runs."""

    def observations(self) -> dict[str, object]:
        """What the operator's ports saw in the tick just run (event receipts,
        dropped deliveries); empty if nothing. The run journal records it."""


__all__ = [
    "CoordinatorFault", "DRAIN_BUDGET_US", "RunFault", "FINALIZATION_BUDGET_US", "HostFault", "Manifest",
    "OperatorCancel", "OperatorHost", "OperatorInbox", "OperatorRequest", "ParticipantFault",
    "ROUTER_DEVICE", "RoleSpec", "STOP_BUDGET_US", "TERMINAL_MARKER", "WallBudgets",
    "backend_ordinal", "fleet_sitl_ports", "serial0_device", "serial1_device",
]
