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

SERIAL_DEVICES = ("lockstep:serial0", "lockstep:serial1")
"""The SITL's serial ports by index: NavPy's companion link and the backend monitor."""
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
                 ordinal: int | None = None) -> None:
        super().__init__(detail)
        self.kind = kind
        self.tick = tick
        self.ordinal = ordinal

    def journal_record(self, tick: int | None) -> dict[str, object]:
        return {"fault": type(self).__name__, "kind": self.kind,
                "tick": self.tick if self.tick is not None else tick, "ordinal": self.ordinal}


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
    #: The logical devices it owns (``SERIAL_DEVICES`` entries).
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
    #: Participants the coordinator spawns itself, admitted at the barrier:
    #: ``(role, ordinal)`` in spawn order.
    initial: tuple[tuple[str, int], ...] = ()
    #: The NavPy roles: only their log lines carry the terminal marker, and
    #: each must retire through an acked participant stop before drain (the
    #: operator stops companions in finalization).
    companion_roles: tuple[str, ...] = ()


@dataclass(frozen=True)
class WallBudgets:
    """Host supervision only (seconds)."""

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
    "RoleSpec", "SERIAL_DEVICES", "STOP_BUDGET_US", "TERMINAL_MARKER", "WallBudgets",
]
