"""The coordinator's participants: spawn, registration, exchange and exit.

A :class:`Member` is one spawned participant generation. It is admitted at
its first ``TICK`` and retired at the tick of its ending ``DONE``; between
the two it gets exactly one ``TICK`` per tick. Retirement is logical: the
OS exit is awaited behind the clock hold and only checked, never journaled.
"""

from __future__ import annotations

import logging
import socket
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.sim.lockstep.coordinator_register import await_register
from navpy.sim.lockstep.coordinator_spawn import (
    ChildRecorder,
    HostChild,
    HostSidecar,
    SpawnRecord,
    Spawner,
    capability,
)
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault, Manifest, WallBudgets
from navpy.sim.lockstep.part_socket import PartListener, PartSocket, PeerClosed
from navpy.sim.lockstep.part_wire import (
    Done,
    Lifecycle,
    PartWireError,
    Request,
    RequestId,
    Tick,
    check_tick,
)
from navpy.sim.lockstep.participant_env import ENVIRONMENT_VARIABLE, EnvironmentRecord

_log = logging.getLogger(__name__)


@dataclass
class PendingInputs:
    """A participant's inputs for its next ``TICK``."""

    requests: list[Request] = field(default_factory=list)
    lifecycle: list[Lifecycle] = field(default_factory=list)
    cancels: list[RequestId] = field(default_factory=list)

    def take(self) -> tuple[tuple[Request, ...], tuple[Lifecycle, ...], tuple[RequestId, ...]]:
        taken = (tuple(sorted(self.requests, key=lambda r: r.id)), tuple(self.lifecycle),
                 tuple(sorted(self.cancels)))
        self.requests.clear()
        self.lifecycle.clear()
        self.cancels.clear()
        return taken


@dataclass
class Member:
    spawn: SpawnRecord
    child: HostChild
    link: PartSocket
    devices: tuple[str, ...]
    admitted_at: int | None = None
    ready: bool = False
    retired_at: int | None = None
    flushed: bool | None = None
    end_sent: bool = False
    exited: bool = False
    inputs: PendingInputs = field(default_factory=PendingInputs)

    @property
    def ordinal(self) -> int:
        return self.spawn.ordinal

    @property
    def live(self) -> bool:
        return self.retired_at is None

    def take_tick(self, k: int, t_us: int, ports: tuple[tuple[str, bytes], ...], end: bool) -> Tick:
        requests, lifecycle, cancels = self.inputs.take()
        tick = Tick(k, t_us, ports, requests, lifecycle, cancels, end and not self.end_sent)
        self.end_sent = self.end_sent or tick.end
        return tick


class Roster:
    def __init__(
        self,
        manifest: Manifest,
        spawner: Spawner,
        recorder: ChildRecorder,
        sidecar: HostSidecar,
        budgets: WallBudgets,
    ) -> None:
        self._manifest = manifest
        self._spawner = spawner
        self._recorder = recorder
        self._sidecar = sidecar
        self._budgets = budgets
        self._listener = PartListener()
        self._generations: dict[int, int] = {}
        self.members: list[Member] = []
        #: Children killed before admission whose exit is unconfirmed, with their
        #: record if one was made: stop_all retries them.
        self.unadmitted: list[tuple[HostChild, SpawnRecord | None]] = []

    def generation(self, ordinal: int) -> int | None:
        """The ordinal's latest generation, if it was ever spawned."""
        return self._generations.get(ordinal)

    def current(self, ordinal: int) -> Member | None:
        found = [m for m in self.members if m.ordinal == ordinal]
        return found[-1] if found else None

    def active(self) -> list[Member]:
        """Admitted, not yet retired: ``TICK`` order (ordinal order)."""
        return sorted((m for m in self.members if m.admitted_at is not None and m.live),
                      key=lambda m: m.ordinal)

    def spawn(self, role: str, ordinal: int, spawn_id: RequestId, requester: int) -> Member:
        """Spawn, record, ``go``, then hold until it registers (wall budget).

        The child is owned from creation: any failure before admission kills
        it and releases its record, if one was made.
        """
        spec = self._manifest.roles.get(role)
        if spec is None:
            raise HostFault("unknown_role", f"no role {role!r} in the manifest", ordinal=ordinal)
        live = self.current(ordinal)
        if live is not None and live.live:
            raise HostFault("ordinal_live", f"ordinal {ordinal} is already live", ordinal=ordinal)
        generation = self._generations.get(ordinal, 0) + 1
        self._generations[ordinal] = generation
        record = SpawnRecord(spawn_id, role, ordinal, generation,
                             capability(self._manifest.seed, spawn_id), requester)
        environment = EnvironmentRecord(
            self._listener.address, record.capability, role, ordinal, spawn_id, generation,
            self._manifest.seed, self._manifest.epoch_us, spec.hooks, dict(spec.config),
        )
        child = self._spawner.spawn(record, environment.to_json(), ENVIRONMENT_VARIABLE)
        recorded = False
        try:
            recorded = self._recorder.record(child, record)
            if not recorded:
                raise HostFault("slot_stopping", f"spawn {spawn_id} refused before go",
                                ordinal=ordinal)
            child.go()
            self._sidecar.note(record, pid=child.pid, coordinator_port=self._listener.address[1])
            link = await_register(self._listener, record, child, self._budgets.register_s)
        except BaseException:
            # The record is released only once the child is confirmed gone;
            # otherwise it stays recorded (gcs_stop recovers it) and stop_all
            # retries it. The original error stands.
            failures = _attempt(child.kill)
            if not failures and child.wait(self._budgets.exit_s) is not None:
                if recorded:
                    failures += _attempt(lambda: self._recorder.forget(child, record))
            else:
                self.unadmitted.append((child, record if recorded else None))
                failures.append("not confirmed gone; its record is kept")
            for failure in failures:
                _log.error("unwinding spawn %s: %s", spawn_id, failure)
            raise
        member = Member(record, child, link, spec.devices)
        self.members.append(member)
        return member

    def send(self, member: Member, tick: Tick) -> bytes:
        try:
            return member.link.send(tick, time.monotonic() + self._budgets.done_s)
        except OSError as error:
            raise CoordinatorFault("tick_unsent", f"TICK not sent: {error}", tick=tick.k,
                                   ordinal=member.ordinal) from error

    def receive(self, member: Member, k: int) -> tuple[Done, bytes]:
        context = {"tick": k, "ordinal": member.ordinal}
        try:
            message, raw = member.link.recv(time.monotonic() + self._budgets.done_s)
        except (PeerClosed, ConnectionResetError, ConnectionAbortedError) as error:
            raise CoordinatorFault(  # EOF or a reset: the process went away
                "eof_without_end", f"EOF at tick {k} without an ending DONE", **context) from error
        except socket.timeout as error:
            raise HostFault("done_missed", f"no DONE {k} within the wall budget", **context) from error
        except (OSError, PartWireError) as error:
            raise CoordinatorFault("done_invalid", f"DONE {k}: {error}", **context) from error
        if not isinstance(message, Done):
            raise CoordinatorFault("done_invalid", f"expected DONE, got {type(message).__name__}",
                                   **context)
        try:
            check_tick(message.k, k)
        except PartWireError as error:
            raise CoordinatorFault("done_tick", str(error), **context) from error
        own = [r.id for r in (*message.spawn_requests, *message.stop_requests)]
        if any(origin != member.ordinal for origin, _ in own):
            raise CoordinatorFault("foreign_id", f"request ids of another origin {own}", **context)
        return message, raw

    def await_exit(self, member: Member) -> None:
        """The clock hold after an ending ``DONE``: the process must exit cleanly."""
        member.link.close()
        code = member.child.wait(self._budgets.exit_s)
        member.exited = code is not None
        self._sidecar.note(member.spawn, exit_code=code)
        if code is None:
            raise HostFault("exit_missed", "did not exit within the wall budget",
                            tick=member.retired_at, ordinal=member.ordinal)
        if member.flushed and code != 0:
            raise HostFault("exit_nonzero", f"exited nonzero ({code}) after a flushed end",
                            tick=member.retired_at, ordinal=member.ordinal)

    def stop_all(self) -> None:
        """Stop by ownership: every spawned child that has not exited, then the listener.

        Every step is tried whatever the others do; the failures are raised
        together at the end.
        """
        failures = []
        for member in self.members:
            failures += _attempt(member.link.close)
            failures += _attempt(lambda m=member: self._kill(m))
        for pending in list(self.unadmitted):
            failures += _attempt(lambda p=pending: self._kill_unadmitted(*p))
        failures += _attempt(self._listener.close)
        if failures:
            raise HostFault("teardown_failed", "; ".join(failures))

    def _kill(self, member: Member) -> None:
        if not member.exited:
            member.child.kill()
            if member.child.wait(self._budgets.exit_s) is None:
                raise HostFault("teardown_failed", "killed but not confirmed gone",
                                ordinal=member.ordinal)
            member.exited = True

    def _kill_unadmitted(self, child: HostChild, record: SpawnRecord | None) -> None:
        child.kill()
        if child.wait(self._budgets.exit_s) is None:
            raise HostFault("teardown_failed", "an unadmitted child is not confirmed gone")
        self.unadmitted.remove((child, record))
        if record is not None:  # confirmed gone: now its record may go
            self._recorder.forget(child, record)

    def close(self) -> None:
        self._listener.close()


def _attempt(step: Callable[[], object]) -> list[str]:
    """Run one teardown step; its failure, if any, as text."""
    try:
        step()
    except Exception as error:
        return [f"{type(error).__name__}: {error}"]
    return []


__all__ = ["Member", "PendingInputs", "Roster"]
