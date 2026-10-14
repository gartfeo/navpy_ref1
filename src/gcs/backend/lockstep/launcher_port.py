"""The companion launcher port: NavPy companions as lockstep participants.

It has the companion manager's surface (``NavpyProcessManager``: start,
stop, stop-all, status, ``wait_ready``) and is the manager the backend
builds under lockstep (``navpy_sim_runtime.set_manager_factory``). A start
puts a spawn request in this tick's ``DONE``; a stop a stop request. The
answers are the coordinator's lifecycle records, carrying the ordinal and
generation, never a PID (``lockstep_slice.md``, "Lifecycle"):

- ``spawn`` (ok or error) answers the start;
- ``ready`` is the companion's ``runtime_ready`` payload (``wait_ready``);
- ``exited`` retires it: ``running`` until then, and ``exit_code`` 0 after a
  flushed end (``failed`` gives 1);
- ``stop_result``: ``acked`` is today's ``STOP_ACKED``; ``missed`` a failed
  stop; an error (a stale or ended generation) is not a stop.

A companion's ``pid`` is a surrogate naming its ordinal and generation
(:func:`generation_pid`), so a replacement never reuses its predecessor's
and a stale identifier never stops a replacement. Records arrive on the coordinator
thread in the tick's input phase (:meth:`deliver`), so they only set events.
Role config: ``companions``, ``{sys_id: {"role", "ordinal", "device"}}``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any

from gcs.backend.navpy_process_stop import STOP_ACKED, STOP_ALREADY_EXITED, STOP_FAILED
from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.part_wire import Lifecycle, RequestId, SpawnRequest, StopRequest
from navpy.sim.lockstep.participant_hooks import OutboxBuffer, RequestIds

_STOP_OUTCOMES = {"acked": STOP_ACKED, "missed": STOP_FAILED}
#: Generations per ordinal in a pid surrogate: ``ordinal * span + generation``.
GENERATION_SPAN = 1_000_000


def generation_pid(ordinal: int, generation: int) -> int:
    """The route-facing pid of one companion generation (deterministic, never a host pid)."""
    if not 0 < generation < GENERATION_SPAN:
        raise ValueError(f"generation {generation} outside the pid surrogate's span")
    return ordinal * GENERATION_SPAN + generation


@dataclass
class _Companion:
    sys_id: int
    connection: str
    ordinal: int
    spawn_id: RequestId
    started_at: float
    spawned: Any
    ready_or_exit: Any
    generation: int | None = None
    spawn_error: str | None = None
    ready_payload: dict | None = None
    failed: bool | None = None
    stops: dict[RequestId, Any] = field(default_factory=dict)
    stop_results: dict[RequestId, str] = field(default_factory=dict)

    @property
    def running(self) -> bool:
        return self.generation is not None and self.failed is None

    @property
    def exit_code(self) -> int | None:
        return None if self.failed is None else int(self.failed)

    @property
    def pid(self) -> int | None:
        """This generation's surrogate pid; None until its spawn is answered."""
        return None if self.generation is None else generation_pid(self.ordinal, self.generation)

    @property
    def process(self) -> SimpleNamespace:
        """The route-facing process facts."""
        return SimpleNamespace(pid=self.pid)


class LockstepLauncher:
    def __init__(self, companions: dict[int, dict[str, Any]], out: OutboxBuffer,
                 ids: RequestIds) -> None:
        self._roles = {int(sys_id): dict(spec) for sys_id, spec in companions.items()}
        self._out = out
        self._ids = ids
        self._instances: dict[int, _Companion] = {}
        self._by_spawn: dict[RequestId, _Companion] = {}

    # -- coordinator side ---------------------------------------------------------
    def deliver(self, entry: Lifecycle) -> None:
        if entry.kind == "stop_result":
            for companion in self._by_spawn.values():
                if entry.request in companion.stops:
                    companion.stop_results[entry.request] = entry.result
                    companion.stops[entry.request].deliver_set()
            return
        companion = self._by_spawn.get(entry.request)
        if companion is None:
            raise RuntimeError(f"lifecycle {entry.kind} for an unknown spawn {entry.request}")
        if entry.kind == "spawn":
            if entry.result == "ok":
                companion.generation = entry.generation
            else:
                companion.spawn_error = str(entry.payload)
            companion.spawned.deliver_set()
        elif entry.kind == "ready":
            companion.ready_payload = dict(entry.payload)
            companion.ready_or_exit.deliver_set()
        elif entry.kind == "exited":
            companion.failed = bool(entry.payload["failed"])
            companion.ready_or_exit.deliver_set()

    # -- the manager surface (executor jobs, on the baton) ---------------------------
    def start(self, sys_id: int, connection: str, **_options: Any) -> _Companion:
        spec = self._roles.get(sys_id)
        if spec is None:
            raise ValueError(f"no lockstep companion role for sys_id {sys_id}")
        existing = self._instances.get(sys_id)
        if existing is not None and (existing.running or existing.generation is None):
            raise ValueError(f"NavPy instance for sys_id {sys_id} already running")
        spawn_id = self._ids.next()
        # The companion's logical device, never the settings' host transport.
        companion = _Companion(sys_id, spec.get("device", connection), int(spec["ordinal"]),
                               spawn_id, rt.time(), rt.Event(), rt.Event())
        self._instances[sys_id] = self._by_spawn[spawn_id] = companion
        self._out.spawn_requests.append(SpawnRequest(spawn_id, spec["role"], companion.ordinal))
        companion.spawned.wait()
        if companion.spawn_error is not None:
            del self._instances[sys_id]
            raise ValueError(f"NavPy spawn for sys_id {sys_id} refused: {companion.spawn_error}")
        return companion

    def stop(self, sys_id: int) -> bool:
        companion = self._instances.get(sys_id)
        if companion is None:
            return False
        self._stop(companion)
        return True

    def stop_generation(self, sys_id: int, pid: int) -> bool:
        companion = self._instances.get(sys_id)
        if companion is None or companion.pid != pid:
            return False  # a stale or foreign identifier never stops this generation
        self._stop(companion)
        return True

    def stop_all(self) -> dict[int, str]:
        return {sys_id: self._stop(companion)
                for sys_id, companion in sorted(self._instances.items())}

    def _stop(self, companion: _Companion) -> str:
        if not companion.running:
            outcome = STOP_ALREADY_EXITED
        else:
            stop_id = self._ids.next()
            companion.stops[stop_id] = rt.Event()
            self._out.stop_requests.append(StopRequest(stop_id, companion.ordinal, companion.generation))
            companion.stops[stop_id].wait()
            outcome = _STOP_OUTCOMES.get(companion.stop_results[stop_id], STOP_FAILED)
        if self._instances.get(companion.sys_id) is companion:
            del self._instances[companion.sys_id]
        return outcome

    def get_status(self, sys_id: int) -> dict[str, object] | None:
        companion = self._instances.get(sys_id)
        return None if companion is None else self._snapshot(companion)

    def get_all_status(self) -> list[dict[str, object]]:
        return [self._snapshot(companion) for companion in self._instances.values()]

    def wait_ready(self, sys_id: int, pid: int, timeout_s: float) -> dict[str, object]:
        companion = self._instances.get(sys_id)
        if companion is None or companion.pid != pid:
            raise RuntimeError(f"NavPy instance sys_id={sys_id} pid={pid} is not tracked")
        if not companion.ready_or_exit.wait(max(float(timeout_s), 0.0)):
            raise TimeoutError(f"NavPy instance sys_id={sys_id} pid={pid} did not become ready")
        if companion.ready_payload is None:
            raise RuntimeError(f"NavPy instance sys_id={sys_id} pid={pid} exited before "
                               f"readiness (exit_code={companion.exit_code})")
        if not companion.running:
            raise RuntimeError(f"NavPy instance sys_id={sys_id} pid={pid} exited after "
                               f"readiness (exit_code={companion.exit_code})")
        return dict(companion.ready_payload)

    @staticmethod
    def _snapshot(companion: _Companion) -> dict[str, object]:
        running = companion.running
        return {
            "sys_id": companion.sys_id, "connection": companion.connection,
            "pid": companion.pid, "ordinal": companion.ordinal,
            "generation": companion.generation, "running": running,
            "exit_code": companion.exit_code,
            "uptime_s": round(rt.time() - companion.started_at, 1) if running else 0,
            "ready": running and companion.ready_payload is not None,
            "runtime": None if companion.ready_payload is None else dict(companion.ready_payload),
            "startup_error": companion.spawn_error, "log": [],
        }


__all__ = ["GENERATION_SPAN", "LockstepLauncher", "generation_pid"]
