"""Capability spawn: create a participant, record it, then let it run.

Order (``lockstep_slice.md``, "Processes and ownership"):

1. The spawner creates the child; it waits for ``go`` and runs nothing.
2. The recorder records it under the registry lock, only while the slot's
   entry is ``active``. A refusal (``stopping``) kills the child before ``go``.
3. ``go``. A child that never got it exits on stdin EOF.

The capability is derived from the run seed and the spawn id, so it is
deterministic, and is passed only in that child's environment record.
``REGISTER`` must match the recorded spawn (capability, spawn id, generation,
role, ordinal), else a host fault. PIDs, start times, ports and exit codes go
only to the ``*_host`` sidecar.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Protocol

from navpy.sim.lockstep.coordinator_types import HostFault
from navpy.sim.lockstep.part_wire import Register, RequestId
from navpy.sim.lockstep.participant_main import HOST_DELAY_VARIABLE

#: Domain tag of the capability derivation; a new derivation gets a new tag.
CAPABILITY_DOMAIN = "navpy.lockstep.capability.v1"
PARTICIPANT_MODULE = "navpy.sim.lockstep.participant_main"
PROCESSES_HOST_FILE = "lockstep_participants_host.json"
#: A participant's hash seed: fixed, so set and dict-of-str order cannot vary.
PYTHON_HASH_SEED = "0"
#: Host variables the interpreter needs to start (Windows sockets need SYSTEMROOT).
HOST_START_VARIABLES = ("PATH", "SYSTEMROOT") if os.name == "nt" else ("PATH",)


def capability(seed: int, spawn_id: RequestId) -> str:
    material = f"{CAPABILITY_DOMAIN}:{int(seed)}:{spawn_id[0]}:{spawn_id[1]}".encode("ascii")
    return hashlib.sha256(material).hexdigest()[:32]


@dataclass(frozen=True)
class SpawnRecord:
    """A recorded spawn: deterministic, journaled."""

    spawn_id: RequestId
    role: str
    ordinal: int
    generation: int
    capability: str
    #: The ordinal that asked for it (0: the operator side, the coordinator).
    requester: int


def validate_register(register: Register, record: SpawnRecord) -> None:
    expected = Register(record.capability, record.role, record.ordinal, record.spawn_id,
                        record.generation)
    if register != expected:
        raise HostFault(
            "register_mismatch",
            f"REGISTER does not match spawn {record.spawn_id}: role {register.role!r}, "
            f"ordinal {register.ordinal}, spawn id {register.spawn_id}, generation "
            f"{register.generation}, capability {'ok' if register.capability == record.capability else 'wrong'}",
            ordinal=record.ordinal,
        )


class HostChild(Protocol):
    pid: int

    def go(self) -> None: ...

    def kill(self) -> None: ...

    def wait(self, timeout_s: float) -> int | None: ...


class Spawner(Protocol):
    def spawn(self, record: SpawnRecord, environment_record: str, variable: str) -> HostChild:
        """Create the child held before ``go``, its environment record in ``variable``."""


class ChildRecorder(Protocol):
    def record(self, child: HostChild, spawn: SpawnRecord) -> bool:
        """Record under the registry lock; False if the entry is not ``active``."""

    def forget(self, child: HostChild, spawn: SpawnRecord) -> None:
        """Release the record of a child killed before admission."""


class LocalChild:
    """A participant process on this host, held until :meth:`go`."""

    def __init__(self, process: subprocess.Popen, log: IO[bytes]) -> None:
        self._process = process
        self._log = log
        self.pid = process.pid

    def go(self) -> None:
        assert self._process.stdin is not None
        self._process.stdin.write(b"go\n")
        self._process.stdin.close()

    def kill(self) -> None:
        try:
            self._process.kill()
            self._process.wait()
        finally:
            self._close()  # the pipes and log close even if the kill failed

    def wait(self, timeout_s: float) -> int | None:
        try:
            code = self._process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return None
        self._close()
        return code

    def _close(self) -> None:
        if self._process.stdin and not self._process.stdin.closed:
            self._process.stdin.close()
        self._log.close()


def hermetic_environment(python_path: tuple[str, ...], variable: str, record: str) -> dict[str, str]:
    """A child's whole environment: run inputs only, nothing ambient.

    The environment record (the run inputs), the source roots, a fixed hash
    seed, and what the interpreter needs to start on this host (``PATH``, and
    ``SYSTEMROOT`` on Windows). No ambient NavPy, pose-recording or logging
    setting reaches a participant.
    """
    environment = {
        variable: record,
        "PYTHONPATH": os.pathsep.join(python_path),
        "PYTHONHASHSEED": PYTHON_HASH_SEED,
    }
    for name in HOST_START_VARIABLES:
        if name in os.environ:
            environment[name] = os.environ[name]
    return environment


class LocalSpawner:
    """Spawns ``participant_main`` children; their output goes to host logs.

    ``host_delay_seed`` (acceptance (c)) makes each child sleep a seeded host
    delay before every ``DONE``; it is a host perturbation, passed only in
    ``HOST_DELAY_VARIABLE``, never in the environment record.
    """

    def __init__(self, log_dir: Path, python_path: tuple[str, ...], *,
                 host_delay_seed: int = 0) -> None:
        self._log_dir = log_dir
        self._python_path = tuple(python_path)
        self._host_delay_seed = host_delay_seed

    def spawn(self, record: SpawnRecord, environment_record: str, variable: str) -> LocalChild:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        name = f"participant_{record.ordinal}_g{record.generation}_host.log"
        log = (self._log_dir / name).open("ab")
        environment = hermetic_environment(self._python_path, variable, environment_record)
        if self._host_delay_seed:
            environment[HOST_DELAY_VARIABLE] = str(self._host_delay_seed * 1000 + record.ordinal)
        process = subprocess.Popen(
            [sys.executable, "-m", PARTICIPANT_MODULE], env=environment, cwd=self._log_dir,
            stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
        )
        return LocalChild(process, log)


class HostSidecar:
    """``<run>/lockstep_participants_host.json``: host facts per spawn, atomically."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._rows: dict[str, dict[str, object]] = {}

    def note(self, spawn: SpawnRecord, **facts: object) -> None:
        key = f"{spawn.spawn_id[0]}.{spawn.spawn_id[1]}"
        row = self._rows.setdefault(key, {
            "spawn_id": list(spawn.spawn_id), "role": spawn.role, "ordinal": spawn.ordinal,
            "generation": spawn.generation,
        })
        row.update(facts)
        temporary = self._path.with_suffix(".tmp")
        temporary.write_text(json.dumps(list(self._rows.values()), indent=2), encoding="utf-8")
        temporary.replace(self._path)

    @property
    def rows(self) -> list[dict[str, object]]:
        return [dict(row) for row in self._rows.values()]


__all__ = [
    "CAPABILITY_DOMAIN", "ChildRecorder", "HostChild", "HostSidecar", "LocalChild",
    "LocalSpawner", "PROCESSES_HOST_FILE", "SpawnRecord", "Spawner", "capability",
    "hermetic_environment",
    "validate_register",
]
