"""The processes one approach run owns (hitl_final_approach.md section 9, stage 1 step 13).

Each role starts at most once per run and only when its ports are free and nothing refuses it (another run's
process): a second owner of a port or a role is refused before anything starts. A start waits for the role's ready
line in its output (or another file it writes) and fails if the process exits first or the line does not come in
time. ``stop_all`` stops every process in reverse start order, each its own way (input closed, Ctrl-C or a stop
line), then terminates and kills what is left; it runs on every exit of the runner. ``orphans`` names any role
whose process group still has a member after that.

POSIX processes run in their own session, so a role's children (WSL's MediaMTX, JSBSim) share its process group
and are stopped and checked with it. Unreal runs on Windows; approach_chain.py gives its handle.
"""
from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import IO, Optional, Protocol

READY_POLL_S = 0.25            # the OS gives no event when a process writes its ready line
TERMINATE_WAIT_S = 10.0        # after the role's own stop: terminate, wait this long, then kill
STOP_EOF = "eof"
STOP_SIGINT = "sigint"


class RunnerError(RuntimeError):
    """A run that cannot start or continue: the message says which role and why."""


class Handle(Protocol):
    pid: int

    def running(self) -> bool: ...
    def exit_code(self) -> Optional[int]: ...
    def request_stop(self) -> None: ...
    def terminate(self) -> None: ...
    def kill(self) -> None: ...
    def group_alive(self) -> bool: ...
    def confirm_ready(self) -> None: ...


@dataclass(frozen=True)
class ProcSpec:
    role: str
    argv: tuple[str, ...]
    ready: Optional[str]                     # regex in the ready file; None: ready once started
    ready_timeout_s: float
    stop: str = STOP_SIGINT                  # STOP_EOF, STOP_SIGINT or the line written to its input
    stop_wait_s: float = 30.0
    ready_file: Optional[Path] = None        # default: its output file
    ports: tuple[tuple[str, int], ...] = ()  # ("tcp" | "udp", port) it binds
    refuse: Optional[Callable[[], Optional[str]]] = None   # why it may not start now (another owner), or None
    cwd: Optional[str] = None
    env: Optional[Mapping[str, str]] = None
    kind: str = "posix"                      # which spawner starts it


Spawner = Callable[[ProcSpec, IO[str]], Handle]


class PosixHandle:
    """A Popen in its own session: the role's stop and the escalation reach its whole process group."""

    def __init__(self, process: subprocess.Popen, stop: str) -> None:
        self.process, self.pid, self._stop = process, process.pid, stop

    def running(self) -> bool:
        return self.process.poll() is None

    def exit_code(self) -> Optional[int]:
        return self.process.poll()

    def request_stop(self) -> None:
        if self._stop == STOP_SIGINT:
            self._signal(signal.SIGINT)
            return
        try:
            if self._stop != STOP_EOF and self.process.stdin is not None:
                self.process.stdin.write(self._stop + "\n")
                self.process.stdin.flush()
            if self.process.stdin is not None:
                self.process.stdin.close()
        except (OSError, ValueError):        # its input already closed: it is on its way out
            pass

    def terminate(self) -> None:
        self._signal(signal.SIGTERM)

    def kill(self) -> None:
        self._signal(signal.SIGKILL)

    def confirm_ready(self) -> None:
        """Nothing more to learn once its ready line came."""

    def group_alive(self) -> bool:
        self.process.poll()                  # reap the leader first: a zombie still counts as a member
        try:
            os.killpg(self.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _signal(self, number: int) -> None:
        try:
            os.killpg(self.pid, number)
        except ProcessLookupError:
            pass


def spawn_posix(spec: ProcSpec, output: IO[str]) -> Handle:
    stdin = subprocess.DEVNULL if spec.stop == STOP_SIGINT else subprocess.PIPE
    env = None if spec.env is None else {**os.environ, **spec.env}
    process = subprocess.Popen(list(spec.argv), stdin=stdin, stdout=output, stderr=subprocess.STDOUT,
                               cwd=spec.cwd, env=env, start_new_session=True, text=True)
    return PosixHandle(process, spec.stop)


@dataclass
class Owned:
    spec: ProcSpec
    handle: Handle
    output: Optional[IO[str]]
    started_utc: str
    stopped: dict = field(default_factory=dict)


def _alive(check: Callable[[], bool], errors: list[str]) -> bool:
    """A group_alive or running check; one that fails counts as alive (recorded): unknown is not gone."""
    try:
        return check()
    except Exception as exc:       # noqa: BLE001 - the stop goes on and the role is reported
        if f"alive check: {type(exc).__name__}: {exc}" not in errors:
            errors.append(f"alive check: {type(exc).__name__}: {exc}")
        return True


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


class ProcessGroup:
    """The run's owned processes; ``spawners`` by ProcSpec.kind, ``port_free(protocol, port)`` checks a port."""

    def __init__(self, run_dir: Path, spawners: Mapping[str, Spawner], port_free: Callable[[str, int], bool],
                 clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep) -> None:
        self.run_dir = Path(run_dir)
        self._spawners = spawners
        self._port_free = port_free
        self._clock, self._sleep = clock, sleep
        self.owned: list[Owned] = []

    def output_path(self, role: str) -> Path:
        return self.run_dir / f"{role}.out"

    def refusals(self, specs: Sequence[ProcSpec]) -> list[str]:
        """Why these roles may not start: a role twice, owned already, a port in use or another owner."""
        problems, roles = [], [owned.spec.role for owned in self.owned]
        for spec in specs:
            if spec.role in roles:
                problems.append(f"{spec.role}: a second {spec.role} in one run")
            roles.append(spec.role)
            problems += [f"{spec.role}: {protocol} port {port} is in use (another owner)"
                         for protocol, port in spec.ports if not self._port_free(protocol, port)]
            reason = spec.refuse() if spec.refuse is not None else None
            if reason:
                problems.append(f"{spec.role}: {reason}")
        return problems

    def start(self, spec: ProcSpec) -> Owned:
        problems = self.refusals([spec])
        if problems:
            raise RunnerError("; ".join(problems))
        output = open(self.output_path(spec.role), "w", encoding="utf-8")
        try:
            handle = self._spawners[spec.kind](spec, output)
        except Exception:
            output.close()
            raise
        owned = Owned(spec, handle, output, _utc())
        self.owned.append(owned)
        self._wait_ready(owned)
        handle.confirm_ready()
        print(f"[approach] {spec.role} ready (pid {handle.pid})", flush=True)
        return owned

    def adopt(self, role: str, handle: Handle, stop: str = STOP_EOF) -> Owned:
        """A process another owner started for this run (the bridge, hitl_procs.py); stopped and checked here too."""
        if any(owned.spec.role == role for owned in self.owned):
            raise RunnerError(f"{role}: a second {role} in one run")
        owned = Owned(ProcSpec(role, (), None, 0.0, stop), handle, None, _utc())
        self.owned.append(owned)
        return owned

    def mark_stopped(self, role: str, exit_code: Optional[int]) -> None:
        """An adopted process its other owner stopped (the bridge, Board.stop): ended after stop if it exited."""
        owned = next((owned for owned in self.owned if owned.spec.role == role), None)
        if owned is not None and not owned.stopped and not owned.handle.running():
            owned.stopped = {"how": "stop", "exit_code": owned.handle.exit_code(),
                             "ended_after_stop": exit_code is not None, "ended_utc": _utc()}

    def _wait_ready(self, owned: Owned) -> None:
        spec = owned.spec
        if spec.ready is None:
            return
        pattern = re.compile(spec.ready, re.MULTILINE)
        path = spec.ready_file or self.output_path(spec.role)
        deadline = self._clock() + spec.ready_timeout_s
        while True:
            if owned.output is not None:
                owned.output.flush()
            if path.exists() and pattern.search(path.read_text(encoding="utf-8", errors="replace")):
                return
            if not owned.handle.running():
                raise RunnerError(f"{spec.role} exited with code {owned.handle.exit_code()} before it was ready; "
                                  f"see {self.output_path(spec.role)}")
            if self._clock() > deadline:
                raise RunnerError(f"{spec.role} not ready within {spec.ready_timeout_s:g} s; "
                                  f"see {self.output_path(spec.role)}")
            self._sleep(READY_POLL_S)

    def check(self) -> None:
        """Raise if an owned process has ended: the run would go on without it."""
        for owned in self.owned:
            if not owned.stopped and not owned.handle.running():
                raise RunnerError(f"{owned.spec.role} exited with code {owned.handle.exit_code()} during the run")

    def handle(self, role: str) -> Handle:
        return next(owned.handle for owned in self.owned if owned.spec.role == role)

    def stop_all(self) -> None:
        """Every owned process, newest first; each role's own stop, then terminate and kill. Never raises."""
        for owned in reversed(self.owned):
            try:
                if not owned.stopped:
                    owned.stopped = self._stop(owned)
                elif _alive(owned.handle.group_alive, []):       # its other owner stopped the leader; members are left
                    leftover = self._escalate(owned.handle, self._steps(owned, own_stop=False))
                    owned.stopped = {**owned.stopped, "leftover": leftover, "ended_after_stop": False}
            except Exception as exc:       # noqa: BLE001 - the others must still stop
                owned.stopped = {**owned.stopped, "error": f"{type(exc).__name__}: {exc}", "ended_after_stop": False}
            if owned.output is not None and not owned.output.closed:
                owned.output.close()

    def _stop(self, owned: Owned) -> dict:
        handle = owned.handle
        if not _alive(handle.group_alive, []):
            return {"how": "already exited", "exit_code": handle.exit_code(), "ended_after_stop": False}
        leader_gone = not _alive(handle.running, [])       # the leader exited during the run; its members are left
        result = self._escalate(handle, self._steps(owned, own_stop=True))
        return {**result, "ended_after_stop": result["how"] == "stop" and not leader_gone and "errors" not in result}

    @staticmethod
    def _steps(owned: Owned, own_stop: bool) -> list[tuple[str, Callable[[], None], float]]:
        handle = owned.handle
        steps = [("terminate", handle.terminate, TERMINATE_WAIT_S), ("kill", handle.kill, TERMINATE_WAIT_S)]
        return [("stop", handle.request_stop, owned.spec.stop_wait_s), *steps] if own_stop else steps

    def _escalate(self, handle: Handle, steps: Sequence[tuple[str, Callable[[], None], float]]) -> dict:
        """Each step until the whole process group is gone; a step that raises is recorded and the next one runs."""
        errors = []
        for how, act, wait_s in steps:
            try:
                act()
            except Exception as exc:       # noqa: BLE001 - the next step must still run
                errors.append(f"{how}: {type(exc).__name__}: {exc}")
                continue
            deadline = self._clock() + wait_s
            while _alive(handle.group_alive, errors) and self._clock() < deadline:
                self._sleep(READY_POLL_S)
            if not _alive(handle.group_alive, errors):
                return {"how": how, "exit_code": handle.exit_code(), "ended_utc": _utc(),
                        **({"errors": errors} if errors else {})}
        how = "still running after kill" if _alive(handle.group_alive, errors) else "ended after a failed step"
        return {"how": how, "exit_code": handle.exit_code(), **({"errors": errors} if errors else {})}

    def orphans(self) -> list[str]:
        return [owned.spec.role for owned in self.owned if _alive(owned.handle.group_alive, [])]

    def producers(self) -> dict[str, dict]:
        return {owned.spec.role: {"pid": owned.handle.pid, "argv": list(owned.spec.argv),
                                  "started_utc": owned.started_utc, **owned.stopped}
                for owned in self.owned}


__all__ = ["Handle", "Owned", "PosixHandle", "ProcSpec", "ProcessGroup", "RunnerError", "STOP_EOF", "STOP_SIGINT",
           "spawn_posix"]
