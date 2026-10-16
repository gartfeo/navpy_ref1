"""Spawning a lockstep WSL process, and holding it for its life.

:func:`spawn` starts a child in WSL that runs only once it is recorded
(``lockstep_wsl``, the run's ownership records). Right after ``wsl.exe``
starts, a :class:`WslChild` joins the caller's ``held`` list and owns it (an
interrupt between the two leaves an unrecorded child that has not had ``go``;
stdin EOF at owner exit kills it): it holds
the ``wsl.exe`` handle and copies the child's output to
``<run>/lockstep_wsl_<role>_host.log``. A dropped handle closes the pipe, and a
child that keeps writing then dies of ``SIGPIPE`` (step 9h, todo 46).

Every wait of a spawn is bounded: the pid line, the identity probe and the
ready line each get :data:`SPAWN_WAIT_S`, or less when the caller's deadline
leaves less (``wait_budget``; a fleet's later SITLs launch on frame 1's
exchange deadline, step 10d).

One ownership structure, so nothing leaks and nothing in the spawn path
blocks on cleanup (Codex 10d round 2):

1. A :class:`WslChild` starts no thread when it is built. It joins ``held``
   first; only then, inside the spawn's cleanup scope, does its reader start.
2. A failed or interrupted spawn only releases the child
   (:meth:`WslChild.release`: stdin closed, which kills a launcher's child
   that has not had ``go``) and raises at once. It never stops or reaps.
3. The owner (``WslFleet.stop``, the step 7 driver's cleanup) stops every
   recorded process by ownership; every child that may run is recorded
   before its ``go``. Then it closes every held child with bounded waits
   (:meth:`WslChild.close`); one that does not finish stays in ``held`` for
   the owner's next stop.
"""

from __future__ import annotations

import json
import queue
import shlex
import subprocess
import threading
import time
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path
from typing import BinaryIO

from scripts import lockstep_wsl
from scripts.lockstep_wsl import OwnedProcess

#: Wall budget for each bounded wait of a close (the launcher's exit, the drain's end).
CLOSE_TIMEOUT_S = 30.0
#: Own wall budget of each wait of a spawn: the launcher's pid line, the
#: identity probe (one ``wsl.exe`` round trip), the ready line. A cold WSL
#: start takes seconds; a minute means the launcher is stuck.
SPAWN_WAIT_S = 60.0


class WslChild:
    """One spawned ``wsl.exe`` and everything the host holds for it.

    The output (the child's stdout and stderr) is read line by line by a
    reader thread (:meth:`start_reader`, started once the child is held), so
    every wait for a line is bounded (:meth:`next_line`); after the hand-off
    it is written to the sidecar
    ``sink`` until EOF. If the sink fails, the output is still read, so the
    child never blocks on a full pipe. The first failure is recorded, the
    rest of the output is dropped, and :meth:`close` raises the failure.
    """

    def __init__(self, process: subprocess.Popen, role: str) -> None:
        self.process = process
        self.role = role
        #: Set once the child is recorded (``lockstep_processes_host.json``).
        self.owned: OwnedProcess | None = None
        self.sink: BinaryIO | None = None
        self._error: BaseException | None = None
        self._drain: threading.Thread | None = None
        #: Output lines as read; ``b""`` is EOF (left in place once seen).
        self._lines: queue.Queue[bytes] = queue.Queue()
        self._reader: threading.Thread | None = None
        #: True once ``wsl.exe`` is reaped and the streams and the sink are closed.
        self.closed = False

    def start_reader(self) -> None:
        """Read the output from here on; only for a child already in ``held``."""
        self._reader = threading.Thread(target=self._read, name=f"lockstep-wsl-{self.role}-reader",
                                        daemon=True)
        self._reader.start()

    def _read(self) -> None:
        try:
            for line in iter(self.process.stdout.readline, b""):
                self._lines.put(line)
        except Exception as error:  # noqa: BLE001 - raised by close()
            self._error = self._error or error
        finally:
            self._lines.put(b"")

    def next_line(self, timeout_s: float) -> bytes:
        """The next output line (``b""`` at EOF); none within ``timeout_s`` is ``TimeoutError``."""
        try:
            line = self._lines.get(timeout=max(0.0, timeout_s))
        except queue.Empty:
            raise TimeoutError(f"{self.role}: no output line within {timeout_s:.1f} s") from None
        if not line:
            self._lines.put(b"")  # EOF stays visible to the next reader
        return line

    def read_ready(self, prefix: str, timeout_s: Callable[[], float]) -> None:
        """Before the drain: read lines up to one starting with ``prefix``
        (each goes to the sink), each within ``timeout_s()``; an exit before it raises."""
        while True:
            line = self.next_line(timeout_s())
            self._write(line)
            if not line:
                raise RuntimeError(f"{self.role} exited before {prefix}")
            if line.decode(errors="replace").startswith(prefix):
                return

    def start_drain(self) -> None:
        self._drain = threading.Thread(target=self._copy, name=f"lockstep-wsl-{self.role}-output",
                                       daemon=True)
        self._drain.start()

    def _write(self, data: bytes) -> None:
        if self._error is None and self.sink is not None:
            try:
                self.sink.write(data)
                self.sink.flush()
            except Exception as error:  # noqa: BLE001 - raised by close(); the reading goes on
                self._error = error

    def _copy(self) -> None:
        for line in iter(self._lines.get, b""):
            self._write(line)

    def release(self) -> None:
        """A spawn that failed: close stdin and return at once (never waits).

        Before ``go``, EOF makes the launcher's bash kill its stopped child.
        Once ``go`` was attempted the child may run; it is recorded, and the
        owner stops it by ownership before it closes the child."""
        if not self.process.stdin.closed:
            self.process.stdin.close()

    def close(self, timeout_s: float = CLOSE_TIMEOUT_S) -> None:
        """After the Linux process is stopped: reap ``wsl.exe``, end the drain,
        close the streams and the sink. Every wait is bounded; one that runs
        out raises and leaves the rest for a retry (stdout is never closed
        under a live drain). Then raises the first output failure once."""
        if self.closed:
            return
        if not self.process.stdin.closed:
            self.process.stdin.close()
        self.process.wait(timeout=timeout_s)
        for thread in (self._reader, self._drain):
            if thread is not None and thread.is_alive():
                thread.join(timeout_s)
                if thread.is_alive():
                    raise RuntimeError(f"{self.role}: output still open after {timeout_s} s")
        self.process.stdout.close()
        if self.sink is not None:
            try:
                self.sink.close()
            except Exception as error:  # noqa: BLE001 - raised below
                self._error = self._error or error
        self.closed = True
        if self._error is not None:
            raise RuntimeError(f"{self.role}: output sidecar failed: "
                               f"{type(self._error).__name__}: {self._error}") from self._error


def output_sidecar(record: Path, role: str) -> Path:
    """Where a role's WSL output goes: beside the run's process record, ``*_host``."""
    return record.with_name(f"lockstep_wsl_{role}_host.log")


def spawn(
    role: str, script: str, cwd: str, record: Path, *, held: list[WslChild],
    admit: Callable[[OwnedProcess], bool] | None = None, ready: str | None = None,
    on_go: Callable[[], None] | None = None,
    wait_budget: Callable[[float], float] | None = None,
) -> WslChild:
    """Start ``script`` in WSL under ``cwd``; it runs only once it is recorded.

    The child stops itself before it runs anything. The WSL-side bash
    publishes its pid only once it is stopped, then continues it on a ``go``
    line from us, written after the pid, start time and cwd are recorded
    atomically. Anything else (we fail, are interrupted or die: stdin closes)
    makes that bash kill the child, so no unrecorded process survives. The
    child runs as bash's child, not by an ``exec`` of the session's bash: a
    SITL that replaced the top-level bash of a ``wsl.exe`` session exited
    early.

    ``admit`` (the lockstep slot's ``record_child`` under the registry lock)
    runs after the run record and before ``go``; a refusal (the slot is
    ``stopping``) closes stdin instead, so the child is killed unstarted.

    After ``go`` the child's output goes to :func:`output_sidecar` (appended).
    With ``ready``, spawn first waits for a line starting with it (the
    relay's ``LOCKSTEP_RELAY_READY``); an exit before it raises.

    ``on_go`` runs right before ``go`` is written: a fleet launch anchors
    frame 1's exchange deadline at SITL 1's ``go`` there, never later than
    the firmware can arm its own deadline (``lockstep_fleet.md``, Fleet
    barrier, launch step 3).

    Each wait (the pid line, the identity probe, the ready line) gets
    :data:`SPAWN_WAIT_S`, or ``wait_budget(own)`` when given: the caller's
    deadline bounds it, and its raising ends the spawn. A wait that runs
    out raises ``TimeoutError`` (``subprocess.TimeoutExpired`` for the probe).

    One cleanup scope: the :class:`WslChild` joins ``held`` as soon as
    ``wsl.exe`` starts, before any thread of it runs. Any failure or
    interrupt releases it (:meth:`WslChild.release`) and the spawn's error is
    raised at once; stopping and closing are the owner's (module docstring).
    """
    launcher = (
        f"cd {shlex.quote(cwd)} && {{ {{ kill -STOP $BASHPID; exec {script}; }} & c=$!; "
        f"while [ \"$(cut -d' ' -f3 /proc/$c/stat)\" != T ]; do sleep 0.001; done; "
        f"echo LOCKSTEP_PID $c; "
        f"if read -r go && [ \"$go\" = go ]; then kill -CONT $c; wait $c; "
        f"else kill -KILL $c; fi; }}"
    )
    child = WslChild(subprocess.Popen(
        ["wsl.exe", "-e", "bash", "-c", launcher],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE,
    ), role)
    held.append(child)
    try:
        child.start_reader()
        _hand_off(child, record, admit, ready, on_go, wait_budget or (lambda own: own))
    except BaseException:
        try:
            child.release()
        except BaseException:  # noqa: BLE001 - the child stays in held; the spawn's error wins
            pass
        raise
    return child


def _hand_off(child: WslChild, record: Path, admit: Callable[[OwnedProcess], bool] | None,
              ready: str | None, on_go: Callable[[], None] | None,
              budget: Callable[[float], float]) -> None:
    """Record the stopped child, admit it, send ``go``, then start its output."""
    process = child.process
    first = child.next_line(budget(SPAWN_WAIT_S)).decode().split()
    if len(first) != 2 or first[0] != "LOCKSTEP_PID":
        raise RuntimeError(f"{child.role} did not report its pid: {first}")
    pid = int(first[1])
    ticks, proc_cwd = lockstep_wsl.proc("identity", pid,
                                        timeout_s=budget(SPAWN_WAIT_S)).split(" ", 1)
    owned = OwnedProcess(child.role, pid, int(ticks), proc_cwd)
    records = json.loads(record.read_text()) if record.exists() else []
    records.append(asdict(owned))
    tmp = record.with_suffix(".tmp")
    tmp.write_text(json.dumps(records, indent=2), encoding="utf-8")
    tmp.replace(record)  # atomic: a crash never leaves a half-written record
    child.owned = owned
    if admit is not None and not admit(owned):
        raise RuntimeError(f"{child.role}: the lockstep slot is stopping; not started")
    child.sink = output_sidecar(record, child.role).open("ab")
    if on_go is not None:
        on_go()
    # From here the child may run: its record is the owner's to stop.
    process.stdin.write(b"go\n")
    process.stdin.flush()
    if ready is not None:
        ready_end = time.monotonic() + SPAWN_WAIT_S
        child.read_ready(ready, lambda: budget(ready_end - time.monotonic()))
    child.start_drain()


__all__ = ["CLOSE_TIMEOUT_S", "SPAWN_WAIT_S", "WslChild", "output_sidecar", "spawn"]
