"""Spawning a lockstep WSL process, and holding it for its life.

:func:`spawn` starts a child in WSL that runs only once it is recorded
(``lockstep_wsl``, the run's ownership records). Right after ``wsl.exe``
starts, a :class:`WslChild` joins the caller's ``held`` list and owns it (an
interrupt between the two leaves an unrecorded child that has not had ``go``;
stdin EOF at owner exit kills it): it holds
the ``wsl.exe`` handle and copies the child's output to
``<run>/lockstep_wsl_<role>_host.log``. A dropped handle closes the pipe, and a
child that keeps writing then dies of ``SIGPIPE`` (step 9h, todo 46).
"""

from __future__ import annotations

import json
import shlex
import subprocess
import threading
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path
from typing import BinaryIO

from scripts import lockstep_wsl
from scripts.lockstep_wsl import OwnedProcess

#: Wall budget for each bounded wait of a close (the launcher's exit, the drain's end).
CLOSE_TIMEOUT_S = 30.0


class WslChild:
    """One spawned ``wsl.exe`` and everything the host holds for it.

    The output (the child's stdout and stderr) is read until EOF and written
    to the sidecar ``sink``. If the sink fails, the output is still read, so
    the child never blocks on a full pipe. The first failure is recorded, the
    rest of the output is dropped, and :meth:`close` raises the failure.
    """

    def __init__(self, process: subprocess.Popen, role: str) -> None:
        self.process = process
        self.role = role
        #: Set once the child is recorded (``lockstep_processes_host.json``).
        self.owned: OwnedProcess | None = None
        #: Set before ``go`` is written: from then on the child may run.
        self.go_attempted = False
        self.sink: BinaryIO | None = None
        self._error: BaseException | None = None
        self._drain: threading.Thread | None = None
        #: True once ``wsl.exe`` is reaped and the streams and the sink are closed.
        self.closed = False

    def read_ready(self, prefix: str) -> None:
        """Before the drain: read lines up to one starting with ``prefix``
        (each goes to the sink); an exit before it raises."""
        while True:
            line = self.process.stdout.readline()
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
        try:
            for chunk in iter(lambda: self.process.stdout.read1(1 << 16), b""):
                self._write(chunk)
        except Exception as error:  # noqa: BLE001 - raised by close()
            self._error = self._error or error

    def abandon(self, timeout_s: float = CLOSE_TIMEOUT_S) -> None:
        """A spawn that failed: stop the child, then :meth:`close`.

        Before ``go`` was attempted, closing stdin is the stop (the launcher's
        bash kills its stopped child on EOF). Once it was attempted, the child
        may run, so it is stopped by its record."""
        if self.go_attempted and self.owned is not None:
            owned = self.owned
            lockstep_wsl.proc("stop", owned.wsl_pid, owned.start_ticks, owned.cwd)
        self.close(timeout_s)

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
        if self._drain is not None and self._drain.is_alive():
            self._drain.join(timeout_s)
            if self._drain.is_alive():
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

    One cleanup scope: the :class:`WslChild` joins ``held`` as soon as
    ``wsl.exe`` starts. Any failure or interrupt abandons it
    (:meth:`WslChild.abandon`), and the spawn's error is raised. Cleanup that
    cannot finish (a bounded wait runs out) leaves the child unclosed in
    ``held`` for the owner's stop to retry.
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
        _hand_off(child, record, admit, ready)
    except BaseException:
        try:
            child.abandon()
        except BaseException:  # noqa: BLE001 - the child stays in held; the spawn's error wins
            pass
        raise
    return child


def _hand_off(child: WslChild, record: Path, admit: Callable[[OwnedProcess], bool] | None,
              ready: str | None) -> None:
    """Record the stopped child, admit it, send ``go``, then start its output."""
    process = child.process
    first = process.stdout.readline().decode().split()
    if len(first) != 2 or first[0] != "LOCKSTEP_PID":
        raise RuntimeError(f"{child.role} did not report its pid: {first}")
    pid = int(first[1])
    ticks, proc_cwd = lockstep_wsl.proc("identity", pid).split(" ", 1)
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
    child.go_attempted = True  # from here the child may run: abandon stops it by record
    process.stdin.write(b"go\n")
    process.stdin.flush()
    if ready is not None:
        child.read_ready(ready)
    child.start_drain()


__all__ = ["CLOSE_TIMEOUT_S", "WslChild", "output_sidecar", "spawn"]
