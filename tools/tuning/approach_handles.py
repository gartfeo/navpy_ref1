"""Handles for the approach runner's roles whose stop is not a plain signal or end of input.

The ZR10 emulator runs on Windows behind its WSL interop proxy (approach_chain.emulator); the aas-hitl publisher
stops cleanly only by ``drain`` then ``stop`` on its control socket (gstreamer_encoder.py: exit 0 only then; a signal
is a failure, exit 1).
"""
from __future__ import annotations

import json
import os
import re
import signal
import socket
import subprocess
from pathlib import Path
from typing import IO, Callable, Optional

from tools.tuning.approach_procs import PosixHandle, ProcSpec, RunnerError, spawn_posix

CONTROL_TIMEOUT_S = 30.0         # the publisher's drain waits its profile's drain_timeout_s (5 s) for the encoder


def windows_udp_owners(ports: tuple[int, ...], listing: Optional[str] = None) -> set[int]:
    """The Windows pids bound to any of *ports* (netstat -ano -p UDP: proto, local address, foreign, pid)."""
    if listing is None:
        listing = subprocess.run(["netstat.exe", "-ano", "-p", "UDP"], capture_output=True, text=True, timeout=30,
                                 check=False).stdout
    owners = set()
    for line in listing.splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[0] == "UDP" and fields[3].isdigit() \
                and any(fields[1].endswith(f":{port}") for port in ports):
            owners.add(int(fields[3]))
    return owners


def windows_udp_in_use(ports: tuple[int, ...]) -> Optional[str]:
    owners = windows_udp_owners(ports)
    return f"Windows pid(s) {sorted(owners)} own udp port(s) {list(ports)}" if owners else None


GONE = "gone"


def windows_process(pid: int, started: str = "") -> str:
    """Windows process *pid*'s start time (FILETIME, UTC; with the pid, its identity), or GONE when there is none.

    With *started*, the process is killed in the same call if it still has that start time. ``$p.Handle`` opens the
    process handle first and .NET keeps it for StartTime and Kill (without it each opens its own by pid), so the
    kill reaches the process whose start time was checked even if the pid were reused meanwhile. A query that fails
    raises.
    """
    kill = (f"if ($p.StartTime.ToFileTimeUtc() -eq {int(started)}) "
            f"{{ $p.Kill(); $p.WaitForExit(10000) | Out-Null; '{GONE}'; return }}; ") if started else ""
    script = (f"$p = Get-Process -Id {pid} -ErrorAction SilentlyContinue; "
              f"if ($p) {{ $null = $p.Handle; {kill}$p.StartTime.ToFileTimeUtc() }} else {{ '{GONE}' }}")
    result = subprocess.run(["powershell.exe", "-NoProfile", "-Command", script], capture_output=True, text=True,
                            timeout=30, check=False)
    return parse_process(result.returncode, result.stdout, result.stderr)


def parse_process(returncode: int, stdout: str, stderr: str) -> str:
    answer = stdout.strip()
    if returncode != 0 or stderr.strip() or not (answer == GONE or answer.isdigit()):
        raise OSError(f"Windows process query failed (exit {returncode}): {(stderr or stdout).strip()[:200]}")
    return answer


class InteropHandle(PosixHandle):
    """A Windows Python behind its WSL interop proxy: the proxy ending does not end it. Its ready line (*pid_line*)
    names its own pid and creation time (process_identity.py); at ready that pid must have that start time, and
    from then on it counts as alive, and a kill ends it, only while it still does (a reused pid is another
    process; a failed query raises, so it counts as alive). Before ready, a Windows owner of its *ports* counts as alive (reported, never killed)."""

    def __init__(self, process: subprocess.Popen, stop: str, ports: tuple[int, ...], output: Path,
                 pid_line: str) -> None:
        super().__init__(process, stop)
        self.ports, self.output, self.pid_line = ports, output, re.compile(pid_line, re.MULTILINE)
        self.windows_pid: Optional[int] = None
        self._started = ""

    def confirm_ready(self) -> None:
        match = self.pid_line.search(self.output.read_text(encoding="utf-8", errors="replace"))
        if not match or windows_process(int(match.group(1))) != match.group(2):     # gone, or its pid reused
            raise RunnerError(f"the ready line names no running Windows pid with its start time; see {self.output}")
        self.windows_pid, self._started = int(match.group(1)), match.group(2)

    def kill(self) -> None:
        super().kill()
        if self.windows_pid is not None:
            windows_process(self.windows_pid, self._started)

    def group_alive(self) -> bool:
        if super().group_alive():
            return True
        if self.windows_pid is None:
            return bool(windows_udp_owners(self.ports))
        return windows_process(self.windows_pid) == self._started


def spawn_interop(ports: tuple[int, ...], pid_line: str) -> Callable[[ProcSpec, IO[str]], InteropHandle]:
    def spawn(spec: ProcSpec, output: IO[str]) -> InteropHandle:
        process = subprocess.Popen(list(spec.argv), stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT,
                                   cwd=spec.cwd, env={**os.environ, **(spec.env or {})}, start_new_session=True,
                                   text=True)
        return InteropHandle(process, spec.stop, ports, Path(output.name), pid_line)
    return spawn


def control(path: str, command: str, timeout_s: float = CONTROL_TIMEOUT_S) -> dict:
    """One command on the publisher's control socket; its one JSON line back."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_s)
        sock.connect(path)
        sock.sendall((command + "\n").encode("utf-8"))
        return json.loads(sock.makefile("r", encoding="utf-8").readline())


class PublisherHandle(PosixHandle):
    """The publisher's stop: drain (EOS through the encoder), then stop; SIGINT only if its socket is not there."""

    def __init__(self, process: subprocess.Popen, stop: str, status: Path) -> None:
        super().__init__(process, stop)
        self.status = status

    def request_stop(self) -> None:
        try:
            path = json.loads(self.status.read_text(encoding="utf-8"))["control_socket"]
            control(path, "drain")
            control(path, "stop")
        except (OSError, ValueError, KeyError):     # not started that far, or already gone: its signal stop
            self._signal(signal.SIGINT)


def spawn_publisher(spec: ProcSpec, output: IO[str]) -> PublisherHandle:
    if spec.ready_file is None:
        raise ValueError("the publisher's spec names its status file (publisher.json) as its ready file")
    posix = spawn_posix(spec, output)
    return PublisherHandle(posix.process, spec.stop, spec.ready_file)
