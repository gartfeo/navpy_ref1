"""Handles for the approach runner's roles whose stop is not a plain signal or end of input.

The ZR10 emulator runs on Windows behind its WSL interop proxy (approach_chain.emulator); the aas-hitl publisher
stops cleanly only by ``drain`` then ``stop`` on its control socket (gstreamer_encoder.py: exit 0 only then; a signal
is a failure, exit 1).
"""
from __future__ import annotations

import csv
import json
import os
import re
import signal
import socket
import subprocess
from pathlib import Path
from typing import IO, Callable, Optional

from tools.tuning.approach_procs import PosixHandle, ProcSpec, spawn_posix

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


def windows_image(pid: int, listing: Optional[str] = None) -> str:
    """The image name of Windows process *pid*, or "" when there is none (tasklist /FO CSV: image, pid, ...)."""
    if listing is None:
        listing = subprocess.run(["tasklist.exe", "/FI", f"PID eq {pid}", "/NH", "/FO", "CSV"], capture_output=True,
                                 text=True, timeout=30, check=False).stdout
    return next((row[0] for row in csv.reader(listing.splitlines()) if len(row) > 1 and row[1] == str(pid)), "")


class InteropHandle(PosixHandle):
    """A Windows Python behind its WSL interop proxy: the proxy ending does not end it. Its own pid comes from its
    ready line (*pid_line*); it counts as alive while that pid is a Python, and a kill ends that pid's tree only.
    Before the line names a pid, a Windows owner of its *ports* counts as alive (reported, never killed)."""

    def __init__(self, process: subprocess.Popen, stop: str, ports: tuple[int, ...], output: Path,
                 pid_line: str) -> None:
        super().__init__(process, stop)
        self.ports, self.output, self.pid_line = ports, output, re.compile(pid_line, re.MULTILINE)
        self._windows_pid: Optional[int] = None

    def windows_pid(self) -> Optional[int]:
        if self._windows_pid is None and self.output.exists():
            match = self.pid_line.search(self.output.read_text(encoding="utf-8", errors="replace"))
            self._windows_pid = int(match.group(1)) if match else None
        return self._windows_pid

    def _own_alive(self) -> bool:
        pid = self.windows_pid()
        return pid is not None and windows_image(pid).lower().startswith("python")

    def kill(self) -> None:
        super().kill()
        if self._own_alive():
            subprocess.run(["taskkill.exe", "/PID", str(self.windows_pid()), "/T", "/F"], capture_output=True,
                           timeout=30, check=False)

    def group_alive(self) -> bool:
        if super().group_alive():
            return True
        return self._own_alive() if self.windows_pid() is not None else bool(windows_udp_owners(self.ports))


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
