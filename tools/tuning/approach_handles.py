"""Handles for the approach runner's roles whose stop is not a plain signal or end of input.

The ZR10 emulator runs on Windows behind its WSL interop proxy (approach_chain.emulator); the aas-hitl publisher
stops cleanly only by ``drain`` then ``stop`` on its control socket (gstreamer_encoder.py: exit 0 only then; a signal
is a failure, exit 1).
"""
from __future__ import annotations

import json
import os
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


class InteropHandle(PosixHandle):
    """A Windows process behind its WSL interop proxy: the proxy ending does not end it, so it counts as alive while
    a Windows pid holds its ports, and a kill ends those pids too (the start refused any earlier owner)."""

    def __init__(self, process: subprocess.Popen, stop: str, ports: tuple[int, ...]) -> None:
        super().__init__(process, stop)
        self.ports = ports

    def kill(self) -> None:
        super().kill()
        for pid in windows_udp_owners(self.ports):
            subprocess.run(["taskkill.exe", "/PID", str(pid), "/F"], capture_output=True, timeout=30, check=False)

    def group_alive(self) -> bool:
        return super().group_alive() or bool(windows_udp_owners(self.ports))


def spawn_interop(ports: tuple[int, ...]) -> Callable[[ProcSpec, IO[str]], InteropHandle]:
    def spawn(spec: ProcSpec, output: IO[str]) -> InteropHandle:
        process = subprocess.Popen(list(spec.argv), stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT,
                                   cwd=spec.cwd, env={**os.environ, **(spec.env or {})}, start_new_session=True,
                                   text=True)
        return InteropHandle(process, spec.stop, ports)
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
