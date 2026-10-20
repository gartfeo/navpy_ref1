"""WSL side of ``test_router_parity``: replay datagrams through the real mavlink-routerd.

Runs under WSL's ``python3`` with the standard library only, offline, never
inside a lockstep run. Reads a JSON job on stdin::

    {"binary": "/usr/bin/mavlink-routerd", "endpoints": [...names],
     "cases": {name: [[endpoint, hex, fence], ...]},
     "filters": {name: {endpoint: [component, ...]}}}

Per case it starts its own router, ``-s 255`` as ``run_swarm.sh`` does, with
an empty conf dir (the default ``main.conf`` stays unread), no TCP server,
and one UDP server endpoint per name on a free loopback port. Without
filters the endpoints are command-line arguments and the conf file holds
only ``[General]``; a case with filters declares every endpoint in the conf
file instead (``[UdpEndpoint <name>]``, ``Mode = Server``, its
``BlockSrcCompOut``). The router logs at trace level; its lines ``to unknown
sysid/compid`` count its ``msg_to_unknown``.
Each harness socket first sends one ``0x00`` byte, which teaches the router
the socket's address and nothing else (no start marker: the router skips
it); the harness waits until the router has read it (its socket's receive
queue in ``/proc/net/udp`` is empty). Then it sends the steps in order. A
``fence`` step is a broadcast packet from a source only its sender uses:
the harness waits until every other endpoint has received it, so every
packet routed before it has arrived. The router is stopped by its own pid
and the temporary directory removed.

Writes ``{case: {"received": {endpoint: hex of everything it received},
"unknown": count}}`` on stdout.
"""

from __future__ import annotations

import json
import os
import select
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable

WAIT_S = 5.0


def free_ports(count: int) -> list[int]:
    probes = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(count)]
    for probe in probes:
        probe.bind(("127.0.0.1", 0))
    ports = [probe.getsockname()[1] for probe in probes]
    for probe in probes:
        probe.close()
    return ports


def receive_queue(port: int) -> int | None:
    with open("/proc/net/udp", encoding="ascii") as stream:
        for line in stream.readlines()[1:]:
            fields = line.split()
            if int(fields[1].split(":")[1], 16) == port:
                return int(fields[4].split(":")[1], 16)
    return None


def wait_until(condition: Callable[[], bool], what: str) -> None:
    deadline = time.monotonic() + WAIT_S
    while not condition():
        if time.monotonic() > deadline:
            raise RuntimeError(f"timed out waiting for {what}")
        time.sleep(0.001)


UNKNOWN_LINE = "to unknown sysid/compid"


def write_conf(path: str, names: list[str], ports: list[int], filters: dict[str, list[int]]) -> list[str]:
    """The conf file; returns the endpoint arguments the command line still needs."""
    lines = ["[General]"]  # an empty file does not parse
    endpoints = [f"127.0.0.1:{port}" for port in ports]
    if filters:
        endpoints = []
        for name, port in zip(names, ports):
            lines += ["", f"[UdpEndpoint {name}]", "Mode = Server", "Address = 127.0.0.1",
                      f"Port = {port}"]
            if filters.get(name):
                lines.append("BlockSrcCompOut = " + ",".join(str(c) for c in filters[name]))
    with open(path, "w", encoding="ascii") as stream:
        stream.write("\n".join(lines) + "\n")
    return endpoints


def run_case(binary: str, names: list[str], steps: list, workdir: str,
             filters: dict[str, list[int]]) -> dict[str, object]:
    conf_dir = os.path.join(workdir, "conf.d")
    os.makedirs(conf_dir, exist_ok=True)
    conf = os.path.join(workdir, "general.conf")
    ports = free_ports(len(names))
    endpoints = write_conf(conf, names, ports, filters)
    command = [binary, "-s", "255", "-t", "0", "-g", "trace", "-c", conf, "-d", conf_dir, *endpoints]
    log_path = os.path.join(workdir, "router.log")
    log = open(log_path, "w", encoding="utf-8")
    router = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    sockets = {}
    try:
        wait_until(lambda: all(receive_queue(port) is not None for port in ports), "router ports")
        for name, port in zip(names, ports):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 0))
            sock.connect(("127.0.0.1", port))
            sockets[name] = sock
            sock.send(b"\x00")
            wait_until(lambda port=port: receive_queue(port) == 0, f"{name}'s address")
        received = {name: bytearray() for name in names}
        for endpoint, data, fence in steps:
            payload = bytes.fromhex(data)
            sockets[endpoint].send(payload)
            if fence:
                others = [name for name in names if name != endpoint]
                wait_until(lambda: _drain(sockets, received, others, payload), f"fence from {endpoint}")
        _drain(sockets, received, names, b"")
        if router.poll() is not None:
            raise RuntimeError(f"the router exited ({router.returncode})")
        result: dict[str, object] = {
            "received": {name: bytes(data).hex() for name, data in received.items()}}
    finally:
        for sock in sockets.values():
            sock.close()
        router.terminate()
        router.wait(timeout=WAIT_S)
        log.close()
    with open(log_path, encoding="utf-8", errors="replace") as stream:
        result["unknown"] = sum(UNKNOWN_LINE in line for line in stream)
    return result


def _drain(sockets: dict[str, socket.socket], received: dict[str, bytearray], names: list[str],
           fence: bytes) -> bool:
    """Read what has arrived; true once each of ``names`` ends with ``fence``."""
    while True:
        ready, _, _ = select.select([sockets[name] for name in names], [], [], 0)
        if not ready:
            break
        for sock in ready:
            name = next(n for n, s in sockets.items() if s is sock)
            received[name] += sock.recv(65535)
    return all(received[name].endswith(fence) for name in names)


def main() -> int:
    job = json.load(sys.stdin)
    workdir = tempfile.mkdtemp(prefix="navpy-router-parity-")
    try:
        results = {name: run_case(job["binary"], job["endpoints"], steps, workdir,
                                  job.get("filters", {}).get(name, {}))
                   for name, steps in job["cases"].items()}
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
    json.dump(results, sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
