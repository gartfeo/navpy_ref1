"""WSL relay: one SITL's serial-step socket to the Windows coordinator.

Runs under WSL's python3 (standard library only). It binds the AF_UNIX
SEQPACKET socket the firmware connects to (``navpy-serial-step.sock`` in the
instance directory) and a loopback TCP port the coordinator connects to, then
copies strictly alternating frames and replies, each TCP message prefixed by
its length (``<I``). It adds nothing and reorders nothing.

``--delay-seed`` injects seeded host delays before each forward
(acceptance (c)): the journal must not change.

Usage: python3 lockstep_relay.py --dir DIR --port PORT [--delay-seed N]
"""

from __future__ import annotations

import argparse
import os
import random
import socket
import struct
import sys
import time

SOCKET_NAME = "navpy-serial-step.sock"
LENGTH = struct.Struct("<I")
MAX_MESSAGE = 1 << 20
#: Upper bound of one injected host delay (seconds).
MAX_DELAY_S = 0.005


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("coordinator closed")
        data += chunk
    return bytes(data)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--delay-seed", type=int, default=0)
    args = parser.parse_args()
    delays = random.Random(args.delay_seed) if args.delay_seed else None

    path = os.path.join(args.dir, SOCKET_NAME)
    if os.path.exists(path):
        os.unlink(path)
    unix_listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    unix_listener.bind(path)
    unix_listener.listen(1)
    tcp_listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_listener.bind(("127.0.0.1", args.port))
    tcp_listener.listen(1)
    print(f"LOCKSTEP_RELAY_READY pid={os.getpid()}", flush=True)

    coordinator, _ = tcp_listener.accept()
    coordinator.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sitl, _ = unix_listener.accept()
    frames = 0
    while True:
        frame = sitl.recv(MAX_MESSAGE)
        if not frame:
            break  # the SITL closed: the run is over
        if delays:
            time.sleep(delays.random() * MAX_DELAY_S)
        coordinator.sendall(LENGTH.pack(len(frame)) + frame)
        (size,) = LENGTH.unpack(recv_exact(coordinator, LENGTH.size))
        if size > MAX_MESSAGE:
            print(f"LOCKSTEP_RELAY_INVALID reply_size={size}", file=sys.stderr, flush=True)
            return 72
        reply = recv_exact(coordinator, size)
        if delays:
            time.sleep(delays.random() * MAX_DELAY_S)
        sitl.send(reply)
        frames += 1
    coordinator.close()
    print(f"LOCKSTEP_RELAY_DONE frames={frames}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
