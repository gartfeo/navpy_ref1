"""WSL-side bounded peer for the opt-in, identity-only SITL experiment."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import signal
import time
from types import FrameType

from simtime_step_protocol import StepSequence, WINDOW_SIZE, WIRE


# Deliberate experimental perturbation, not scheduling logic or a retry policy.
DELAYS_S = (0.0, 0.002, 0.0, 0.006, 0.001)
FAULTS = ("none", "boot", "step", "version", "truncated", "oversized",
          "duplicate", "disconnect", "timeout")


def reply_packet(packet: bytes, fault: str) -> bytes:
    result = bytearray(packet)
    if fault in ("boot", "step", "version"):
        result[{"boot": 8, "step": 24, "version": 0}[fault]] ^= 1
    elif fault == "truncated":
        return packet[:-1]
    elif fault == "oversized":
        return packet + b"!"
    return bytes(result)


def serve(directory: Path, *, delayed: bool, fault: str) -> dict:
    """Never remove a pre-existing endpoint; own this socket for one run only."""
    address = directory / "navpy-step.sock"
    sequence = StepSequence()
    records = []
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as listener:
        listener.settimeout(120.0)  # outer experiment startup watchdog
        listener.bind(str(address))
        identity = address.stat().st_ino
        try:
            listener.listen(1)
            print("NAVPY_STEP_PEER_READY", flush=True)
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(10.0)
                for _ in range(WINDOW_SIZE):
                    packet = connection.recv(WIRE.size + 1)
                    request = sequence.accept(packet)
                    records.append({"step": request.step, "tick": request.tick,
                                    "source_us": request.source_us,
                                    "wall_ns": time.monotonic_ns(),
                                    "boot": request.boot, "vehicle": request.vehicle})
                    injected = fault if request.step == 5 else "none"
                    if injected == "disconnect":
                        break
                    if injected == "timeout":
                        time.sleep(6.0)  # exceeds the firmware's native 5s deadline
                        break
                    if delayed:
                        time.sleep(DELAYS_S[(request.step - 1) % len(DELAYS_S)])
                    connection.sendall(reply_packet(packet, injected))
                    if injected == "duplicate":
                        connection.sendall(packet)
                        # Keep the peer open while SITL encounters the queued
                        # duplicate at its next step; don't test EOF instead.
                        connection.recv(WIRE.size + 1)
                        try:
                            connection.recv(WIRE.size + 1)
                        except ConnectionResetError:
                            pass  # firmware rejects the queued duplicate and exits
                    if injected != "none":
                        break
        finally:
            if address.exists() and address.stat().st_ino == identity:
                address.unlink()
    return {"version": 1, "delayed": delayed, "fault": fault, "records": records}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--delayed", action="store_true")
    parser.add_argument("--fault", choices=FAULTS, default="none")
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--pid-file", type=Path, required=True)
    args = parser.parse_args()
    with args.pid_file.open("x", encoding="ascii") as output:
        output.write(str(os.getpid()))
    def stop(_signum: int, _frame: FrameType | None) -> None:
        raise SystemExit(143)
    signal.signal(signal.SIGTERM, stop)
    result = serve(args.directory, delayed=args.delayed, fault=args.fault)
    with args.result.open("x", encoding="utf-8") as output:
        json.dump(result, output)


if __name__ == "__main__":
    main()
