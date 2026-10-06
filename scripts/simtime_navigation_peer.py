"""Owned Linux peer: camera -> existing navigation -> next-tick AP command."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from types import FrameType

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from navpy.modules.common.models.location import Location  # noqa: E402
from scripts.simtime_navigation_protocol import (  # noqa: E402
    ATTITUDE, DRAIN_STEPS, GUIDED, HOST_DELAYS_S, TAKEOFF_ARM, SNAPSHOT_SIZE,
    WINDOW_SIZE, SnapshotSequence, StepCommand,
)
from scripts.simtime_navigation_faults import FAULTS, corrupt  # noqa: E402
from scripts.simtime_navigation_config import read_defaults  # noqa: E402
from scripts.simtime_navigation_runtime import SynchronousNavigation  # noqa: E402


def serve(args: argparse.Namespace) -> dict:
    parameters, throttle_policy = read_defaults(args.defaults, args.throttle_percent)
    address = args.directory / "navpy-navigation.sock"
    sequence = SnapshotSequence()
    records = []
    engine = SynchronousNavigation(
        Location(*args.dock, is_absolute=True), speedup=args.speedup,
        camera_fraction=(4, 5) if args.camera_hz == 40 else (1, 1),
        output=args.result.parent, trim_throttle_percent=parameters["TRIM_THROTTLE"],
        configured_throttle_percent=args.throttle_percent)
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as listener:
        listener.settimeout(120.)
        listener.bind(str(address))
        inode = address.stat().st_ino
        try:
            listener.listen(1)
            print("NAVPY_NAVIGATION_PEER_READY", flush=True)
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(10.)
                for index in range(WINDOW_SIZE):
                    raw = connection.recv(SNAPSHOT_SIZE + 1)
                    receipt = time.monotonic()
                    snapshot = sequence.accept(raw)
                    injected = args.fault if index == 4 else "none"
                    if injected != "none":
                        with (args.result.parent / "injection.json").open("x") as stream:
                            json.dump({"fault": injected, "step": snapshot.identity.step}, stream)
                    if injected == "disconnect":
                        break
                    if injected == "timeout":
                        time.sleep(6.)
                        break
                    delay = HOST_DELAYS_S[index % len(HOST_DELAYS_S)] if args.delayed else 0.
                    if injected == "expired":
                        delay = 1.5 / args.speedup
                    if delay:
                        time.sleep(delay)  # experimental host perturbation only
                    if index >= WINDOW_SIZE - DRAIN_STEPS:
                        command, evidence = StepCommand(), engine.drain()
                    else:
                        command, evidence = engine.advance(snapshot, receipt)
                    if index == 0:
                        command = StepCommand(TAKEOFF_ARM)
                    if index == args.guided_step - 1:
                        command = StepCommand(GUIDED)
                    if injected == "rejected":
                        command = StepCommand(ATTITUDE, 196, (1., 0., 0., 0.))
                    reply = sequence.reply(command)
                    elapsed = time.monotonic() - receipt
                    if elapsed > 1.0 / args.speedup:
                        raise ValueError("receipt liveness exceeded before reply")
                    connection.sendall(corrupt(reply, injected))
                    if injected == "duplicate":
                        connection.sendall(reply)
                    records.append({"snapshot": raw.hex(), "reply": reply.hex(),
                                    "wall_s": receipt, "elapsed_s": elapsed,
                                    "delay_s": delay, "evidence": evidence})
            return {"version": 4, "throttle_policy": throttle_policy, "records": records, "camera_hz": args.camera_hz,
                    "speedup": args.speedup, "delayed": args.delayed,
                    "dock": args.dock, "guided_step": args.guided_step,
                    "seed_step": engine.seed_step, "command_count": engine.command_count}
        except Exception as error:
            with (args.result.parent / "peer-failure.json").open("x") as stream:
                json.dump({"error": str(error), "records": records}, stream)
            raise
        finally:
            engine.close()
            if address.exists() and address.stat().st_ino == inode:
                address.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--throttle-percent", type=float,
                        help="Explicit approach throttle: 0..100 percent, -1 masks throttle; omitted uses normal trim fallback")
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--pid-file", type=Path, required=True)
    parser.add_argument("--speedup", type=float, required=True)
    parser.add_argument("--camera-hz", type=int, choices=(40, 50), default=40)
    parser.add_argument("--guided-step", type=int, default=751)
    parser.add_argument("--dock", type=float, nargs=3, default=[40.3207414, 44.4552111, 1294.86])
    parser.add_argument("--delayed", action="store_true")
    parser.add_argument("--fault", choices=FAULTS, default="none")
    args = parser.parse_args()
    with args.pid_file.open("x") as stream:
        stream.write(str(os.getpid()))
    def stop(_signum: int, _frame: FrameType | None) -> None:
        raise SystemExit(143)
    signal.signal(signal.SIGTERM, stop)
    result = serve(args)
    with args.result.open("x") as stream:
        json.dump(result, stream)


if __name__ == "__main__":
    main()
