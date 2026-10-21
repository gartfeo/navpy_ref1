"""ZR10 SIYI emulator for the HITL final approach (hitl_final_approach.md section 4).

    python -m tools.hitl.zr10_emulator --vehicle-state 127.0.0.1:PORT [--siyi 127.0.0.1:37260]
        [--unreal 127.0.0.1:17212] [--profile siyi_zr10] [--log FILE.jsonl] [effect options]

navpy's real SIYI client talks SIYI to it on UDP (``--siyi``), as to the aircraft's ZR10. The aircraft attitude
comes from the bridge's vehicle_state.v1 datagram (the bridge's ``--vehicle-state-copy``: the same datagram Unreal
gets), never from navpy: it stands in for the real ZR10's own IMU. The emulated
gimbal (zr10_device.py) steps every 20 ms on time.perf_counter and each step's true state goes to the Unreal
gimbal actor as gimbal_state.v1 (``--unreal``). Effects (dead time, lags, scale, deadband, lens speed) are options,
ideal by default; their sources are in zr10_config.SOURCES and in the log header.

One thread, one select loop: all overdue steps run before any datagram is handled, so a command never acts on
time that passed before it arrived.

Fail closed on the aircraft input: nothing is answered before the first vehicle_state. If none arrives for
VEHICLE_STATE_TIMEOUT_S, until one does gimbal_state is not sent, SIYI datagrams are dropped unanswered, no 0x0D
push or 0x25 stream frame goes out, and the gap is counted. A gap, a second bridge session, a board-time step back
(a reboot), a vehicle_state from another system, or a bad, unknown or refused SIYI command (FPV is refused: not
modelled) marks the run invalid (a camera at a mechanical stop is held there, as the real gimbal, and only counted)
in the summary line (ZR10_EMULATOR_SUMMARY_V1) the runner checks.
"""
from __future__ import annotations

import argparse
import json
import select
import socket
import sys
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import IO

from tools.hitl.zr10_config import SOURCES, STEP_S, EmulatorConfig, load_zoom_table
from tools.hitl.zr10_device import Zr10Device, mode_name
from tools.hitl.zr10_gimbal import TrueState
from tools.hitl.zr10_vehicle_input import VehicleInput

GIMBAL_STATE_SCHEMA = "navpy.unreal.gimbal_state.v1"     # NavPyGimbalCameraActor.cpp GimbalStateSchema
DEFAULT_SIYI = "127.0.0.1:37260"      # siyi_sdk.py SIYISDK port 37260; loopback for HITL
DEFAULT_UNREAL = "127.0.0.1:17212"    # NavPyGimbalCameraActor.h GimbalStateListenPort
DEFAULT_DEVICE_ID = 1                 # NavPyGimbalCameraActor.h GimbalStateDeviceId
SUMMARY_TAG = "ZR10_EMULATOR_SUMMARY_V1"
MAX_DATAGRAM = 2048


def parse_endpoint(text: str) -> tuple[str, int]:
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port.isdigit() or not 0 <= int(port) < 65536:
        raise ValueError(f"{text!r}: expected HOST:PORT")
    return socket.gethostbyname(host), int(port)


def encode_gimbal_state(state: TrueState, sys_id: int, device_id: int, session_id: str, sequence: int,
                        sender_time_s: float) -> bytes:
    """One gimbal_state.v1 datagram (fields as ApplyExternalGimbalState reads them)."""
    payload = {
        "schema": GIMBAL_STATE_SCHEMA, "sys_id": sys_id, "device_id": device_id, "session_id": session_id,
        "sequence": sequence,
        "attitude_deg": {"yaw": state.yaw, "pitch": state.pitch, "roll": state.roll},
        "zoom": state.zoom,
        "motion_mode": mode_name(state.motion_mode),
        "roll_stabilize": mode_name(state.motion_mode) != "fpv",
        # time.perf_counter (QPC) at the step, for the U2 stamped history and the logs; Unreal ignores it today.
        "sender_time_s": sender_time_s,
    }
    return json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")


@dataclass
class LoopStats:
    steps: int = 0
    late_steps: int = 0        # steps run more than one step late (a stalled loop catching up)
    max_late_s: float = 0.0
    gimbal_states_sent: int = 0
    gimbal_state_send_errors: int = 0
    siyi_send_errors: int = 0
    siyi_dropped_stale: int = 0   # SIYI datagrams and pushed 0x0D frames dropped while the aircraft attitude was stale


class Zr10Emulator:
    def __init__(self, device: Zr10Device, siyi: tuple[str, int], vehicle_state: tuple[str, int],
                 unreal: tuple[str, int], device_id: int = DEFAULT_DEVICE_ID, log: IO[str] | None = None,
                 clock: Callable[[], float] = time.perf_counter) -> None:
        self.device = device
        self.vehicle = VehicleInput()
        self.stats = LoopStats()
        self.unreal = unreal
        self.device_id = device_id
        self.session_id = uuid.uuid4().hex
        self._sequence = 0
        self._log = log
        self._clock = clock
        self.siyi_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vehicle_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.unreal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.siyi_sock.bind(siyi)
            self.vehicle_sock.bind(vehicle_state)
        except OSError:
            self.close()
            raise
        for sock in (self.siyi_sock, self.vehicle_sock, self.unreal_sock):
            sock.setblocking(False)
        self._write({"event": "start", "config": asdict(device.config), "non_ideal": device.config.non_ideal(),
                     "sources": dict(SOURCES), "siyi": list(self.siyi_sock.getsockname()),
                     "vehicle_state": list(self.vehicle_sock.getsockname()), "unreal": list(unreal),
                     "session_id": self.session_id})

    @property
    def siyi_address(self) -> tuple[str, int]:
        return self.siyi_sock.getsockname()

    @property
    def vehicle_state_address(self) -> tuple[str, int]:
        return self.vehicle_sock.getsockname()

    def serve(self, stop: threading.Event) -> None:
        start_s = self._clock()
        next_step_s = start_s + STEP_S
        sockets = [self.siyi_sock, self.vehicle_sock]
        while not stop.is_set():
            timeout_s = max(0.0, next_step_s - self._clock())
            readable, _, _ = select.select(sockets, [], [], min(timeout_s, 0.05))
            now_s = self._clock()
            while now_s >= next_step_s:
                late_s = now_s - next_step_s
                if late_s > STEP_S:
                    self.stats.late_steps += 1
                self.stats.max_late_s = max(self.stats.max_late_s, late_s)
                self._step(now_s, next_step_s)
                next_step_s += STEP_S
            if readable:
                self.device.advance_to(now_s - start_s)
            if self.vehicle_sock in readable:
                self._drain_vehicle(now_s)
            if self.siyi_sock in readable:
                self._drain_siyi(now_s)
            self._write_records()

    def _step(self, now_s: float, step_s: float) -> None:
        """One plant step, run at ``now_s`` (late when catching up) for the scheduled instant ``step_s``."""
        self.stats.steps += 1
        pushes = self.device.step()
        fresh = self.vehicle.fresh(now_s)
        if not fresh:
            self.stats.siyi_dropped_stale += len(pushes)
            return
        for frame, address in pushes:
            self._send_siyi(frame, address)
        state = self.device.true_state()
        if state is None:
            return
        latest = self.vehicle.latest
        assert latest is not None
        self._sequence += 1
        try:
            self.unreal_sock.sendto(encode_gimbal_state(state, latest.sys_id, self.device_id, self.session_id,
                                                        self._sequence, step_s), self.unreal)
            self.stats.gimbal_states_sent += 1
        except OSError:
            self.stats.gimbal_state_send_errors += 1
        report = self.device.reported()
        self._write({"event": "step", "t": step_s, "run_t": now_s, "device_t": self.device.now_s, "seq": self._sequence,
                     "true": asdict(state), "reported": None if report is None else asdict(report),
                     "board_time_us": latest.board_time_us})

    def _drain_vehicle(self, now_s: float) -> None:
        while True:
            try:
                datagram, _address = self.vehicle_sock.recvfrom(MAX_DATAGRAM)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                continue     # Windows reports an earlier send's ICMP unreachable here
            state = self.vehicle.accept(datagram, now_s)
            if state is not None:
                self.device.set_vehicle_attitude(state.attitude)

    def _drain_siyi(self, now_s: float) -> None:
        fresh = self.vehicle.fresh(now_s)
        while True:
            try:
                datagram, address = self.siyi_sock.recvfrom(MAX_DATAGRAM)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                continue
            if not fresh:
                self.stats.siyi_dropped_stale += 1
                continue
            for reply in self.device.handle_datagram(datagram, address):
                self._send_siyi(reply, address)

    def _send_siyi(self, frame: bytes, address: tuple[str, int]) -> None:
        try:
            self.siyi_sock.sendto(frame, address)
        except OSError:
            self.stats.siyi_send_errors += 1

    def _write_records(self) -> None:
        for record in self.device.drain_records():
            self._write({"event": "command", **asdict(record)})

    def _write(self, record: dict) -> None:
        if self._log is not None:
            self._log.write(json.dumps(record, separators=(",", ":")) + "\n")

    def summary(self) -> dict:
        reasons = self.vehicle.invalid_reasons() + self.device.invalid_reasons()
        return {"valid": not reasons, "invalid_reasons": reasons, "non_ideal": self.device.config.non_ideal(),
                "loop": asdict(self.stats), "vehicle_state": asdict(self.vehicle.stats),
                "siyi": asdict(self.device.counters)}

    def close(self) -> None:
        for sock in (self.siyi_sock, self.vehicle_sock, self.unreal_sock):
            sock.close()


def open_log(path: str) -> IO[str]:
    """Line-buffered: every record is in the file when written, so a stopped process loses none of its steps."""
    return open(path, "w", encoding="utf-8", buffering=1)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.zr10_emulator", description=__doc__.split("\n")[0])
    parser.add_argument("--vehicle-state", required=True, metavar="HOST:PORT",
                        help="where the bridge's vehicle_state.v1 copy arrives (required: the aircraft attitude)")
    parser.add_argument("--siyi", default=DEFAULT_SIYI, metavar="HOST:PORT", help="SIYI UDP address navpy talks to")
    parser.add_argument("--unreal", default=DEFAULT_UNREAL, metavar="HOST:PORT",
                        help="Unreal gimbal actor's gimbal_state listener")
    parser.add_argument("--device-id", type=int, default=DEFAULT_DEVICE_ID, help="gimbal_state device_id")
    parser.add_argument("--profile", default="siyi_zr10", help="vision profile holding camera.zoom_calibration")
    parser.add_argument("--log", default=None, metavar="FILE", help="JSONL log: config, every command, every step")
    for item in EmulatorConfig.__dataclass_fields__.values():
        parser.add_argument("--" + item.name.replace("_", "-"), type=float if item.type == "float" else int,
                            default=item.default, help=SOURCES[item.name])
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = EmulatorConfig(**{name: getattr(args, name) for name in EmulatorConfig.__dataclass_fields__})
    device = Zr10Device(config, load_zoom_table(args.profile))
    log = open_log(args.log) if args.log else None
    emulator = Zr10Emulator(device, parse_endpoint(args.siyi), parse_endpoint(args.vehicle_state),
                            parse_endpoint(args.unreal), args.device_id, log)
    stop = threading.Event()
    print(f"ZR10 emulator: SIYI {emulator.siyi_address}, vehicle_state {emulator.vehicle_state_address}, "
          f"Unreal {args.unreal}, non-ideal {config.non_ideal() or 'none'}", flush=True)
    try:
        emulator.serve(stop)
    except KeyboardInterrupt:
        pass
    finally:
        summary = emulator.summary()
        emulator._write({"event": "summary", **summary})
        emulator.close()
        if log is not None:
            log.close()
        print(f"{SUMMARY_TAG} {json.dumps(summary, separators=(',', ':'))}", flush=True)
    return 0 if summary["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
