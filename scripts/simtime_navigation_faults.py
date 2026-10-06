"""Deliberate protocol failures and exact-cause validation for the experiment."""

from __future__ import annotations

import json
from pathlib import Path
import re
import struct
import socket
import threading
import time

FAULTS = ("none", "logger", "boot", "late", "nonfinite", "short", "long",
          "duplicate", "disconnect", "timeout", "expired", "rejected", "stray")
EXPECTED = {
    "logger": ("logger_not_ready", 2), "boot": ("identity_or_version", 5),
    "late": ("apply_step", 5), "nonfinite": ("nonfinite_command", 5),
    "short": ("frame_length", 5), "long": ("frame_length", 5),
    "duplicate": ("identity_or_version", 6), "disconnect": ("disconnected", 5),
    "timeout": ("deadline", 5), "expired": ("disconnected", 5),
    "rejected": ("attitude_rejected", 6), "stray": ("stray_attitude_command", None),
}


def corrupt(reply: bytes, fault: str) -> bytes:
    result = bytearray(reply)
    if fault == "boot":
        result[8] ^= 1
    elif fault == "late":
        struct.pack_into("<Q", result, 56, 7)
    elif fault == "nonfinite":
        struct.pack_into("<f", result, 72, float("nan"))
    elif fault == "short":
        return reply[:-1]
    elif fault == "long":
        return reply + b"x"
    return bytes(result)


class StrayInjector:
    """One test command through the reserved Windows GCS endpoint."""

    def __init__(self, port: int, vehicle: int, case: Path) -> None:
        from pymavlink import mavutil
        self._parser = mavutil.mavlink.MAVLink(None, srcSystem=255)
        self._message = mavutil.mavlink.MAVLink_set_attitude_target_message(
            0, vehicle, 0, 196, [1., 0., 0., 0.], 0., 0., 0., 0.)
        self._vehicle, self._case = vehicle, case
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", port))
        self._socket.settimeout(.1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="owned-stray-test")
        self._thread.start()

    def _run(self) -> None:
        peer = None
        try:
            while not self._stop.is_set():
                try:
                    data, address = self._socket.recvfrom(65535)
                    messages = self._parser.parse_buffer(data) or []
                    if any(m.get_srcSystem() == self._vehicle for m in messages):
                        peer = address
                except socket.timeout:
                    pass
                if peer is not None and (self._case / "injection.json").exists():
                    self._socket.sendto(self._message.pack(self._parser), peer)
                    with (self._case / "stray-sent.json").open("x") as stream:
                        json.dump({"vehicle": self._vehicle, "message": "SET_ATTITUDE_TARGET",
                                   "wall_s": time.monotonic()}, stream)
                    return
        except Exception as error:
            (self._case / "stray-error.txt").write_text(str(error))

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self._socket.close()
        if self._thread.is_alive():
            raise RuntimeError("stray injector did not stop")


def validate_failure(case: Path, fault: str) -> dict:
    log = (case / "supervisor.log").read_text(encoding="utf-8", errors="replace")
    matches = re.findall(r"NAVPY_NAVIGATION_INVALID (\w+) step=(\d+)", log)
    reason, step = EXPECTED[fault]
    if len(matches) != 1 or matches[0][0] != reason:
        raise ValueError(f"wrong rejection: expected {reason}, observed {matches}")
    actual_step = int(matches[0][1])
    if (step is not None and actual_step != step) or (step is None and actual_step < 5):
        raise ValueError("wrong rejection step")
    if log.count("Starting sketch 'ArduPlane'") != 1 or (case / "navpy-navigation.csv").exists():
        raise ValueError("invalid run lifecycle/evidence")
    if fault != "logger":
        injected = json.loads((case / "injection.json").read_text())
        if injected["fault"] != fault or injected["step"] != 5:
            raise ValueError("missing intended fault")
    if fault == "expired":
        failure = json.loads((case / "peer-failure.json").read_text())
        if "receipt liveness" not in failure["error"]:
            raise ValueError("wrong peer liveness rejection")
    if fault == "stray" and not (case / "stray-sent.json").exists():
        raise ValueError("stray message was not sent")
    return {"expected_failure_verified": True, "reason": reason, "step": actual_step}
