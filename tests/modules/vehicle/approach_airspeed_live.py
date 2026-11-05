"""The live part of test_approach_airspeed.py: the eligible source against pinned SITL (step 20).

Chain: the SITL board's serial0 (``endpoint``, tcp:host:port) -> the PC router in-process (tools/tuning/mav_router_core.py)
with two clients as in the approach runner: ``runner`` (pymavlink: 10 Hz streams, parameter changes) and ``navpy``
as a reader, where navpy's real ``VehicleMav`` and its inbound router feed the source through ``attach``. Each state
gets a fresh source that must be admitted first (60 healthy seconds at 18 Hz or more), then the fault:
- unhealthy: ARSPD_OFFSET 0 with ARSPD_SKIP_CAL 0 (AP_Airspeed::healthy refuses a zero offset);
- not used: ARSPD_USE 0;
- stale status: SYS_STATUS withheld from the source for 1 s on navpy's side (the router keeps every client's streams,
  so no client can stop SYS_STATUS at the board), AIRSPEED still flowing;
and a plausible VFR_HUD airspeed is read beside a pending source. A law reader thread reads the source at 20 Hz
throughout and counts reads that return a value after the revoke (must be 0).
"""
from __future__ import annotations

import math
import socket
import threading
import time

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_ONBOARD_COMPUTER, MAV_TYPE_GCS

from navpy.plugins.vehicle.ardupilot.airspeed_attach import attach
from navpy.plugins.vehicle.ardupilot.airspeed_profile import ADMITTED, ADMIT_WINDOW_S, SIMULATED_PROFILES
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav
from tools.tuning.hitl_bridge.transport import TcpTransport
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mp_mirror import LOOPBACK

PROFILE = SIMULATED_PROFILES["pc-sitl"]
ADMIT_WAIT_S = ADMIT_WINDOW_S + 15.0        # the window plus room for a restart after boot
REVOKE_WAIT_S = 10.0                         # a fault shows within a few packets; this bounds a failing run
READ_PERIOD_S = 0.05                         # the law's 20 Hz read


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((LOOPBACK, 0))
        return s.getsockname()[1]


class _Withholding:
    """navpy's vehicle with SYS_STATUS withheld from the source while ``hold`` is set (the stale-status state)."""

    def __init__(self, vehicle: VehicleMav) -> None:
        self._vehicle = vehicle
        self.target_system = vehicle.target_system
        self.hold = threading.Event()

    def on_message(self, name, callback):
        if name != "SYS_STATUS":
            return self._vehicle.on_message(name, callback)
        return self._vehicle.on_message(name, lambda m: None if self.hold.is_set() else callback(m))

    def send_command_long(self, command, **parameters):
        return self._vehicle.send_command_long(command, **parameters)

    def get_parameter_fresh(self, name, timeout=2.0):
        return self._vehicle.get_parameter_fresh(name, timeout=timeout)

    def exclusive_send(self):
        return self._vehicle.exclusive_send()


class _LawReader:
    def __init__(self, source) -> None:
        self.source, self.values, self.after_revoke = source, 0, 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.wait(READ_PERIOD_S):
            value = self.source.read(time.perf_counter())
            if value is not None:
                self.values += 1
                if self.source.revoked:
                    self.after_revoke += 1

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(2.0)


def _wait(predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.1)
    return predicate()


def _set(runner, name: str, value: float) -> None:
    """``runner`` is the pymavlink connection (the runner's router client)."""
    for _ in range(5):
        runner.param_set_send(name, value)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            msg = runner.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name and abs(msg.param_value - value) < 1e-6:
                return
    raise RuntimeError(f"{name} not set to {value}")


def _state(navpy, events, fault=None, restore=None) -> dict:
    mark = len(events)
    source = attach(navpy, PROFILE, events.append)
    reader = _LawReader(source)
    try:
        admitted = _wait(lambda: source.state == ADMITTED, ADMIT_WAIT_S)
        result = {"admitted": admitted, "counts_at_admit": dict(source.counts)}
        if admitted and fault is not None:
            fault()
            result["revoked"] = _wait(lambda: source.revoked is not None, REVOKE_WAIT_S)
            result["reason"] = source.revoked
            time.sleep(1.0)                         # the reader keeps reading after the revoke
        result["reads_after_revoke"] = reader.after_revoke
        result["law_reads"] = reader.values
    finally:
        reader.stop()
    if restore is not None:
        restore()
    result["events"] = [e for e in events[mark:] if e["event"] != "window_restart"][:12]
    return result


def run_live_states(endpoint: str) -> dict:
    _, host, port = endpoint.split(":")
    runner_port, navpy_port = _free_udp_port(), _free_udp_port()
    router = MavRouter(TcpTransport(host, int(port)), [Client("runner", runner_port, False),
                                                        Client("navpy", navpy_port, False, reader=True)],
                       announce=lambda line: None).start()
    runner = mavutil.mavlink_connection(f"udpin:{LOOPBACK}:{runner_port}", source_system=250, source_component=190)
    vehicle = None
    events: list[dict] = []
    try:
        runner.wait_heartbeat(timeout=30)
        runner.mav.request_data_stream_send(runner.target_system, runner.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        vehicle = VehicleMav(f"udpin:{LOOPBACK}:{navpy_port}", target_system=runner.target_system,
                             skip_mission_download=True, mav_type=MAV_TYPE_GCS, mav_comp_id=MAV_COMP_ID_ONBOARD_COMPUTER)
        navpy = _Withholding(vehicle)
        pending = attach(navpy, PROFILE, events.append)
        time.sleep(2.0)
        hud = vehicle.air_speed
        result: dict = {"hud": {"hud_airspeed": None if hud is None or not math.isfinite(hud) else float(hud),
                                "law_read": pending.read(time.perf_counter()), "state": pending.state}}
        result["params_at_start"] = {name: vehicle.get_parameter_fresh(name, timeout=2.0)
                                     for name in ("ARSPD_TYPE", "ARSPD_USE", "ARSPD_PRIMARY", "ARSPD_OFFSET",
                                                  "ARSPD_SKIP_CAL")}
        offset = result["params_at_start"]["ARSPD_OFFSET"] or 1.0
        result["unhealthy"] = _state(
            navpy, events,
            fault=lambda: (_set(runner, "ARSPD_SKIP_CAL", 0), _set(runner, "ARSPD_OFFSET", 0.0)),
            restore=lambda: _set(runner, "ARSPD_OFFSET", offset))
        result["not_used"] = _state(navpy, events, fault=lambda: _set(runner, "ARSPD_USE", 0),
                                    restore=lambda: _set(runner, "ARSPD_USE", 1))
        result["stale_status"] = _state(navpy, events, fault=lambda: (navpy.hold.set(), time.sleep(1.0)),
                                        restore=navpy.hold.clear)
        result["admitted"] = all(result[k]["admitted"] for k in ("unhealthy", "not_used", "stale_status"))
        result["router"] = {k: v for k, v in router.stats.items() if k.startswith(("to_", "from_", "claim"))}
        return result
    finally:
        if vehicle is not None:
            vehicle.close()
        runner.close()
        router.close()
