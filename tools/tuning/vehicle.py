"""A vehicle the tuning flights drive over MAVLink, and the MAVLink RC sink.

Vehicle holds the MAVLink helpers that need only a connection, an RC sink and the
time scale: Sitl (sitl_sim.py) adds an ArduPlane SITL process, Board (board.py) a
flight controller running simulation firmware. OverrideTransmitter is the RC sink
for a vehicle without SITL's UDP receiver: the Transmitter interface sent as
RC_CHANNELS_OVERRIDE.
"""
from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Callable

from pymavlink import mavutil

MODE = {"MANUAL": 0, "FBWA": 5, "FBWB": 6, "CRUISE": 7, "AUTOTUNE": 8, "AUTO": 10,
        "RTL": 11, "LOITER": 12, "TAKEOFF": 13}

RC_RATE_HZ = 50          # a receiver's frame rate; ArduPilot drops an override RC_OVERRIDE_TIME after the last
RC_CHANNELS = 16         # sent channels; RC_CHANNELS_OVERRIDE also has 17 and 18, sent as 0 (ignore)
# A stick position is within these; 0 on channels 1-8 would release the override and 65535
# means "ignore" (GCS_Common.cpp:4018-4030), so neither may be sent as a stick value.
RC_PWM_MIN_US = 800
RC_PWM_MAX_US = 2200


def mavlink2_connection(device: str, **kwargs: object) -> mavutil.mavfile:
    """Open device speaking MAVLink 2, with writes serialised by a lock.

    pymavlink picks the wire version from MAVLINK20 when it loads a dialect, and
    RC_CHANNELS_OVERRIDE carries channels 9-18 only in MAVLink 2 (channel 9 is the tuning
    switch). The RC thread and the main thread both write; pyserial on Windows shares one
    OVERLAPPED write structure per port (serial/serialwin32.py:69-71, 314), so writes must
    not overlap.
    """
    os.environ["MAVLINK20"] = "1"
    mavutil.set_dialect(mavutil.current_dialect)
    conn = mavutil.mavlink_connection(device, **kwargs)
    write, lock = conn.write, threading.Lock()

    def locked_write(buf: bytes) -> object:
        with lock:
            return write(buf)

    conn.write = locked_write      # pymavlink sends through self.file.write
    return conn


class Vehicle:
    """MAVLink helpers shared by Sitl and Board: a connection (mav), an RC sink (tx) and the
    time scale (speedup: wall seconds are simulated seconds divided by it)."""

    label = "sitl"               # prefix of the vehicle's STATUSTEXT lines on the console

    def __init__(self, workdir: Path, speedup: int) -> None:
        self.workdir = Path(workdir).expanduser()
        self.speedup = speedup
        self.mav = None
        self.tx = None
        self.messages: list[str] = []

    def request_streams(self, rate_hz: int = 10) -> None:
        self.mav.mav.request_data_stream_send(self.mav.target_system, self.mav.target_component,
                                              mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1)

    def pump(self, timeout: float = 0.05) -> mavutil.mavlink.MAVLink_message | None:
        """Read pending messages; keep STATUSTEXT for the test record. Returns the last message."""
        last = None
        while True:
            m = self.mav.recv_match(blocking=True, timeout=timeout)
            if m is None:
                return last
            if m.get_type() == "STATUSTEXT":
                self.messages.append(m.text)
                print(f"[{self.label}] {m.text}", flush=True)
            last = m
            timeout = 0

    def wait_message(self, mtype: str, timeout: float = 10.0,
                     cond: Callable[[mavutil.mavlink.MAVLink_message], object] | None = None
                     ) -> mavutil.mavlink.MAVLink_message | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.mav.recv_match(type=[mtype, "STATUSTEXT"], blocking=True, timeout=0.5)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                self.messages.append(m.text)
                print(f"[{self.label}] {m.text}", flush=True)
                continue
            if cond is None or cond(m):
                return m
        return None

    def set_param(self, name: str, value: float, retries: int = 5) -> None:
        for _ in range(retries):
            self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component,
                                        name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            m = self.wait_message("PARAM_VALUE", 3, lambda p: p.param_id == name)
            if m is not None and abs(m.param_value - float(value)) < 1e-4 * max(1.0, abs(float(value))):
                return
        raise RuntimeError(f"could not set {name}={value}")

    def get_param(self, name: str) -> float:
        self.mav.mav.param_request_read_send(self.mav.target_system, self.mav.target_component,
                                             name.encode(), -1)
        m = self.wait_message("PARAM_VALUE", 5, lambda p: p.param_id == name)
        if m is None:
            raise RuntimeError(f"no value for {name}")
        return m.param_value

    def set_mode(self, mode: str) -> None:
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                       MODE[mode], 0, 0, 0, 0, 0)
        hb = self.wait_message("HEARTBEAT", 10, lambda h: h.custom_mode == MODE[mode])
        if hb is None:
            raise RuntimeError(f"mode {mode} not reached")

    def mode(self) -> int:
        hb = self.wait_message("HEARTBEAT", 5)
        return hb.custom_mode if hb else -1

    def arm(self, timeout: float = 120) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                           1, 0, 0, 0, 0, 0, 0)
            hb = self.wait_message("HEARTBEAT", 3,
                                   lambda h: h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if hb is not None:
                return
            time.sleep(2)
        raise RuntimeError(f"could not arm; see the [{self.label}] messages above")

    def wait_ready(self, timeout: float = 120) -> None:
        """Wait until the EKF has a position (GPS lock) so arming checks can pass."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.wait_message("GPS_RAW_INT", 2)
            if m is not None and m.fix_type >= 3:
                ekf = self.wait_message("EKF_STATUS_REPORT", 3)
                if ekf is not None and ekf.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS:
                    return
        raise RuntimeError(f"{self.label.upper()} not ready (no GPS/EKF position)")

    def state(self) -> dict:
        """Latest attitude, airspeed and height above home."""
        att = self.wait_message("ATTITUDE", 2)
        vfr = self.wait_message("VFR_HUD", 2)
        pos = self.wait_message("GLOBAL_POSITION_INT", 2)
        return {"roll": att.roll if att else None, "pitch": att.pitch if att else None,
                "airspeed": vfr.airspeed if vfr else None,
                "alt": pos.relative_alt / 1000.0 if pos else None}

    def fly_for(self, seconds: float) -> None:
        end = time.time() + seconds / max(1, self.speedup)
        while time.time() < end:
            self.pump(0.05)

    def logs(self) -> list[Path]:
        return sorted((self.workdir / "logs").glob("*.BIN"))

    def report(self) -> dict:
        """Facts about the vehicle's setup for the run's JSON; SITL has none."""
        return {}


class OverrideTransmitter:
    """The Transmitter interface (start/set/get/stop) sent as RC_CHANNELS_OVERRIDE at RC_RATE_HZ.

    ArduPlane takes overrides only from SYSID_MYGCS (GCS_Common.cpp:3990); sitl/hitl.parm sets
    it to the tools' system id. The connection is looked up on every tick: a reconnect replaces
    vehicle.mav, and pymavlink replaces conn.mav when the wire version changes
    (mavutil.py:303-313). Send errors while the link is down are counted, not raised. If the
    overrides stop, the vehicle's RC failsafe follows, as with a lost transmitter.
    """

    def __init__(self, vehicle: Vehicle, rate_hz: float = RC_RATE_HZ) -> None:
        self._vehicle = vehicle
        self._period = 1.0 / rate_hz
        self._pwm = [1500] * RC_CHANNELS
        self._pwm[2] = 1000          # throttle low
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.send_errors = 0

    def start(self) -> OverrideTransmitter:
        self._thread.start()
        return self

    def set(self, channel: int, pwm: int) -> None:
        if not RC_PWM_MIN_US <= int(pwm) <= RC_PWM_MAX_US:
            raise ValueError(f"RC channel {channel}: {pwm} us is not a stick position")
        with self._lock:
            self._pwm[channel - 1] = int(pwm)

    def get(self, channel: int) -> int:
        with self._lock:
            return self._pwm[channel - 1]

    def send_once(self) -> None:
        conn = self._vehicle.mav
        if conn is None:             # reconnecting
            return
        with self._lock:
            pwm = list(self._pwm)
        try:
            conn.mav.rc_channels_override_send(conn.target_system, conn.target_component, *pwm, 0, 0)
        except OSError:
            self.send_errors += 1

    def _run(self) -> None:
        while True:
            self.send_once()
            if self._stop.wait(self._period):
                return

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1)
