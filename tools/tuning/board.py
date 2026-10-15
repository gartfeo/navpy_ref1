"""A flight controller running ArduPlane simulation firmware, flown like SITL over MAVLink.

start() reproduces SITL's "fresh parameters from the --defaults files": read and check every
parameter file before the board is touched, back up every parameter, refuse unless this is
simulation firmware with real outputs off and the vehicle is disarmed, erase only when asked
(and only after the verified backup), load the parameter files and reboot until a fresh boot
reports them all, start the HITL processes (the bridge, then the relay: hitl_procs.py) and
reboot once more so that the board boots with the bridge answering (hitl_boot.py), record the
log list and start the RC overrides. stop() force-disarms what
start() configured, stops the HITL processes, downloads the new logs and closes. report()
adds, for HITL, the link-health gate (hitl_gate.link_health). With a mirror (mp_mirror.py),
Mission Planner watches the link through every reconnect.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Callable

from pymavlink import mavutil

from tools.tuning.hitl_boot import boot_on_bridge
from tools.tuning.hitl_gate import link_health
from tools.tuning.hitl_logs import download_logs, list_logs, new_log_entries, timing_summary
from tools.tuning.hitl_params import (ParamFile, Refused, changes, check_disarmed, check_loaded, check_sim_firmware,
                                      check_sim_rate, fetch_params, merge_param_files, origin_params, write_backup)
from tools.tuning.hitl_procs import HitlProcesses
from tools.tuning.mp_mirror import MavlinkMirror
from tools.tuning.vehicle import OverrideTransmitter, Vehicle, mavlink2_connection

DEFAULT_BAUD = 115200          # USB CDC ignores it; a UART link needs the port's SERIALn_BAUD
SOURCE_SYSTEM = 250            # the tools' system id; sitl/hitl.parm makes it SYSID_MYGCS
ACK_NO_TARGET = 0              # COMMAND_ACK.target_system when absent (a MAVLink 2 extension field)
CONNECT_TIMEOUT_S = 60         # reboot through the bootloader and USB re-enumeration
OPEN_RETRY_S = 0.5             # the OS gives no event when the port comes back, so opening is retried
HEARTBEAT_TIMEOUT_S = 5.0
ACK_TIMEOUT_S = 5.0
DISARM_TIMEOUT_S = 10.0
MAX_LOAD_ROUNDS = 4            # rounds of: set differing values, reboot, list again
FORCE_DISARM_MAGIC = 21196     # GCS.h:745 magic_force_arm_disarm_value: disarm even when flying
BACKUP_NAME = "params_before.parm"
ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def is_serial(device: str) -> bool:
    return not device.startswith(("tcp:", "udp:", "tcpin:", "udpin:", "udpout:"))


class Board(Vehicle):
    """A flight controller on a serial port (COMn, /dev/ttyACMn) or tcp:host:port."""

    label = "board"

    def __init__(self, device: str, workdir: Path, param_files: list[ParamFile], origin: str = "",
                 erase: bool = False, hitl: HitlProcesses | None = None, log_fetch: bool = True,
                 baud: int = DEFAULT_BAUD, connect: Callable[..., object] = mavlink2_connection,
                 mirror: MavlinkMirror | None = None) -> None:
        super().__init__(workdir, speedup=1)      # simulation on hardware runs in real time
        self.device = device
        self.param_files = list(param_files)
        self.origin = origin
        self.erase = erase
        self.hitl = hitl
        self.log_fetch = log_fetch
        self.mirror = mirror
        self._open = lambda: connect(device, baud=baud, source_system=SOURCE_SYSTEM)
        self.log_baseline: dict[int, tuple[int, int]] | None = None
        self.started = False
        self.record: dict = {"device": device, "erased": False}

    # ---- link ----------------------------------------------------------------
    def connect(self, since: float | None = None) -> object:
        """(Re)open the link and wait for a heartbeat; with `since`, only from a boot after that time."""
        self.close_link()
        deadline = time.time() + CONNECT_TIMEOUT_S
        while time.time() < deadline:
            hb = self._try_link(since)
            if hb is not None:
                return hb
        raise RuntimeError(f"{self.device}: no {'fresh boot' if since else 'heartbeat'} within {CONNECT_TIMEOUT_S} s")

    def _try_link(self, since: float | None) -> object:
        """One attempt: open, wait for a heartbeat, check the boot time. None if it failed."""
        try:
            self.mav = self._open()
            if self.mirror is not None:
                self.mirror.attach(self.mav)      # Mission Planner sees this boot's link too
            hb = self.mav.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
            if hb is not None and (since is None or self._uptime_s() <= time.time() - since):
                return hb
        except OSError:                  # the port vanished (reboot) or is not back yet
            time.sleep(OPEN_RETRY_S)
        try:
            self.close_link()
        except OSError:
            self.mav = None
        return None

    def _uptime_s(self) -> float:
        self._command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME)
        m = self.wait_message("SYSTEM_TIME", ACK_TIMEOUT_S)
        return m.time_boot_ms / 1000.0 if m is not None else math.inf

    def close_link(self) -> None:
        if self.mirror is not None:
            self.mirror.detach()                  # no Mission Planner packet goes to a closing port
        if self.mav is not None:
            conn, self.mav = self.mav, None       # no RC override goes to it either
            conn.close()

    def _command(self, command: int, *params: float) -> None:
        p = list(params) + [0] * (7 - len(params))
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, command, 0, *p)

    def _ack(self, command: int) -> object:
        """The board's answer to our command: an acknowledgement addressed to another system (a
        mirrored Mission Planner, 255) is not ours; MAVLink 1 acknowledgements carry no target (0)."""
        return self.wait_message("COMMAND_ACK", ACK_TIMEOUT_S, lambda a: a.command == command and getattr(
            a, "target_system", ACK_NO_TARGET) in (ACK_NO_TARGET, SOURCE_SYSTEM))

    def _require_disarmed(self, action: str) -> None:
        hb = self.wait_message("HEARTBEAT", HEARTBEAT_TIMEOUT_S)
        if hb is None or hb.base_mode & ARMED:
            raise Refused(f"no heartbeat or armed: not {action}")

    def reboot(self) -> None:
        self._require_disarmed("rebooting")
        sent = time.time()
        self._command(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
        try:
            ack = self._ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)   # may be lost as the link drops
        except OSError:          # a USB board's COM port vanishes as it reboots, before or after the ack
            ack = None
        if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise Refused(f"reboot refused (result {ack.result})")
        self.connect(since=sent)

    def load(self, desired: dict[str, float]) -> dict[str, float]:
        """Set the differing values and reboot until a fresh boot lists them all; an enable
        parameter shows its group only once set, some only after a boot. Returns the last list."""
        for round_no in range(1, MAX_LOAD_ROUNDS + 1):
            current = fetch_params(self)
            todo = changes(desired, current)
            if round_no > 1 and not todo:
                return current
            for name, value in todo.items():
                try:
                    self.set_param(name, value)
                except RuntimeError:
                    self.record.setdefault("refused_sets", []).append(name)
            self.reboot()
            self.record["load_rounds"] = round_no
        return fetch_params(self)

    def pump(self, timeout: float = 0.05) -> object:
        self._check_hitl()
        return super().pump(timeout)

    def wait_message(self, mtype: str | tuple[str, ...], timeout: float = 10.0, cond: Callable[..., object] | None = None) -> object:
        self._check_hitl()
        return super().wait_message(mtype, timeout, cond)

    # ---- run -----------------------------------------------------------------
    def start(self) -> Board:
        try:
            return self._start()
        except BaseException:
            self.stop()
            raise

    def _start(self) -> Board:
        if self.workdir.exists() and any(self.workdir.iterdir()):
            raise Refused(f"{self.workdir} is not empty: board run folders keep the parameter backup "
                          "and are never wiped; choose a new --workdir")
        desired, required = self._desired()          # a bad file must stop the run before the erase
        self.workdir.mkdir(parents=True, exist_ok=True)
        check_disarmed(self.connect())
        params = fetch_params(self)
        self.record["backup"] = str(write_backup(self.workdir / BACKUP_NAME, params, self.device))
        check_sim_firmware(params)
        if self.erase:
            self._erase()
        current = self.load(desired)
        self.record["absent_optional"] = check_loaded(desired, required, current)
        check_sim_firmware(current)          # no parameter file may enable a real output
        if self.hitl is None and is_serial(self.device):
            check_sim_rate(current)
        if self.hitl is not None:
            self.record["hitl"] = self.hitl.start()
            self.record["hil_boot_attempts"] = boot_on_bridge(self)
        self.request_streams()
        self.log_baseline = list_logs(self)
        self.tx = OverrideTransmitter(self).start()
        self.started = True
        return self

    def _desired(self) -> tuple[dict[str, float], set[str]]:
        """Every parameter file read and checked, plus SIM_OPOS_* from the origin: the values to
        load and the names the board must have."""
        try:
            values, required = merge_param_files(self.param_files)
        except FileNotFoundError as exc:
            raise Refused(f"parameter file {exc.filename} not found; on Windows set ARDUPILOT_ROOT to the ArduPilot "
                          r"tree as WSL shares it, e.g. \\wsl.localhost\<distro>\home\<user>\ardupilot") from None
        origin = origin_params(self.origin) if self.origin else {}
        return {**values, **origin}, required | set(origin)

    def _erase(self) -> None:
        if "backup" not in self.record:
            raise Refused("no verified parameter backup: not erasing")
        self._require_disarmed("erasing")        # the erase command itself has no armed check
        self._command(mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 2)
        ack = self._ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE)
        if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"parameter erase not accepted: {ack}")
        self.record["erased"] = True
        self.reboot()          # erase_all clears storage; the values in RAM stay until a boot

    def _check_hitl(self) -> None:
        if self.started and self.hitl is not None:
            self.hitl.check()

    def stop(self) -> None:
        """Force-disarm what start() configured, stop the HITL processes, fetch the logs, close. Errors are
        recorded, not raised: this runs in the drivers' finally blocks."""
        steps = [self._disarm, self._stop_hitl, self._fetch_logs] if self.started else [self._stop_hitl]
        self.started = False      # from here on the HITL processes end
        for step in steps:
            try:
                step()
            except Exception as exc:     # noqa: BLE001 - every step runs; the record keeps the error
                self.record.setdefault("stop_errors", []).append(f"{step.__name__}: {exc}")
                print(f"[board] {step.__name__} failed: {exc}", flush=True)
        if self.tx is not None:
            self.tx.stop()
        self.close_link()
        if self.mirror is not None:
            self.record["mp_mirror"] = self.mirror.close()

    def _disarm(self) -> None:
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, FORCE_DISARM_MAGIC)
        if self.wait_message("HEARTBEAT", DISARM_TIMEOUT_S, lambda h: not h.base_mode & ARMED) is None:
            raise RuntimeError("still armed")

    def _stop_hitl(self) -> None:
        if self.hitl is not None:
            self.record.setdefault("hitl", {}).update(self.hitl.stop())

    def _fetch_logs(self) -> None:
        if not self.log_fetch:
            after = list_logs(self)
            entries = {i: after[i] for i in new_log_entries(self.log_baseline, after)}
            self.record["logs_on_card"] = {i: {"bytes": s, "time_utc": t} for i, (s, t) in entries.items()}
            print(f"[board] copy these logs from the SD card (APM/LOGS) into {self.workdir / 'logs'}: "
                  f"{self.record['logs_on_card']}", flush=True)
            return
        self.record["log_entries"] = [p.name for p in download_logs(self, self.log_baseline, self.workdir / "logs")]

    def report(self) -> dict:
        """The run's setup record, the tuner timing per log and, for HITL, the link-health gate."""
        timing = {}
        for path in self.logs():
            try:
                timing[path.name] = timing_summary(path)
            except Exception as exc:     # noqa: BLE001 - a broken log must not lose the run's record
                timing[path.name] = {"error": str(exc)}
        report = {**self.record, "timing": timing}
        if self.hitl is not None:
            report["link_health"] = link_health(self.logs(), self.hitl.link_csv)
        return report

