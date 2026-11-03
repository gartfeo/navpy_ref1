"""One ArduPlane SITL process with a MAVLink connection and an RC input.

Runs inside WSL with the Linux python3 that has pymavlink. Sitl starts its own
arduplane process in its own directory and instance number, apart from the swarm
SITL instances. By default the "pilot" sticks arrive through SITL's simulated
receiver (Transmitter, UDP RC input, AP_RCProtocol_UDP.cpp), so Lua scripts behave
as on the aircraft, not through MAVLink RC overrides that would compete with
script overrides. rc="mavlink" sends the sticks as RC_CHANNELS_OVERRIDE instead,
the path a flight controller on USB needs (board.py); rc=None sends none. The
MAVLink helpers are in vehicle.py, the flight subcommands in sitl_driver.py.
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

from tools.tuning.vehicle import OverrideTransmitter, Vehicle, mavlink2_connection

# On Windows set ARDUPILOT_ROOT to the tree's WSL path (\\wsl.localhost\<distro>\home\<user>\ardupilot)
ARDUPILOT = Path(os.environ.get("ARDUPILOT_ROOT", str(Path.home() / "ardupilot")))
DEFAULT_BINARY = ARDUPILOT / "build" / "sitl" / "bin" / "arduplane"
# the C++ tuner (AP_FlightTune) in the gartfeo ArduPilot fork, feature worktree build
FIRMWARE_BINARY = Path.home() / "fw-worktrees" / "ardupilot-flight-tune" / "build" / "sitl" / "bin" / "arduplane"
DEFAULT_PARAMS = [ARDUPILOT / "Tools" / "autotest" / "models" / "plane.parm",
                  ARDUPILOT / "Tools" / "autotest" / "default_params" / "plane-elevons.parm"]
HERE = Path(__file__).resolve().parent
PERON_LIKE_PARAMS = HERE / "sitl" / "peron_like.parm"
SITL_DEFAULTS = DEFAULT_PARAMS + [PERON_LIKE_PARAMS]
HITL_PARAMS = HERE / "sitl" / "hitl.parm"            # RC over MAVLink: SYSID_MYGCS, log rotation
RC_SINKS = ("udp", "mavlink", None)
# The fork's hardware-in-the-loop backend (libraries/SITL/SIM_HIL.cpp, HIL link contract v2): no
# onboard physics; it trades HIL_ACTUATOR_CONTROLS / HIL_STATE_QUATERNION with the bridge on the
# first SERIALn with protocol 100 (SIM HIL). SITL manages SERIAL0-7 only (the Pixhawk 6X's
# SERIAL8 = USB OTG2 has no SITL counterpart), so the link is on SERIAL5 as in the fork's
# default_params/plane-hil.parm, mapped to a TCP listener (--serial5=tcp:<port>).
HIL_MODEL = "hil"
HIL_SERIAL = 5
# SERIALn_PROTOCOL 100 for that port (hil model only)
HIL_SITL_PARAMS = HERE / "sitl" / "hitl_sitl.parm"
# The HIL port's TCP port, inside the instance's own SITL block 5760+10*I .. +9 (serial0 +0,
# serial1 +2, serial2 +3 are SITL's); +8 is unused there.
HIL_PORT_OFFSET = 8

# Instance 240: serial0 TCP 8160 and RC UDP 7901. A GCS chat slot N runs SITL
# instances 3N..3N+2 (src/gcs/backend/instance_ports.py), so 240 belongs to eval chat
# 80; start() refuses to run if that slot is registered or the ports are in use.
DEFAULT_INSTANCE = 240
WORKDIR_MARKER = ".tuning_sitl_workdir"     # only directories carrying this file are ever wiped
# GCS slot registries: the WSL home one and the Windows one (the GCS backend runs on Windows)
GCS_REGISTRIES = [Path.home() / ".gcs" / "instances.json"] + sorted(Path("/mnt/c/Users").glob("*/.gcs/instances.json"))
HOME_CMAC = "-35.363261,149.165230,584,353"


def prepare_workdir(workdir: Path, wipe: bool) -> None:
    """Create the workdir; wipe it only if this tool created it before (marker file)."""
    if workdir.exists() and any(workdir.iterdir()):
        if not (workdir / WORKDIR_MARKER).exists():
            raise RuntimeError(f"{workdir} is not empty and was not created by this tool")
        if wipe:
            shutil.rmtree(workdir)
    (workdir / "scripts").mkdir(parents=True, exist_ok=True)
    (workdir / WORKDIR_MARKER).write_text("created by tools/tuning/sitl_driver.py\n")


class Transmitter:
    """Send 16 RC channels to SITL's UDP RC input, like a receiver, at 50 Hz."""

    def __init__(self, port: int, host: str = "127.0.0.1") -> None:
        self._addr = (host, port)
        self._pwm = [1500] * 16
        self._pwm[2] = 1000          # throttle low
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "Transmitter":
        self._thread.start()
        return self

    def set(self, channel: int, pwm: int) -> None:
        with self._lock:
            self._pwm[channel - 1] = int(pwm)

    def get(self, channel: int) -> int:
        with self._lock:
            return self._pwm[channel - 1]

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                packet = struct.pack("<16H", *self._pwm)
            self._sock.sendto(packet, self._addr)
            time.sleep(0.02)

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)
        self._sock.close()


class Sitl(Vehicle):
    """One arduplane SITL process plus a MAVLink connection to it.

    defaults: the --defaults files before extra_params; None means SITL_DEFAULTS, the
    tuning runs' set. The hil model also loads HIL_SITL_PARAMS (its link port's protocol), and
    rc="mavlink" HITL_PARAMS so the overrides are accepted.
    """

    def __init__(self, workdir: Path, instance: int = DEFAULT_INSTANCE, model: str = "plane-elevon",
                 speedup: int = 1, extra_params: list[Path] | None = None, home: str = HOME_CMAC,
                 binary: Path = DEFAULT_BINARY, scripts: list[Path] | None = None, wipe: bool = True,
                 rc: str | None = "udp", defaults: list[Path] | None = None) -> None:
        super().__init__(workdir, speedup)
        if rc not in RC_SINKS:
            raise ValueError(f"rc must be one of {RC_SINKS}, not {rc!r}")
        self.instance = instance
        self.model = model
        base = SITL_DEFAULTS if defaults is None else list(defaults)
        base += [HIL_SITL_PARAMS] if model == HIL_MODEL else []
        self.params = base + list(extra_params or []) + ([HITL_PARAMS] if rc == "mavlink" else [])
        self.home = home
        self.binary = Path(binary)
        self.scripts = list(scripts or [])
        self.wipe = wipe
        self.rc = rc
        self.proc: subprocess.Popen | None = None

    @property
    def mavlink_port(self) -> int:
        return 5760 + 10 * self.instance

    @property
    def rc_port(self) -> int:
        return 5501 + 10 * self.instance

    @property
    def hil_port(self) -> int | None:
        """The HIL port's TCP port with the HIL model (the relay's --device tcp:127.0.0.1:<port>), else None."""
        return self.mavlink_port + HIL_PORT_OFFSET if self.model == HIL_MODEL else None

    def _check_free(self) -> None:
        """Refuse to start if a registered GCS slot owns this instance or its ports are taken."""
        for path in GCS_REGISTRIES:
            if not path.exists():
                continue
            try:
                registry = json.loads(path.read_text())
            except (OSError, ValueError) as exc:
                raise RuntimeError(f"cannot read {path}: {exc}") from exc
            for key, entry in (registry.get("instances") or {}).items():
                chat = entry.get("chat") if isinstance(entry, dict) else None
                if isinstance(chat, int) and 3 * chat <= self.instance <= 3 * chat + 2:
                    raise RuntimeError(f"SITL instance {self.instance} belongs to registered GCS slot {key}")
        ports = [(socket.SOCK_STREAM, self.mavlink_port), (socket.SOCK_DGRAM, self.rc_port)]
        ports += [(socket.SOCK_STREAM, self.hil_port)] if self.hil_port is not None else []
        for kind, port in ports:
            with socket.socket(socket.AF_INET, kind) as sock:
                # as SITL does: a closed connection waiting out TIME_WAIT does not block the port,
                # a live listener still does
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    sock.bind(("0.0.0.0", port))
                except OSError as exc:
                    raise RuntimeError(f"port {port} is in use; choose another --instance") from exc

    def command(self) -> list[str]:
        """The arduplane command line. SITL listens on a tcp: serial port above 1000 as given
        (fork libraries/AP_HAL_SITL/UARTDriver.cpp _tcp_start_connection)."""
        cmd = [str(self.binary), "--model", self.model, "--speedup", str(self.speedup),
               "-I", str(self.instance), "--home", self.home]
        cmd += [f"--serial{HIL_SERIAL}=tcp:{self.hil_port}"] if self.hil_port is not None else []
        return cmd + (["--defaults", ",".join(str(p) for p in self.params)] if self.params else [])

    def start(self) -> "Sitl":
        try:
            return self._start()
        except BaseException:
            self.stop()
            raise

    def _start(self) -> "Sitl":
        self._check_free()
        prepare_workdir(self.workdir, self.wipe)
        for script in self.scripts:
            # SITL loads Lua from <workdir>/scripts; strip CRLF like the aircraft deploy step
            text = Path(script).read_text(encoding="utf-8").replace("\r\n", "\n")
            (self.workdir / "scripts" / Path(script).name).write_text(text, encoding="utf-8")
        if self.rc == "udp":
            self.tx = Transmitter(self.rc_port).start()
        log = open(self.workdir / "sitl_stdout.txt", "w")
        self.proc = subprocess.Popen(self.command(), cwd=self.workdir, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.time() + 30
        while True:
            try:
                self.mav = mavlink2_connection(f"tcp:127.0.0.1:{self.mavlink_port}",
                                               source_system=250, autoreconnect=True)
                break
            except OSError:
                if time.time() > deadline or self.proc.poll() is not None:
                    raise RuntimeError(f"SITL did not open port {self.mavlink_port}; see sitl_stdout.txt")
                time.sleep(0.5)
        self.mav.wait_heartbeat(timeout=30)
        self.request_streams()
        if self.rc == "mavlink":
            self.tx = OverrideTransmitter(self).start()
        return self

    def stop(self) -> None:
        if self.tx:
            self.tx.stop()
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
