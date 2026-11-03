"""The pc-sitl chain of one approach run (hitl_final_approach.md section 1, stage 1 step 13), as ProcSpecs.

Most roles run in WSL beside the runner: the SITL backing (hitl.py sitl-backing, HIL model), the router on its
serial0, the aas-hitl publisher (MediaMTX is its child) and navpy. The bridge is started by the board setup
(hitl_procs.py, direct: it opens the backing's HIL port). Unreal and the ZR10 emulator run on Windows and are reached
on the Windows host's address; Unreal sends its frames to WSL's own address (the direct route: todo 186). The
emulator runs there, as in steps 8 and 9, because Unreal joins its gimbal samples to frames on the PC's one clock
(QPC, hitl_final_approach.md "One time axis"), which a WSL process does not read.
"""
from __future__ import annotations

import json
import os
import re
import shlex
import socket
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Callable, Optional

from navpy.app.runtime_ready import NAVPY_RUNTIME_READY_MARKER
from navpy.app.runtime_stop import NAVPY_STOP_REQUEST
from tools.tuning import approach_handles as handles
from tools.tuning.approach_procs import STOP_EOF, STOP_SIGINT, ProcSpec, RunnerError

SITL_PORT = 8160                 # sitl_sim DEFAULT_INSTANCE 240: serial0 (the router's board) and SERIAL5 (HIL) + 8
HIL_PORT = 8168
RUNNER_PORT, NAVPY_PORT, CONTROL_PORT = 9641, 9642, 9643     # router clients and barrier (mav_router.py usage)
POSE_PORT, VEHICLE_STATE_PORT, GIMBAL_PORT = 17110, 17113, 17212
SIYI_PORT = 37260
EMULATOR_PORTS = (VEHICLE_STATE_PORT, SIYI_PORT)    # the emulator's Windows binds
EMULATOR_PID_LINE = r"^ZR10 emulator: .*, pid (\d+)"    # its ready line names its own Windows pid
NATIVE_PORT, RTSP_PORT, RTP_PORT, MEDIA_API_PORT = 18110, 18554, 18556, 9997
# The frame consumers' Linux nice value. Section 8 puts the bridge and router above them; WSL lets a user only lower a
# priority (RLIMIT_NICE 0), so navpy and the publisher start lowered instead. Fit run 8 (2026-11-03): with frames
# flowing, the bridge's p99 turnaround rose from 0.5 to 232 ms while navpy ran, and the board lost its state.
CONSUMER_NICE = ("nice", "-n", "10")
UNREAL_ALT_OFFSET_M = "23.5"     # hitl_unreal_camera.md "Running it" step 2
VISION_PROFILE = "siyi_zr10_hitl_encoded"
SITL_SYSTEM_ID = 1
# Start bounds (my values): SITL boots in seconds, Unreal loads its level in about a minute (step 9: under 180 s), the
# publisher starts MediaMTX and CUDA, navpy opens its link, profile and stream.
READY_TIMEOUT_S = {"backing": 120.0, "router": 30.0, "emulator": 30.0, "publisher": 300.0, "unreal": 240.0,
                   "navpy": 180.0}
UNREAL_EXE = "C:\\Program Files\\Epic Games\\UE_5.7\\Engine\\Binaries\\Win64\\UnrealEditor.exe"
UNREAL_LEVEL = "/Game/AAS/Levels/Yeghvard_Default"
UNREAL_IMAGE = "UnrealEditor.exe"


@dataclass(frozen=True)
class ChainPaths:
    run_dir: Path                # WSL path of the run folder (private)
    repo: Path                   # WSL path of this checkout
    python: str                  # the WSL venv python navpy and the tools run with (python -s)
    aas_hitl: Path               # WSL path of the aas-hitl clone the publisher runs from
    publisher_profile: Path      # the publisher's JSON profile template (native_host is set per run)
    uproject: Path               # WSL path of the Unreal project (/mnt/c/...); Unreal gets its Windows form
    wsl_ip: str                  # WSL's own address, Unreal's frame target
    windows_ip: str              # the Windows host's address seen from WSL
    windows_python: str = ""     # WSL path of the Windows Python the emulator runs with (/mnt/c/.../python.exe)
    unreal_affinity: Optional[int] = None


def win_path(path: Path, distro: Optional[str] = None) -> str:
    """Unreal's command line: /mnt/c/a/b -> C:\\a\\b; another absolute WSL path -> \\\\wsl.localhost\\DISTRO\\..."""
    parts = Path(path).as_posix().split("/")
    if len(parts) >= 3 and parts[1] == "mnt" and len(parts[2]) == 1:
        return parts[2].upper() + ":\\" + "\\".join(parts[3:])
    distro = distro or os.environ.get("WSL_DISTRO_NAME")
    if parts[0] != "" or not distro:
        raise ValueError(f"{path} is neither under /mnt/<drive> nor an absolute path in a named WSL distro")
    return "\\\\wsl.localhost\\" + distro + "\\" + "\\".join(parts[1:])


def port_free(protocol: str, port: int) -> bool:
    """No other socket owns the port. On Linux SO_REUSEADDR lets a TCP bind pass over the TIME_WAIT a run just
    stopped leaves for 60 s when its listener set it too (MediaMTX's RTSP port: Go sets it), while a listener still
    refuses it; on Windows it would let the bind take a listener's port, so it is set only off Windows."""
    kind = socket.SOCK_STREAM if protocol == "tcp" else socket.SOCK_DGRAM
    with socket.socket(socket.AF_INET, kind) as sock:
        if kind == socket.SOCK_STREAM and os.name != "nt":
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", port))
        except OSError:
            return False
    return True


def tool(paths: ChainPaths, *args: str) -> tuple[str, ...]:
    return (paths.python, "-s", "-m", *args)


def backing(paths: ChainPaths, origin: str, binary: str) -> ProcSpec:
    return ProcSpec("backing", tool(paths, "tools.tuning.hitl", "sitl-backing", "--model", "hil", f"--origin={origin}",
                                    "--binary", binary, "--workdir", str(paths.run_dir / "backing")),
                    r"^\[backing\] SITL instance", READY_TIMEOUT_S["backing"], STOP_SIGINT,
                    ports=(("tcp", SITL_PORT), ("tcp", HIL_PORT)), cwd=str(paths.repo))


def router(paths: ChainPaths) -> ProcSpec:
    return ProcSpec("router", tool(paths, "tools.tuning.mav_router", "--board", f"tcp:127.0.0.1:{SITL_PORT}",
                                   "--client", f"runner={RUNNER_PORT}", "--client", f"navpy={NAVPY_PORT}",
                                   "--control", str(CONTROL_PORT),
                                   "--write-log", str(paths.run_dir / "router_writes.csv"),
                                   "--stats", str(paths.run_dir / "router_stats.json"), "--exit-on-stdin-eof"),
                    r"^\[router\] tcp:\S+ <-> ", READY_TIMEOUT_S["router"], STOP_EOF,
                    ports=(("udp", RUNNER_PORT), ("udp", NAVPY_PORT), ("udp", CONTROL_PORT)), cwd=str(paths.repo))


def bridge_template(model_dir: Path, servo_speed_s: float, windows_ip: str) -> str:
    """hitl_procs' direct bridge: the program plant opens the backing's HIL port; pose to Unreal, a copy to the
    emulator."""
    return (f"{{python}} -s -m tools.tuning.hitl_bridge --device tcp:127.0.0.1:{HIL_PORT} --plant program "
            f"--reconnect-interval-s 0.5 --model-dir {shlex.quote(str(model_dir))} --servo-speed-s {servo_speed_s:g} "
            "--origin-lat {lat} --origin-lon {lon} --origin-alt {alt} --heading {hdg} --exit-on-stdin-eof "
            f"--log-csv {{link_csv}} --unreal-pose {windows_ip}:{POSE_PORT} {UNREAL_ALT_OFFSET_M} "
            f"--vehicle-state-copy {windows_ip}:{VEHICLE_STATE_PORT}")


def emulator(paths: ChainPaths) -> ProcSpec:
    """On Windows through WSL interop: its output comes back here, and closing its input stops it (a signal reaches
    only the interop proxy). It imports this checkout over \\\\wsl.localhost (PYTHONPATH, passed on by WSLENV)."""
    if not paths.windows_python:
        raise RunnerError("emulator: no Windows Python given (--windows-python)")
    wslenv = ":".join(item for item in (os.environ.get("WSLENV", ""), "PYTHONPATH", "PYTHONUNBUFFERED") if item)
    return ProcSpec("emulator", (paths.windows_python, "-s", "-m", "tools.hitl.zr10_emulator",
                                 "--vehicle-state", f"{paths.windows_ip}:{VEHICLE_STATE_PORT}",
                                 "--siyi", f"{paths.windows_ip}:{SIYI_PORT}", "--unreal", f"127.0.0.1:{GIMBAL_PORT}",
                                 "--log", win_path(paths.run_dir / "emulator.jsonl"), "--exit-on-stdin-eof"),
                    r"^ZR10 emulator: SIYI", READY_TIMEOUT_S["emulator"], STOP_EOF,
                    refuse=lambda: handles.windows_udp_in_use(EMULATOR_PORTS), cwd=str(paths.repo), kind="interop",
                    env={"PYTHONPATH": f"{win_path(paths.repo / 'src')};{win_path(paths.repo)}",
                         "PYTHONUNBUFFERED": "1", "WSLENV": wslenv})


def publisher_profile(paths: ChainPaths) -> Path:
    """The template with native_host set to WSL's own address, written into the run folder."""
    profile = json.loads(paths.publisher_profile.read_text(encoding="utf-8"))
    if "native_host" not in profile:
        raise RunnerError(f"{paths.publisher_profile}: no native_host")
    profile["native_host"] = paths.wsl_ip
    out = paths.run_dir / "publisher_profile.json"
    out.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    return out


def publisher(paths: ChainPaths, profile: Path, lens_table: Path) -> ProcSpec:
    out = paths.run_dir / "publisher"
    return ProcSpec("publisher", (*CONSUMER_NICE, "/usr/bin/python3", "-E", "-s", "-m",
                                  "aas_hitl.video.gstreamer_encoder", "--profile", str(profile), "--lens-table", str(lens_table), "--out", str(out)),
                    r'"ready_monotonic_ns"', READY_TIMEOUT_S["publisher"], STOP_SIGINT, stop_wait_s=60.0,
                    ready_file=out / "publisher.json",
                    ports=(("tcp", NATIVE_PORT), ("tcp", RTSP_PORT), ("udp", RTP_PORT), ("tcp", MEDIA_API_PORT)),
                    cwd=str(paths.aas_hitl), kind="publisher")


def navpy(paths: ChainPaths, record: Path, lane_first_seq: int) -> ProcSpec:
    return ProcSpec("navpy", CONSUMER_NICE + tool(paths, "navpy.main", "-c", f"udpin:127.0.0.1:{NAVPY_PORT}",
                                  "-ss", str(SITL_SYSTEM_ID), "--detector-type", "real", "--detector-backend",
                                  "charuco", "--vision-profile", VISION_PROFILE, "--pitch-controller", "vision-nav-pn",
                                  "-mwp", str(lane_first_seq), "-tac", "--router-control",
                                  f"127.0.0.1:{CONTROL_PORT}", "--router-client", "navpy", "--stop-on-stdin",
                                  "--integration-timing", str(record),
                                  "--integration-log", str(paths.run_dir / "navpy_integration.jsonl"),
                                  "--siyi-ip", paths.windows_ip),
                    "^" + NAVPY_RUNTIME_READY_MARKER, READY_TIMEOUT_S["navpy"], NAVPY_STOP_REQUEST, stop_wait_s=30.0,
                    cwd=str(paths.repo),
                    env={"PYTHONPATH": f"{paths.repo / 'src'}:{paths.repo}", "PYTHONUNBUFFERED": "1"})


def navpy_link_id() -> str:
    """navpy's live-link identity (vehicle_capability_composition._link_identity_source): its device, '#', system."""
    return f"udpin:127.0.0.1:{NAVPY_PORT}#{SITL_SYSTEM_ID}"


# ---- Unreal, a Windows process started from WSL -------------------------------------------------------------------
def _powershell(script: str) -> str:
    done = subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
                          capture_output=True, text=True, timeout=60, check=False)
    if done.returncode != 0:
        raise RunnerError(f"powershell failed: {done.stderr.strip()}")
    return done.stdout.strip()


def unreal_running() -> Optional[str]:
    listing = subprocess.run(["tasklist.exe", "/FI", f"IMAGENAME eq {UNREAL_IMAGE}", "/NH"], capture_output=True,
                             text=True, timeout=30, check=False).stdout
    return f"another {UNREAL_IMAGE} runs" if UNREAL_IMAGE.lower() in listing.lower() else None


class WindowsHandle:
    """Unreal by its Windows pid: taskkill asks it to close (it writes "Log file closed"), /F ends it."""

    def __init__(self, pid: int) -> None:
        self.pid = pid
        self._gone = False

    def running(self) -> bool:
        if not self._gone:
            listing = subprocess.run(["tasklist.exe", "/FI", f"PID eq {self.pid}", "/NH"], capture_output=True,
                                     text=True, timeout=30, check=False).stdout
            self._gone = re.search(rf"\b{self.pid}\b", listing) is None
        return not self._gone

    def exit_code(self) -> Optional[int]:
        return None                          # Windows keeps it from WSL; the log's last line shows a clean close

    def request_stop(self) -> None:
        subprocess.run(["taskkill.exe", "/PID", str(self.pid)], capture_output=True, timeout=30, check=False)

    def terminate(self) -> None:
        self.request_stop()

    def kill(self) -> None:
        subprocess.run(["taskkill.exe", "/PID", str(self.pid), "/F"], capture_output=True, timeout=30, check=False)

    def group_alive(self) -> bool:
        return self.running()

    def confirm_ready(self) -> None:
        """Its pid came from Start-Process; nothing more to learn."""


def unreal(paths: ChainPaths, lens_table: Path) -> ProcSpec:
    log = paths.run_dir / "unreal.log"
    args = (win_path(paths.uproject), UNREAL_LEVEL, "-game", "-windowed", "-ResX=640", "-ResY=360",
            "-NavPyEnableNativeCameraStream=1", f"-NavPyNativeCameraHost={paths.wsl_ip}",
            "-NavPyInterpolatedVehiclePose=1", "-NavPyNativeCameraHeaderVersion=3",
            "-NavPyNativeCameraOverscan=80,48", "-NavPyNativeCameraBoardTimeCadence=1",
            "-NavPyNativeCameraFrameRate=30", "-NavPyNativeCameraWidth=1920", "-NavPyNativeCameraHeight=1080",
            f"-NavPyLensProfile={win_path(lens_table)}", "-NoRayTracing", "-NoVSync", "-nosound",
            f"-abslog={win_path(log)}",
            "-ini:Game:[/Script/NavPyUnrealBridge.NavPyGimbalCameraActor]:bEnablePixelStreaming=False",
            "-ini:Game:[/Script/NavPyUnrealBridge.NavPyOperatorViewActor]:bEnableTailPixelStreaming=False",
            "-ini:Game:[/Script/NavPyUnrealBridge.NavPyOperatorViewActor]:bEnableGimbalInset=False",
            "-ExecCmds=t.MaxFPS 60,ShowFlag.Rendering 0")        # the main view off (todo 186)
    return ProcSpec("unreal", (UNREAL_EXE, *args), "NavPyCameraMount", READY_TIMEOUT_S["unreal"], stop_wait_s=90.0,
                    ready_file=log, refuse=unreal_running, kind="windows")


def spawn_unreal(affinity: Optional[int]) -> Callable[[ProcSpec, IO[str]], WindowsHandle]:
    """Start-Process with one Windows command line (PowerShell 5.1 does not quote list items with spaces)."""
    def spawn(spec: ProcSpec, output: IO[str]) -> WindowsHandle:
        line = subprocess.list2cmdline(list(spec.argv[1:])).replace("'", "''")
        script = (f"$p = Start-Process -PassThru -FilePath '{spec.argv[0]}' -ArgumentList '{line}'; "
                  "$p.PriorityClass = 'AboveNormal'; ")
        if affinity:
            script += f"$p.ProcessorAffinity = {affinity}; "
        pid = int(_powershell(script + "$p.Id").splitlines()[-1])
        output.write(f"unreal pid {pid}: {spec.argv[0]} {line}\n")
        output.flush()
        return WindowsHandle(pid)
    return spawn
