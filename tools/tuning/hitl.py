r"""Fly the tuning runs on a flight controller instead of SITL, and the board's parameter tools.

The flight subcommands (sitl_driver port-check / inflight-tune / vs-autotune and
sitl_faults) take --board DEVICE to fly a flight controller running ArduPlane simulation
firmware: the onboard SITL model (SimOnHW) or, with --bridge and --relay, HITL (HIL link
contract v2): the JSBSim bridge in WSL and the relay from the board's HIL USB port to it
(hitl_procs.py). The board gets the same parameter files as SITL (set ARDUPILOT_ROOT to
the ArduPilot tree that holds Tools/autotest), then the starting parameters (--params-file),
the sticks as RC_CHANNELS_OVERRIDE, and the logs come back over MAVLink. A HITL run fails
(exit code 1) when the link-health gate fails (hitl_gate.link_health; numbers in the run's
JSON under vehicle.link_health). Without --board nothing changes: SITL in WSL with the UDP
receiver (--params-file is added to its --defaults).

The aircraft's own data (aero models, its parameter files, its HITL airspeed, takeoff and launch data)
is private and lives outside the repository (tools/tuning/private_data.py): <private> below is that folder,
%LOCALAPPDATA%\navpy-hitl-tools by default, and <private_wsl> the same folder as WSL sees it (/mnt/c/...).
With --bridge the runner loads <private>\hitl\hitl_aircraft.parm and hitl_launch.parm (--hitl-data DIR
to read them elsewhere); the bridge reads its launch profile with --launch-file.

Usage (Windows PowerShell 5.1, in the repository root; COM15 is MAVLink on USB OTG1 of the
Pixhawk 6X, COM11 the HIL port on OTG2; each command is one line):
  $env:ARDUPILOT_ROOT = "\\wsl.localhost\<distro>\home\<user>\ardupilot"
  .venv\Scripts\python.exe -m tools.tuning.hitl backup --board COM15 --out C:\tune_hitl\before.parm
  .venv\Scripts\python.exe -m tools.tuning.sitl_driver port-check --board COM15 --erase-params
    --workdir C:\tune_hitl\sign_check
    --params-file <private>\params\<aircraft>.parm
    --bridge "wsl.exe --cd {repo_wsl} -- python3 -m tools.tuning.hitl_bridge
      --device tcpin:{port} --model-dir <private_wsl>/models/Peron15 --launch-file <private_wsl>/hitl/launch.json
      --origin-lat {lat} --origin-lon {lon} --origin-alt {alt} --heading {hdg} --exit-on-stdin-eof
      --log-csv {link_csv_wsl}"
    --relay "{python} -m tools.tuning.hitl_bridge.relay --device COM11 --bridge 127.0.0.1:{port}"
  .venv\Scripts\python.exe -m tools.tuning.hitl restore --board COM15 --file C:\tune_hitl\before.parm
Mission Planner can watch a board run while the runner holds COM15: add --mp-mirror to the run,
then in Mission Planner pick UDP, Connect, local port 9640 (mp_mirror.py; it cannot override the RC).
The same chain on SITL (the HIL backend, its link on SERIAL5 over TCP): in WSL, from the repository root,
  ~/venv-ardupilot/bin/python -m tools.tuning.hitl sitl-backing --model hil --origin=-35.363261,149.165230,584,353
    --workdir ~/tune_sitl/hitl_backing
then on Windows the port-check above with --board tcp:127.0.0.1:8160, the same --origin=...,
no --erase-params needed, and the relay's --device tcp:127.0.0.1:8168 (sitl-backing prints both).
Logs copied from the SD card (--log-fetch none) are checked with
  .venv\Scripts\python.exe -m tools.tuning.hitl link-check --log C:\tune_hitl\run\logs\00000012.BIN
    --link-csv C:\tune_hitl\run\link.csv
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Callable, Optional

from tools.tuning.board import DEFAULT_BAUD, Board, is_serial
from tools.tuning.board_backup import backup, restore_cmd
from tools.tuning.hitl_gate import link_health
from tools.tuning.hitl_params import ParamFile, origin_params
from tools.tuning.hitl_procs import FIELDS, HitlProcesses, template_errors
from tools.tuning.mp_mirror import DEFAULT_PORT as DEFAULT_MP_MIRROR_PORT
from tools.tuning.mp_mirror import MavlinkMirror, reserved_band
from tools.tuning.private_data import AIRCRAFT_PARAMS_NAME, LAUNCH_PARAMS_NAME, hitl_data_dir
from tools.tuning.sitl_sim import (DEFAULT_INSTANCE, DEFAULT_PARAMS, FIRMWARE_BINARY, HERE, HIL_MODEL, HITL_PARAMS,
                                   HOME_CMAC, PERON_LIKE_PARAMS, Sitl)
from tools.tuning.vehicle import Vehicle

BRIDGE_PARAMS = HERE / "sitl" / "hitl_bridge.parm"     # elevon map and PWM range the bridge unmixes
BOARD_PARAMS = HERE / "sitl" / "hitl_board.parm"       # HIL sensors and link port on a real board only
SITL_BACKING_MODELS = ("plane-elevon", HIL_MODEL)      # onboard model (SimOnHW stand-in) or the HIL backend
SITL_BACKING_PARAMS = HERE / "sitl" / "sitl_backing.parm"   # after --defaults: SITL's own physics rate
# The bridge's TCP port inside WSL, reached from Windows through WSL's localhost forwarding:
# the relay's own default (hitl_bridge/relay.py DEFAULT_BRIDGE; the JSBSim program takes 9621/9622,
# hitl_bridge/jsbsim_program.py). Outside the GCS slot bands and the SITL ports (src/gcs/backend/
# instance_ports.py: 3000+, 8000+, 14550+, 15550+, 16550+ per chat; 5501-8310 per instance).
DEFAULT_BRIDGE_PORT = 9620
BOARD_ONLY = {"erase_params": "--erase-params", "bridge": "--bridge", "relay": "--relay",
              "bridge_port": "--bridge-port", "baud": "--baud", "log_fetch": "--log-fetch",
              "mp_mirror": "--mp-mirror", "hitl_data": "--hitl-data"}
MAX_PORT = 65535               # UDP port numbers are 16 bit


def add_vehicle_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--instance", type=int, default=DEFAULT_INSTANCE, help="SITL instance")
    p.add_argument("--speedup", type=int, default=1, help="SITL speedup; a board runs at 1")
    p.add_argument("--rc", choices=("udp", "mavlink"), help="SITL RC input (default udp); a board uses mavlink")
    p.add_argument("--origin", default=HOME_CMAC,
                   help="lat,lon,alt,hdg of the start: SITL --home, or the board's SIM_OPOS_* and the bridge's "
                        "origin; write --origin=-35.3,... when the latitude is negative")
    p.add_argument("--params-file", action="append", type=Path, metavar="FILE",
                   help="starting parameters (NAME VALUE lines), e.g. the aircraft's flight-control settings; "
                        "repeatable, later files win; loaded after the standard files and before the run's own "
                        "settings; on a board every name must exist")
    p.add_argument("--board", help="fly a flight controller with simulation firmware: COMn, /dev/ttyACMn or tcp:host:port")
    p.add_argument("--baud", type=int, help=f"board serial baud (default {DEFAULT_BAUD})")
    p.add_argument("--erase-params", action="store_true",
                   help="board: erase all parameters (after the verified backup) before loading the files; "
                        "required with --bridge on a serial board")
    placeholders = " ".join("{%s}" % f for f in FIELDS)
    p.add_argument("--bridge", help="board HITL: the JSBSim bridge command, started before the board's last reboot; "
                                    "it opens the board's HIL port itself (--device COM11 --plant package) or, with "
                                    "--relay, listens on {port}; it must take the origin from {lat} {lon} {alt} {hdg} "
                                    "and write its link log to {link_csv} (or {link_csv_wsl}); placeholders: "
                                    f"{placeholders}; output in bridge_stdout.txt")
    p.add_argument("--relay", help="board HITL with a WSL bridge: the relay command joining the board's HIL port to "
                                   "127.0.0.1:{port}, started once the bridge listens and stopped first; output in "
                                   "relay_stdout.txt. Without it the bridge opens the board's HIL port itself "
                                   "(the recommended Windows bridge, --device COM11 --plant package)")
    p.add_argument("--hitl-data", type=Path, metavar="DIR",
                   help="board HITL: the folder of the aircraft's private HITL files, "
                        f"{AIRCRAFT_PARAMS_NAME} (airspeeds, takeoff) and {LAUNCH_PARAMS_NAME} (launch detection) "
                        "(default: the hitl folder of tools/tuning/private_data.py)")
    p.add_argument("--bridge-port", type=int, help=f"board HITL: the bridge's TCP port (default {DEFAULT_BRIDGE_PORT})")
    p.add_argument("--log-fetch", choices=("mavlink", "none"),
                   help="board: download the new logs over MAVLink (default) or list them for copying from the SD card")
    p.add_argument("--mp-mirror", type=int, nargs="?", const=DEFAULT_MP_MIRROR_PORT, metavar="PORT",
                   help="board: mirror the MAVLink link to UDP 127.0.0.1:PORT (default "
                        f"{DEFAULT_MP_MIRROR_PORT}) for Mission Planner (Connect: UDP, that port); it cannot "
                        "override the RC")


def vehicle_arg_errors(args: argparse.Namespace, scripts: list[Path]) -> list[str]:
    """What is wrong with the vehicle arguments for a run that needs these Lua scripts."""
    errors = [f"--params-file {p}: no such file" for p in args.params_file or [] if not Path(p).expanduser().is_file()]
    if args.board is None:
        return errors + [f"{flag} needs --board" for name, flag in BOARD_ONLY.items()
                         if getattr(args, name) not in (None, False)]
    if scripts:
        errors.append("--board cannot load Lua scripts (that needs MAVLink FTP); use --firmware")
    if args.speedup != 1:
        errors.append("--board runs in real time: --speedup must be 1")
    if args.rc == "udp":
        errors.append("--board has no UDP RC input; it uses MAVLink overrides")
    if args.relay and not args.bridge:
        errors.append("--relay needs --bridge")
    elif args.bridge:
        errors += template_errors(args.bridge, args.relay)
        errors += [f"--bridge needs the aircraft's {path}: no such file (private data, tools/tuning/private_data.py; "
                   "--hitl-data DIR to read it elsewhere)" for path in hitl_data_files(args.hitl_data)
                   if not path.is_file()]
        if is_serial(args.board) and not args.erase_params:
            errors.append("--bridge on a serial board needs --erase-params: HITL starts from the HIL firmware's "
                          "defaults, not the flight settings saved on the board")
    if args.bridge_port is not None and not args.relay:
        errors.append("--bridge-port needs --bridge and --relay")
    if args.mp_mirror is not None and (not 0 < args.mp_mirror <= MAX_PORT or reserved_band(args.mp_mirror)):
        errors.append(f"--mp-mirror {args.mp_mirror}: not a free port ({reserved_band(args.mp_mirror) or 'out of range'})")
    try:
        origin_params(args.origin)
    except ValueError as exc:
        errors.append(str(exc))
    return errors


def hitl_data_files(hitl_data: Optional[Path]) -> tuple[Path, Path]:
    """The aircraft's private HITL parameter files: (airspeeds and takeoff, launch detection), in hitl_data
    or, when None, in the private folder's hitl subfolder (private_data.hitl_data_dir)."""
    folder = Path(hitl_data).expanduser() if hitl_data is not None else hitl_data_dir()
    return folder / AIRCRAFT_PARAMS_NAME, folder / LAUNCH_PARAMS_NAME


def board_param_files(extra_params: list[Path], params_files: list[Path], bridge: bool,
                      serial_board: bool, hitl_data: Optional[Path] = None) -> list[ParamFile]:
    """SITL's --defaults files in SITL's order (the two upstream files may name parameters the
    board lacks, e.g. EK2_ENABLE), then for HITL the bridge's file, the aircraft's private airspeed and
    takeoff file and, on a serial board, the board-only file (SITL has no SERIAL8 or BRD_HEAT_TARG), the
    starting parameters, for HITL the aircraft's private launch file (over the starting launch settings),
    the run's own files and hitl.parm. Later files win."""
    aircraft, launch = hitl_data_files(hitl_data)
    files = [ParamFile(p, required=False) for p in DEFAULT_PARAMS] + [ParamFile(PERON_LIKE_PARAMS, True)]
    files += [ParamFile(BRIDGE_PARAMS, True), ParamFile(aircraft, True)] if bridge else []
    files += [ParamFile(BOARD_PARAMS, True)] if bridge and serial_board else []
    files += [ParamFile(p, True) for p in params_files]
    files += [ParamFile(launch, True)] if bridge else []
    return files + [ParamFile(p, True) for p in extra_params] + [ParamFile(HITL_PARAMS, True)]


def open_vehicle(args: argparse.Namespace, extra_params: list[Path], scripts: list[Path], binary: Path) -> Vehicle:
    """Sitl, as before, without --board; else the Board with the same parameter files."""
    errors = vehicle_arg_errors(args, scripts)
    if errors:
        raise SystemExit("error: " + "; ".join(errors))
    workdir = Path(args.workdir).expanduser()
    params_files = [Path(p).expanduser() for p in args.params_file or []]
    if args.board is None:
        return Sitl(workdir, instance=args.instance, speedup=args.speedup, extra_params=params_files + extra_params,
                    home=args.origin, binary=binary, scripts=scripts, rc=args.rc or "udp")
    hitl = None
    if args.bridge:
        hitl = HitlProcesses(args.bridge, args.relay, args.bridge_port or DEFAULT_BRIDGE_PORT, workdir, args.origin)
    files = board_param_files(extra_params, params_files, hitl is not None, is_serial(args.board), args.hitl_data)
    mirror = MavlinkMirror(args.mp_mirror) if args.mp_mirror is not None else None
    return Board(args.board, workdir, files, origin=args.origin, erase=args.erase_params, hitl=hitl,
                 log_fetch=args.log_fetch != "none", baud=args.baud or DEFAULT_BAUD, mirror=mirror)


def run_file(workdir: str, suffix: str) -> Path:
    """<run folder name><suffix> next to the run folder: the run's own parameters, its JSON record."""
    path = Path(workdir).expanduser()
    return path.parent / (path.name + suffix)


def write_run_params(workdir: str, params: dict) -> Path:
    """The run's own boot parameters (NAME VALUE lines) in <workdir>_extra.parm."""
    extra = run_file(workdir, "_extra.parm")
    extra.parent.mkdir(parents=True, exist_ok=True)
    extra.write_text("".join(f"{k} {v}\n" for k, v in params.items()))
    return extra


def record_run(record: dict, report: dict, out: Path) -> int:
    """Add a board's report (setup, logs, timing, HITL link health) to the run's record and write
    it as JSON. Returns the exit code: 1 when the HITL link failed its health gate."""
    if report:
        record["vehicle"] = report
    out.write_text(json.dumps(record, indent=1))
    health = report.get("link_health")
    if health is None or health["ok"]:
        return 0
    for problem in health["problems"]:
        print(f"[hitl] link health FAILED: {problem}", flush=True)
    return 1


def fly_and_record(vehicle: Vehicle, flight: Callable[[], None], record: dict, out: Path,
                   kept_texts: tuple[str, ...] = ()) -> int:
    """Start the vehicle, fly (flight fills record), stop it and write the run's record (record_run) with the
    STATUSTEXTs holding one of kept_texts and the logs, also when the flight failed: a failed takeoff keeps its
    logs, the board's link health and Mission Planner counts, and the error, which is raised again once the
    record is written. Returns record_run's exit code."""
    try:
        vehicle.start()
        flight()
    except Exception as exc:
        record["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        vehicle.stop()
        if kept_texts:
            record["messages"] = [m for m in vehicle.messages if any(t in m for t in kept_texts)]
        record["logs"] = [str(p) for p in vehicle.logs()]
        code = record_run(record, vehicle.report(), out)
    return code


def sitl_backing(args: argparse.Namespace) -> None:
    """A bare SITL with the fork binary for the board path over TCP: no RC, no runner parameters.
    --model hil runs the HIL backend with its link (SERIAL5, protocol 100) on TCP for the relay;
    --origin is SITL's --home and must equal the run's --origin. A board's hwdef defaults may be given with --defaults;
    sitl_backing.parm, loaded after them, keeps SITL's own physics rate."""
    origin_params(args.origin)        # a malformed origin fails here, not inside SITL
    defaults = [Path(p) for p in args.defaults or []] + [SITL_BACKING_PARAMS]
    sitl = Sitl(Path(args.workdir).expanduser(), instance=args.instance, model=args.model, home=args.origin,
                binary=Path(args.binary), rc=None, defaults=defaults)
    try:
        sitl.start()
        sitl.mav.close()       # serial0 is the runner's; after a reboot SITL waits for it again
        sitl.mav = None
        relay = f"; relay --device tcp:127.0.0.1:{sitl.hil_port}" if sitl.hil_port is not None else ""
        print(f"[backing] SITL instance {args.instance} ({args.model}): connect with --board "
              f"tcp:127.0.0.1:{sitl.mavlink_port} --origin={args.origin}{relay}; Ctrl-C stops it", flush=True)
        sitl.proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        sitl.stop()


def link_check(args: argparse.Namespace) -> None:
    """The HITL link-health gate on logs copied from the SD card (--log-fetch none)."""
    health = link_health([Path(p) for p in args.log], Path(args.link_csv))
    print(json.dumps(health, indent=1))
    sys.exit(0 if health["ok"] else 1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sitl-backing", help=sitl_backing.__doc__)
    s.add_argument("--workdir", required=True)
    s.add_argument("--instance", type=int, default=DEFAULT_INSTANCE)
    s.add_argument("--binary", default=str(FIRMWARE_BINARY))
    s.add_argument("--model", choices=SITL_BACKING_MODELS, default=SITL_BACKING_MODELS[0],
                   help=f"{HIL_MODEL}: the HIL backend, its link on SERIAL5 over TCP for the relay and the bridge")
    s.add_argument("--origin", default=HOME_CMAC, help="SITL's --home; the run's --origin must be the same")
    s.add_argument("--defaults", action="append", help="--defaults file, e.g. the board hwdef's defaults.parm (SIM_RATE_HZ stays SITL's: "
                        "sitl/sitl_backing.parm is loaded after it)")
    s.set_defaults(func=sitl_backing)
    c = sub.add_parser("link-check", help=link_check.__doc__)
    c.add_argument("--log", action="append", required=True, help="a board DataFlash log (.BIN); repeatable")
    c.add_argument("--link-csv", required=True, help="the bridge's link log (the run folder's link.csv)")
    c.set_defaults(func=link_check)
    for name, func, file_arg in (("backup", backup, "--out"), ("restore", restore_cmd, "--file")):
        p = sub.add_parser(name, help=func.__doc__)
        p.add_argument("--board", required=True, help="COMn, /dev/ttyACMn or tcp:host:port")
        p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        p.add_argument(file_arg, required=True, help="parameter file (NAME,VALUE lines)")
        p.set_defaults(func=func)
    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
