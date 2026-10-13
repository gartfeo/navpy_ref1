"""PC side of the HITL link: JSBSim physics stepped on the flight controller's clock.

Classic hardware-in-the-loop, HIL link contract v2 of the fork's SIM_HIL backend
(libraries/SITL/examples/HIL/readme.md; reference bridge Tools/autotest/pysim/hil_bridge.py).
The flight controller runs HIL firmware whose backend sends HIL_ACTUATOR_CONTROLS once per
main loop on its SIM HIL port (SERIALn_PROTOCOL 100; SERIAL8 = USB OTG2 on the Pixhawk 6X);
the bridge answers each with one HIL_STATE_QUATERNION carrying the same time_usec
(frame_loop.py). Physics time follows only the board clock: whole fixed JSBSim steps since
the first frame, outputs held between frames (board_clock.BoardClockStepper); the PC clock only
stamps the log and paces reconnects, and nothing is random. The board never waits for the
bridge (flags 0); flags bit 0 (lockstep) comes only from SITL's hil:lockstep model.
The aircraft starts at rest at the origin and is launched a set delay (--catapult-delay-s)
after the armed bit first appears:
- --launch rail (default): the aircraft's catapult as its logs measured it (rail.py), from the private
  launch profile (--launch-file, launch_profile.py): on the carriage on a straight rail, pushed by the
  measured acceleration over the measured travel and released at the measured speed
  (--rail-release-speed-m-s scales the push), the accelerometer reading the true specific force; the
  physics starts at the release;
- --launch launcher: the earlier simple launcher (catapult.py): held at --launcher-pitch-deg
  and --launcher-height-m, then --catapult-force-n along the body x axis for
  --catapult-duration-s, the physics running from the stroke;
- --launch ground: the same force from rest on the skids (the launch profile's ground_rest).
A board reboot (board time going back, or jumping forward by more than 2 s) puts it back at
the start (the JSBSim program is restarted; an ALERT if the board was armed); a shorter forward
jump with lost frames (a USB stall) is stepped across with the held servo outputs and logged as a
GAP. The same frame again (a lockstep SITL resending after a reconnect) is answered with the
same state without stepping; other content at the same board time is reported as an ERROR.

Physics (--plant): "program" (default) runs the WSL JSBSim program /usr/games/JSBSim
(jsbsim_process.py); "package" uses the jsbsim Python package (jsbsim_plant.py). Both add
SITL's servo lag (--servo-speed-s, default SIM_SERVO_SPEED's 0.14 s, 0 = none; plant_options.py).
A constant wind (--wind-speed-m-s, --wind-from-deg; default calm) starts, as in SITL, 5 s of
board time after boot (environment.py). JSBSim is started before the board port is opened,
so a successful connect means the physics is up.

Board link (--device): tcpin:PORT listens for one client at a time (the Windows relay,
relay.py, carrying the board's COM port; a new client is accepted after a disconnect);
tcp:HOST:PORT connects to SITL's --serialN=tcp:<port> (SERIALN_PROTOCOL 100); a serial port
name opens it directly (its input buffer is flushed on every open). The bridge exits when
the board link has been down for --client-grace-s, counted from its start until the first connection and from the end of the last one (start the relay
within that time), and on stdin end-of-file only with --exit-on-stdin-eof. It stops the
JSBSim program on every exit path (also SIGTERM/SIGHUP; on Linux the kernel also stops it if
the bridge is killed).

WSL bash (board through the Windows relay; the module is found from the navpy repo root; <private> is the
private folder of tools/tuning/private_data.py as WSL sees it, e.g. under /mnt/c):
  cd <navpy repo> && python3 -m tools.tuning.hitl_bridge --device tcpin:9620 --model-dir <private>/models/<model> --launch-file <private>/hitl/launch.json --log-csv ~/hitl_logs/run1.csv
SITL pre-validation uses --device tcp:127.0.0.1:<SITL HIL port>. The board's
SIM_OPOS_LAT/LNG/ALT/HDG must equal the origin given here (--origin-lat/lon/alt,
--heading); the defaults are both sides' defaults (CMAC, fork libraries/SITL/SITL.cpp:447-462).
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from types import FrameType
from typing import Callable, NoReturn, Optional

from tools.tuning.hitl_bridge.catapult import Catapult
from tools.tuning.hitl_bridge.frame_loop import Bridge, LinkLog, StopConditions, run_bridge
from tools.tuning.hitl_bridge.jsbsim_process import JSBSimProcessPlant
from tools.tuning.hitl_bridge.jsbsim_program import (DEFAULT_CONTROL_PORT, DEFAULT_STATE_PORT, JSBSIM_PROGRAM,
                                                     ProgramConfig)
from tools.tuning.hitl_bridge.link import DEFAULT_RECONNECT_INTERVAL_S, MavlinkLink
from tools.tuning.hitl_bridge.board_clock import DEFAULT_LOOP_RATE_HZ, BoardClockStepper, step_rate_hz
from tools.tuning.hitl_bridge.physics import Launch, Origin, Plant
from tools.tuning.hitl_bridge.environment import Wind
from tools.tuning.hitl_bridge.launch_profile import ground_rest, rail_spec, read_launch_data
from tools.tuning.hitl_bridge.plant_options import DEFAULT_SERVO_SPEED_S, PlantOptions, StartPose, launcher_pose
from tools.tuning.hitl_bridge.rail import RailGeometry, RailLaunch, spec_stroke
from tools.tuning.private_data import LAUNCH_PROFILE_NAME, hitl_data_dir
from tools.tuning.hitl_bridge.transport import DEFAULT_SERIAL_BAUD, Transport, make_transport

# Origin defaults = SIM_OPOS_LAT/LNG/ALT/HDG defaults (fork libraries/SITL/SITL.cpp:447-462),
# so the board and the bridge agree when neither is set.
DEFAULT_ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.165230, ground_alt_msl_m=584.0, heading_deg=353.0)
# Integration step: the contract's 1/1200 s, six steps per 200 Hz main loop, and SITL's
# default physics rate (SIM_RATE_HZ 1200).
DEFAULT_INTEGRATION_DT_S = 1.0 / 1200.0
# The simple launcher's force (--launch launcher, ground): a typical small-UAV launcher's push, sized to
# bring a launcher-class aircraft to about its cruise speed. Not a measured launcher.
DEFAULT_CATAPULT_FORCE_N = 350.0
DEFAULT_CATAPULT_DURATION_S = 0.5
# Time from the first armed frame to the stroke, for the flight mode to be set and the
# operator to see the arming; an operating choice, not a property of the aircraft.
DEFAULT_CATAPULT_DELAY_S = 3.0
# The simple launcher holds the aircraft at rest until the stroke and releases it in the air. From the
# skids (a nose-down rest pose) the push can end at a negative angle of attack and leave the aircraft on
# the ground or ground-loop it; from a raised, nose-up launcher it flies off. Typical small-UAV values, not
# a measured launcher (that is --launch rail).
DEFAULT_LAUNCHER_PITCH_DEG = 10.0
DEFAULT_LAUNCHER_HEIGHT_M = 1.0
LAUNCH_RAIL = "rail"
LAUNCH_LAUNCHER = "launcher"
LAUNCH_GROUND = "ground"
# The relay keeps its connection to the bridge while the board reboots, so a disconnect
# means the relay (or SITL) stopped. 30 s leaves time to start or restart the relay by hand
# without losing the flight; after that the bridge ends and stops JSBSim.
DEFAULT_CLIENT_GRACE_S = 30.0
PLANT_PROGRAM = "program"
PLANT_PACKAGE = "package"
EXIT_OK = 0
EXIT_PHYSICS_FAILED = 1
EXIT_BAD_SETUP = 2

PlantFactory = Callable[[Path, Origin, float, PlantOptions], Plant]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                epilog="Commands: see the module docstring (python -c \"import "
                                       "tools.tuning.hitl_bridge.bridge as b; print(b.__doc__)\").",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--device", required=True,
                   help="board link: tcpin:PORT (listen; the Windows relay connects), tcp:HOST:PORT (SITL "
                        "--serialN, SERIALN_PROTOCOL 100), or a serial port (COM11, /dev/ttyACM1)")
    p.add_argument("--baud", type=int, default=DEFAULT_SERIAL_BAUD, help="serial baud (USB CDC ignores it)")
    p.add_argument("--model-dir", type=Path, required=True,
                   help="JSBSim aircraft folder holding <name>.xml and Engines/ (the private models folder)")
    p.add_argument("--plant", choices=(PLANT_PROGRAM, PLANT_PACKAGE), default=PLANT_PROGRAM,
                   help="JSBSim program over its sockets, or the jsbsim Python package in-process")
    p.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN.lat_deg, help="deg, = SIM_OPOS_LAT")
    p.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN.lon_deg, help="deg, = SIM_OPOS_LNG")
    p.add_argument("--origin-alt", type=float, default=DEFAULT_ORIGIN.ground_alt_msl_m,
                   help="ground elevation, m MSL, = SIM_OPOS_ALT")
    p.add_argument("--heading", type=float, default=DEFAULT_ORIGIN.heading_deg, help="deg true, = SIM_OPOS_HDG")
    p.add_argument("--launch", choices=(LAUNCH_RAIL, LAUNCH_LAUNCHER, LAUNCH_GROUND), default=LAUNCH_RAIL,
                   help="rail: the measured catapult rail (rail.py); launcher: the earlier simple launcher, held at "
                        "rest, then a force along the body x axis; ground: that force from rest on the skids")
    p.add_argument("--catapult-delay-s", type=float, default=DEFAULT_CATAPULT_DELAY_S,
                   help="from the first armed frame to the start of the stroke")
    p.add_argument("--launch-file", type=Path, default=hitl_data_dir() / LAUNCH_PROFILE_NAME,
                   help="rail, ground: the aircraft's launch profile (private data, launch_profile.py)")
    p.add_argument("--rail-release-speed-m-s", type=float, default=None,
                   help="rail: speed at the end of the push (default: the profile's measured one; another value "
                        "scales its acceleration by the square of the ratio over the same travel)")
    p.add_argument("--rail-elevation-deg", type=float, default=None,
                   help="rail: its angle above the horizon (default: the profile's)")
    p.add_argument("--rail-height-m", type=float, default=None,
                   help="rail: height of the aircraft on the carriage above the ground at rest (default: the profile's)")
    p.add_argument("--catapult-force-n", type=float, default=DEFAULT_CATAPULT_FORCE_N, help="launcher, ground")
    p.add_argument("--catapult-duration-s", type=float, default=DEFAULT_CATAPULT_DURATION_S, help="launcher, ground")
    p.add_argument("--launcher-pitch-deg", type=float, default=DEFAULT_LAUNCHER_PITCH_DEG, help="launcher")
    p.add_argument("--launcher-height-m", type=float, default=DEFAULT_LAUNCHER_HEIGHT_M,
                   help="launcher: height of the aircraft on it above the ground")
    p.add_argument("--integration-dt-s", type=float, default=DEFAULT_INTEGRATION_DT_S,
                   help="JSBSim fixed step, 1/N s for a whole N (the program gets --simulation-rate=N)")
    p.add_argument("--loop-rate-hz", type=float, default=DEFAULT_LOOP_RATE_HZ,
                   help="the board's SCHED_LOOP_RATE; only the GAP reports depend on it")
    p.add_argument("--servo-speed-s", type=float, default=DEFAULT_SERVO_SPEED_S,
                   help="servo lag as ArduPilot SITL's SIM_SERVO_SPEED (1-pole, time constant in s); 0: none")
    p.add_argument("--wind-speed-m-s", type=float, default=0.0,
                   help="constant horizontal wind speed, from 5 s of board time after boot (as SITL)")
    p.add_argument("--wind-from-deg", type=float, default=0.0, help="direction the wind blows from, deg true")
    p.add_argument("--reconnect-interval-s", type=float, default=DEFAULT_RECONNECT_INTERVAL_S)
    p.add_argument("--client-grace-s", type=float, default=DEFAULT_CLIENT_GRACE_S,
                   help="exit when the board link has been down this long, from the start or the last "
                        "connection (0: never)")
    p.add_argument("--exit-on-stdin-eof", action="store_true",
                   help="also exit when stdin reaches end-of-file (when started by another program)")
    jsb = p.add_argument_group("JSBSim program (--plant program)")
    jsb.add_argument("--jsbsim-program", default=JSBSIM_PROGRAM)
    jsb.add_argument("--jsbsim-control-port", type=int, default=DEFAULT_CONTROL_PORT,
                     help="its input socket (TCP); refused if busy")
    jsb.add_argument("--jsbsim-state-port", type=int, default=DEFAULT_STATE_PORT,
                     help="its state output (UDP to us); refused if busy")
    jsb.add_argument("--run-root", type=Path, default=ProgramConfig().run_root,
                     help="a fresh run folder (model copy, script, console log) is made under it")
    p.add_argument("--log-csv", type=Path, default=None,
                   help="link log; default hitl_link_<UTC time>.csv in the current directory")
    return p


def _default_log_path(now: datetime) -> Path:
    return Path(f"hitl_link_{now.strftime('%Y%m%dT%H%M%SZ')}.csv")


def _plant_factory(args: argparse.Namespace) -> PlantFactory:
    if args.plant == PLANT_PACKAGE:
        from tools.tuning.hitl_bridge.jsbsim_plant import JSBSimPlant   # noqa: PLC0415 - imports jsbsim lazily
        return JSBSimPlant
    config = ProgramConfig(program=args.jsbsim_program, control_port=args.jsbsim_control_port,
                           state_port=args.jsbsim_state_port, run_root=args.run_root)

    def make(model_dir: Path, origin: Origin, dt_s: float, options: PlantOptions) -> Plant:
        return JSBSimProcessPlant(model_dir, origin, dt_s, config, options)
    return make


def _given_or(value: Optional[float], default: float) -> float:
    return default if value is None else value


def launch_option(args: argparse.Namespace, origin: Origin) -> tuple[StartPose, Launch]:
    """The launch (--launch) and where the plant starts for it: the rail's release, or at rest."""
    if args.launch == LAUNCH_RAIL:
        spec = rail_spec(read_launch_data(args.launch_file))
        geometry = RailGeometry(origin, elevation_deg=_given_or(args.rail_elevation_deg, spec.elevation_deg),
                                height_m=_given_or(args.rail_height_m, spec.height_m), pitch_deg=spec.carriage_pitch_deg)
        rail = RailLaunch(geometry, spec_stroke(spec, args.rail_release_speed_m_s), args.catapult_delay_s)
        return rail.release_pose(), rail
    holds = args.launch == LAUNCH_LAUNCHER
    start = (launcher_pose(args.launcher_pitch_deg, args.launcher_height_m) if holds
             else ground_rest(read_launch_data(args.launch_file)))
    return start, Catapult(args.catapult_force_n, args.catapult_duration_s, args.catapult_delay_s, holds=holds)


def plant_options(args: argparse.Namespace, start: StartPose) -> PlantOptions:
    return PlantOptions(servo_speed_s=args.servo_speed_s, start=start)


def wind_option(args: argparse.Namespace) -> Wind:
    return Wind(speed_m_s=args.wind_speed_m_s, from_deg=args.wind_from_deg)


def _raise_interrupt(signum: int, frame: Optional[FrameType]) -> NoReturn:
    raise KeyboardInterrupt(f"signal {signum}")


def _install_stop_signals() -> dict:
    """SIGTERM/SIGHUP end the run like Ctrl+C, so the finally block stops JSBSim."""
    previous = {}
    if threading.current_thread() is not threading.main_thread():
        return previous
    for name in ("SIGTERM", "SIGHUP"):
        signum = getattr(signal, name, None)
        if signum is not None:
            previous[signum] = signal.signal(signum, _raise_interrupt)
    return previous


def main(argv: Optional[list[str]] = None, make_plant: Optional[PlantFactory] = None) -> int:
    args = build_parser().parse_args(argv)
    origin = Origin(args.origin_lat, args.origin_lon, args.origin_alt, args.heading)
    previous_handlers = _install_stop_signals()
    transport: Optional[Transport] = None
    plant: Optional[Plant] = None
    try:
        try:
            step_rate_hz(args.integration_dt_s)                    # a step that is not 1/N s is refused
            start, launch = launch_option(args, origin)
            # The physics first: once the board port listens, a client finds JSBSim running.
            plant = (make_plant or _plant_factory(args))(args.model_dir, origin, args.integration_dt_s,
                                                         plant_options(args, start))
            transport = make_transport(args.device, args.baud)      # binds a tcpin port: refused if busy
        except (ValueError, RuntimeError, OSError) as exc:          # FileNotFoundError is an OSError
            print(f"bridge: {exc}", file=sys.stderr)
            return EXIT_BAD_SETUP
        return _serve(args, transport, plant, launch)
    finally:
        if plant is not None:
            plant.close()
        if transport is not None:
            transport.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def _serve(args: argparse.Namespace, transport: Transport, plant: Plant, launch: Launch) -> int:
    stepper = BoardClockStepper(plant, launch, wind_option(args), loop_rate_hz=args.loop_rate_hz,
                                ground_alt_msl_m=args.origin_alt)
    now = datetime.now(timezone.utc)
    log_path = args.log_csv or _default_log_path(now)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    run_info = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    run_info["started_utc"] = now.isoformat()
    log_path.with_suffix(".json").write_text(json.dumps(run_info, indent=1))
    link = MavlinkLink(transport, args.reconnect_interval_s)         # the client grace time starts here
    stop = StopConditions(link, args.client_grace_s)
    if args.exit_on_stdin_eof:
        stop.watch_stdin(lambda: os.read(sys.stdin.fileno(), 4096))
    log = LinkLog(log_path.open("w", newline=""))
    print(f"bridge: {args.device} <-> {args.model_dir.name} ({args.plant}), log {log_path}", flush=True)
    start = time.perf_counter()
    code = EXIT_OK
    try:
        run_bridge(link, Bridge(stepper), log, lambda: time.perf_counter() - start, stop)
        print(f"bridge: stopping: {stop.reason}")
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:                 # the physics failed (e.g. the JSBSim program stopped answering)
        print(f"bridge: physics failed: {exc}", file=sys.stderr)
        code = EXIT_PHYSICS_FAILED
    finally:
        link.close()
        log.close()
        print(f"bridge: {log.rows} frames logged, {stepper.reinits} resets to the origin "
              f"({stepper.armed_reinits} while armed), {stepper.gaps} gaps stepped across, "
              f"{stepper.retransmits} frames resent, {stepper.conflicts} conflicting frames, {stepper.crashes} crashes, "
              f"link {link.stats}")
    return code
