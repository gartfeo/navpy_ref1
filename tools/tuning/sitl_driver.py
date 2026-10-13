"""Fly an ArduPlane SITL instance as pilot (RC transmitter) and ground station.

Runs inside WSL with the Linux python3 that has pymavlink. The SITL process, its
MAVLink connection and the simulated RC receiver are in sitl_sim.py; this module
holds the flight subcommands; the pilot's steps are in flight_steps.py and the vs-autotune
procedure in vs_autotune.py. port-check, inflight-tune --firmware and vs-autotune --firmware also
fly a flight controller with simulation firmware (--board, hitl.py). Once the takeoff has reached
TKOFF_ALT these runs fail (exit 1, the reason in the run's JSON under "error") when the aircraft goes
below --min-height; vs-autotune also fails when a climb, the throttle match or the tuner fails.

Usage (from WSL, in the repository root):
    python3 -m tools.tuning.sitl_driver port-check --workdir ~/tune_sitl
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from tools.tuning.flight_steps import (CLIMB_MARGIN_M, FTUNE_SWITCH_CH, PILOT_THROTTLE_PWM, TEST_HEIGHT_M,
                                       THROTTLE_CH, CruisePitchTrim, FlightCheckFailed, HeightFloor, add_floor_arg,
                                       climb_to, pilot_throttle, read_gains, stick_sequence)
from tools.tuning.hitl import add_vehicle_args, fly_and_record, open_vehicle, run_file, write_run_params
from tools.tuning.sitl_sim import DEFAULT_BINARY, DEFAULT_INSTANCE, FIRMWARE_BINARY, Sitl
from tools.tuning.tuner_legs import PATTERN_STRAIGHT, PATTERNS, fly_tuner
from tools.tuning.vehicle import Vehicle
from tools.tuning.vs_autotune import fly_comparison

HERE = Path(__file__).resolve().parent
TAKEOFF_TIMEOUT_S = 180       # simulated seconds from arming to TKOFF_ALT: ground roll or launch, then the climb
# TAKEOFF mode ends its climb 2 m short of TKOFF_ALT (mode_takeoff.cpp), then TECS may sag a few metres
TAKEOFF_ALT_MARGIN_M = 10
LAUNCH_TEXT = "Triggered AUTO"     # ArduPlane releases the throttle in TAKEOFF mode (takeoff.cpp:120)


def takeoff(sitl: Vehicle) -> float:
    """TAKEOFF mode to TKOFF_ALT; returns TKOFF_ALT. The motor starts when ArduPlane's takeoff check
    passes: at once on SITL's runway (TKOFF_THR_MINACC 0), or after a catapult's launch acceleration
    (TKOFF_THR_MINACC > 0, as on the aircraft; the HITL bridge's catapult fires a set delay after the
    first armed frame, so the mode is set before arming)."""
    height = sitl.get_param("TKOFF_ALT")
    sitl.wait_ready()
    sitl.set_mode("TAKEOFF")
    seen = len(sitl.messages)
    sitl.arm()
    deadline = time.time() + TAKEOFF_TIMEOUT_S / max(1, sitl.speedup)
    while time.time() < deadline:
        s = sitl.state()
        if s["alt"] is not None and s["alt"] > height - TAKEOFF_ALT_MARGIN_M:
            break
    else:
        launched = any(LAUNCH_TEXT in m for m in sitl.messages[seen:])
        raise RuntimeError(f"takeoff did not reach TKOFF_ALT {height:g} m" + (
            "" if launched else f": the motor never started (no '{LAUNCH_TEXT}'); with TKOFF_THR_MINACC set it "
                                "waits for a launch acceleration"))
    return height


def takeoff_to_fbwa(sitl: Vehicle, throttle_pwm: int = PILOT_THROTTLE_PWM) -> None:
    """takeoff, a LOITER climb to TEST_HEIGHT_M when TKOFF_ALT is lower, then FBWA with the pilot's throttle."""
    if takeoff(sitl) < TEST_HEIGHT_M - CLIMB_MARGIN_M:
        climb_to(sitl, TEST_HEIGHT_M)
    sitl.tx.set(THROTTLE_CH, throttle_pwm)
    sitl.set_mode("FBWA")
    sitl.fly_for(10)


def port_check(args: argparse.Namespace) -> int:
    """Fly pilot doublets in FBWA so PIDR/PIDP are logged at loop rate, at the aircraft's level cruise: the
    throttle and the pitch a straight CRUISE leg holds at AIRSPEED_CRUISE, the pitch as PTCH_TRIM_DEG for this
    run only (flight_steps.CruisePitchTrim). At the fixed pilot stick FBWA flew the HITL aircraft several m/s
    faster than its cruise, and at the cruise throttle without the trim still too fast."""
    sitl = open_vehicle(args, [], [], DEFAULT_BINARY)
    record: dict = {}

    def flight() -> None:
        takeoff_to_fbwa(sitl)
        record["pilot_throttle"] = pilot = pilot_throttle(sitl)
        with HeightFloor(sitl, args.min_height, record), CruisePitchTrim(sitl, pilot, record):
            for _ in range(3):
                stick_sequence(sitl, 1, [(1800, 2.0), (1200, 2.0), (1500, 3.0)])
                stick_sequence(sitl, 2, [(1650, 1.5), (1350, 1.5), (1500, 3.0)])
            stick_sequence(sitl, 1, [(1650, 1.0), (1350, 1.0), (1700, 0.5), (1300, 0.5), (1500, 3.0)])
            sitl.fly_for(5)
    code = fly_and_record(sitl, flight, record, run_file(args.workdir, ".json"))    # a board adds its report
    print("logs:", record["logs"])
    return code


CHUTE_PARAMS = {"SERVO5_FUNCTION": 94}          # Script1 output for chute_deploy.lua (CHUTE_SCR_FN 1)


def chute_conflict(args: argparse.Namespace) -> None:
    """Trigger chute_deploy.lua in FBWA level flight, with or without the old tune_pitch_roll.lua loaded."""
    repo = HERE.parents[1]
    scripts = [repo / "scripts" / "lua" / "chute_deploy.lua"]
    if args.with_old_tuner:
        scripts.append(repo / "scripts" / "lua" / "tune_pitch_roll.lua")
    extra = Path(args.workdir).expanduser().parent / "chute_extra.parm"
    extra.parent.mkdir(parents=True, exist_ok=True)
    extra.write_text("".join(f"{k} {v}\n" for k, v in {"SCR_ENABLE": 1, **CHUTE_PARAMS}.items()))
    sitl = Sitl(Path(args.workdir), instance=args.instance, speedup=args.speedup, extra_params=[extra],
                scripts=scripts)
    try:
        sitl.start()
        sitl.tx.set(7, 1000)                      # chute switch low
        sitl.tx.set(9, 1000)                      # old tuner trigger low (tuning "disabled")
        sitl.tx.set(11, 1500)
        sitl.fly_for(5)
        if args.with_old_tuner:
            sitl.set_param("MTUNE_TRG_CH", 9)       # the aircraft files use channel 9; default 7 is the chute channel
            sitl.set_param("MTUNE_TGL_INT_CH", 11)
        sitl.set_param("CHUTE_RC_CHAN", 7)
        sitl.set_param("CHUTE_MIN_ALT", 50)
        sitl.set_param("CHUTE_POST_MODE", 5)
        takeoff_to_fbwa(sitl)
        sitl.fly_for(10)
        print("[driver] chute switch high", flush=True)
        sitl.tx.set(7, 2000)
        sitl.fly_for(12)
    finally:
        sitl.stop()
    print("logs:", [str(p) for p in sitl.logs()])


FTUNE_SCRIPT = HERE / "lua" / "flight_tune.lua"
FTUNE_PARAMS = {"SCR_ENABLE": 1, f"RC{FTUNE_SWITCH_CH}_OPTION": 300, "WP_LOITER_RAD": 150}
# the C++ tuner (AP_FlightTune) is built into FIRMWARE_BINARY (sitl_sim.py)
FIRMWARE_PARAMS = {f"RC{FTUNE_SWITCH_CH}_OPTION": 240, "WP_LOITER_RAD": 150, "FTUN_ENABLE": 1}


def tuner_setup(args: argparse.Namespace, extra_params: dict) -> tuple[dict, list[Path], Path]:
    """Boot parameters, scripts and binary for the Lua tuner or, with --firmware, the C++ one."""
    if args.firmware:
        return {**FIRMWARE_PARAMS, **extra_params}, [], FIRMWARE_BINARY
    return {**FTUNE_PARAMS, **extra_params}, [FTUNE_SCRIPT], DEFAULT_BINARY


def inflight_tune(args: argparse.Namespace) -> int:
    """Run the flight tuner (the C++ one with --firmware, else tools/tuning/lua/flight_tune.lua) on straight
    CRUISE legs, one axis per leg with a turn back between them (--pattern straight, tuner_legs.py), or in
    LOITER (--pattern loiter); print the gains before and after."""
    params, scripts, binary = tuner_setup(args, dict(args.param or []))
    sitl = open_vehicle(args, [write_run_params(args.workdir, params)], scripts, binary)
    result: dict = {}

    def flight() -> None:
        sitl.tx.set(FTUNE_SWITCH_CH, 1000)
        sitl.fly_for(5)
        sitl.set_param("FTUN_ENABLE", 1)
        sitl.set_param("FTUN_AXES", args.axes)
        takeoff_to_fbwa(sitl)
        with HeightFloor(sitl, args.min_height, result):
            result["before"] = read_gains(sitl)
            result["pattern"] = args.pattern
            result["sessions"] = fly_tuner(sitl, args.axes, args.pattern, args.timeout)
            result["end"] = result["sessions"][-1]["end"]
            result["after"] = read_gains(sitl)
    code = fly_and_record(sitl, flight, result, run_file(args.workdir, ".json"), kept_texts=("FTUN",))
    print(json.dumps(result, indent=1))
    return code


def vs_autotune(args: argparse.Namespace) -> int:
    """AUTOTUNE (level --level, 0 keeps TCONST/RMAX), then the flight tuner from the AUTOTUNE gains, both on
    --axes, then the same FBWA card flown A-B-A-B with both gain sets (vs_autotune.py)."""
    params, scripts, binary = tuner_setup(args, {"AUTOTUNE_LEVEL": args.level, "AUTOTUNE_AXES": args.axes})
    sitl = open_vehicle(args, [write_run_params(args.workdir, params)], scripts, binary)
    result: dict = {"axes": args.axes}

    def flight() -> None:
        sitl.tx.set(FTUNE_SWITCH_CH, 1000)
        sitl.fly_for(5)
        sitl.set_param("FTUN_ENABLE", 1)
        sitl.set_param("FTUN_ALT_MIN", args.ftun_alt_min)
        sitl.set_param("FTUN_AXES", args.axes)
        height = takeoff(sitl)
        with HeightFloor(sitl, args.min_height, result):
            fly_comparison(sitl, result, height, args.axes, args.autotune_s, args.timeout, args.min_height)
    code = fly_and_record(sitl, flight, result, run_file(args.workdir, ".json"), kept_texts=("FTUN", "Finished"))
    print(json.dumps(result, indent=1))
    return code


def _param_pair(text: str) -> tuple[str, float]:
    name, value = text.split("=")
    return name, float(value)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("port-check", help=port_check.__doc__)
    p.add_argument("--workdir", default="~/tune_sitl/port_check")
    add_vehicle_args(p)
    add_floor_arg(p)
    p.set_defaults(func=port_check)
    c = sub.add_parser("chute-conflict", help=chute_conflict.__doc__)
    c.add_argument("--workdir", required=True)
    c.add_argument("--instance", type=int, default=DEFAULT_INSTANCE)
    c.add_argument("--speedup", type=int, default=1)
    c.add_argument("--with-old-tuner", action="store_true")
    c.set_defaults(func=chute_conflict)
    f = sub.add_parser("inflight-tune", help=inflight_tune.__doc__)
    f.add_argument("--workdir", required=True)
    add_vehicle_args(f)
    add_floor_arg(f)
    f.add_argument("--axes", type=int, default=3, help="FTUN_AXES: 1 roll, 2 pitch, 3 both")
    f.add_argument("--timeout", type=float, default=900, help="simulated seconds to wait for the tune")
    f.add_argument("--param", type=_param_pair, action="append", help="extra NAME=VALUE at boot")
    f.add_argument("--firmware", action="store_true", help="the C++ tuner build instead of the Lua script")
    f.add_argument("--pattern", choices=PATTERNS, default=PATTERN_STRAIGHT,
                   help="straight: a CRUISE leg per axis, turning back between them; loiter: one session in LOITER")
    f.set_defaults(func=inflight_tune)
    v = sub.add_parser("vs-autotune", help=vs_autotune.__doc__)
    v.add_argument("--workdir", required=True)
    add_vehicle_args(v)
    add_floor_arg(v)
    v.add_argument("--axes", type=int, choices=(1, 2, 3), default=3,
                   help="AUTOTUNE_AXES and FTUN_AXES alike: 1 roll, 2 pitch, 3 both (default 3)")
    v.add_argument("--level", type=int, default=0, help="AUTOTUNE_LEVEL; 0 keeps TCONST and RMAX")
    v.add_argument("--autotune-s", type=float, default=240,
                   help="simulated seconds of AUTOTUNE doublets per axis, climbs not counted")
    v.add_argument("--ftun-alt-min", type=float, default=40)
    v.add_argument("--timeout", type=float, default=1800)
    v.add_argument("--firmware", action="store_true", help="the C++ tuner build instead of the Lua script")
    v.set_defaults(func=vs_autotune)
    args = ap.parse_args()
    try:
        code = args.func(args)           # 1: a board run whose HITL link failed its health gate
    except FlightCheckFailed as exc:
        print(f"[driver] run FAILED: {exc}", flush=True)
        code = 1
    sys.exit(code or 0)


if __name__ == "__main__":
    main()
