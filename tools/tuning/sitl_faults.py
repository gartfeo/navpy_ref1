"""Fault injection for the C++ flight tuner (AP_FlightTune) in ArduPlane SITL.

For each fault the tuner is started in LOITER on one axis and flown until a check run with
the new gains has them installed (the first such run, or the second for some faults). Then
the fault is injected. The tuner must stop with the expected reason, the old gains must be
back when read STOP_WAIT_S later, and they must stay back.
AUTOTUNE tunes while it runs, so for it only the gains it gives back when switched off are
checked: they show what it started from.

Each case runs on its own axis (six on roll, one on pitch) unless --axis puts them all on one axis. The
tuner freezes both axes' settings and watches the mode, the sticks and AUTOTUNE whichever axis it tunes,
so every case keeps its expected stop reason on either axis. An axis that never reaches a check run with
new gains cannot inject its faults: round 2's roll stopped on "model does not fit" (Peron15_hitl2) and its
six roll cases failed, while the same six on pitch (--axis pitch) passed. Like sitl_driver's runs, the
run fails below --min-height once the takeoff has reached TKOFF_ALT.

Usage (from WSL, in the repository root; --board flies a flight controller, see hitl.py):
    python3 -m tools.tuning.sitl_faults --workdir ~/tune_sitl/faults --speedup 2 [--axis pitch]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, replace
from typing import Callable

from pymavlink import mavutil

from tools.tuning.flight_steps import FTUNE_SWITCH_CH, FlightCheckFailed, HeightFloor, add_floor_arg, wait_text
from tools.tuning.hitl import add_vehicle_args, fly_and_record, open_vehicle, run_file, write_run_params
from tools.tuning.sitl_driver import FIRMWARE_PARAMS, takeoff_to_fbwa
from tools.tuning.sitl_sim import FIRMWARE_BINARY, Sitl

AXIS_PREFIX = {"roll": "RLL", "pitch": "PTCH"}
AXIS_BIT = {"roll": 1, "pitch": 2}     # FTUN_AXES
GAIN_TOL = 1e-4             # relative; PARAM_VALUE carries the float itself
WAIT_CANDIDATE_S = 400      # simulated seconds from the switch to the first new-gain check run
STOP_WAIT_S = 2             # simulated seconds for the stop message after a fault
STAY_S = 3                  # simulated seconds after the fault before the gains are read again
AUTOTUNE_ON_S = 1           # simulated seconds AUTOTUNE runs before it is switched off again
BOOT_PARAMS = {**FIRMWARE_PARAMS, "FTUN_AXES": 1, "AUTOTUNE_LEVEL": 0}


def gain_names(axis: str) -> list[str]:
    return [f"{AXIS_PREFIX[axis]}_RATE_{k}" for k in ("FF", "P", "I", "D")]


def same_gains(a: dict, b: dict) -> bool:
    return all(abs(a[n] - b[n]) <= GAIN_TOL * max(1.0, abs(b[n])) for n in b)


def rate_gains(sitl: Sitl, axis: str) -> dict:
    return {n: sitl.get_param(n) for n in gain_names(axis)}


def wait_candidate(sitl: Sitl, axis: str, old: dict, b_run: int) -> bool:
    """Fly until check run b_run has the new gains installed (the rate gains differ from old)."""
    mark = f"check {b_run}/4 (new gains)"
    text = wait_text(sitl, (mark, "FTUN: stopped", "FTUN: done"), WAIT_CANDIDATE_S)
    if text is None or mark not in text:
        return False
    end = time.time() + WAIT_CANDIDATE_S / max(1, sitl.speedup)
    while time.time() < end:
        if not same_gains(rate_gains(sitl, axis), old):
            return True
        sitl.fly_for(0.2)
    return False


def autotune_command(sitl: Sitl, on: bool) -> None:
    sitl.mav.mav.command_long_send(sitl.mav.target_system, sitl.mav.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, 0,
                                   1 if on else 0, 0, 0, 0, 0, 0, 0)


Undo = Callable[[Sitl], None]


@dataclass
class Fault:
    name: str
    expect: str                         # the stop reason the tuner must report
    inject: Callable[[Sitl], Undo]      # injects the fault, returns how to undo it
    # False when another controller owns the gains right after the fault (AUTOTUNE tunes
    # while it runs); then only the gains it gives back afterwards are checked
    tuner_owns_after: bool = True
    axis: str = "roll"
    b_run: int = 2                      # check run 2 or 4: the first or second with new gains


def _switch_off(sitl: Sitl) -> Undo:
    sitl.tx.set(FTUNE_SWITCH_CH, 1000)
    return lambda s: None


def _mode_change(sitl: Sitl) -> Undo:
    sitl.set_mode("CRUISE")
    return lambda s: s.set_mode("LOITER")


def _roll_setting(sitl: Sitl) -> Undo:
    fltd = sitl.get_param("RLL_RATE_FLTD")
    sitl.set_param("RLL_RATE_FLTD", fltd + 2)
    return lambda s: s.set_param("RLL_RATE_FLTD", fltd)


def _stick(sitl: Sitl) -> Undo:
    sitl.tx.set(1, 1700)
    return lambda s: s.tx.set(1, 1500)


def _autotune(sitl: Sitl) -> Undo:
    autotune_command(sitl, True)

    def undo(s: Sitl) -> None:
        # AP_AutoTune::stop gives back the gains it started from: they must be the old ones
        s.fly_for(AUTOTUNE_ON_S)
        autotune_command(s, False)
    return undo


FAULTS = [
    Fault("switch off", "switch off", _switch_off),
    Fault("mode change", "mode changed", _mode_change),
    Fault("roll setting", "roll settings changed", _roll_setting),
    Fault("stick", "stick input", _stick),
    Fault("pitch: switch off", "switch off", _switch_off, axis="pitch"),
    Fault("second new-gain run: mode change", "mode changed", _mode_change, b_run=4),
    # last: AUTOTUNE may adjust IMAX and SMAX when it starts
    Fault("AUTOTUNE over MAVLink", "AUTOTUNE", _autotune, tuner_owns_after=False),
]


def cases(axis: str | None) -> list[Fault]:
    """FAULTS each on its own axis, or with `axis` all on that one. A case that then repeats an earlier one
    (the same fault, stop reason, check run and gain owner) is left out: on one axis "pitch: switch off"
    is "switch off" again."""
    if axis is None:
        return list(FAULTS)
    kept: list[Fault] = []
    for fault in FAULTS:
        moved = replace(fault, axis=axis)
        key = (moved.inject, moved.expect, moved.b_run, moved.tuner_owns_after)
        if all((k.inject, k.expect, k.b_run, k.tuner_owns_after) != key for k in kept):
            kept.append(moved)
    return kept


def run_fault(sitl: Sitl, fault: Fault) -> dict:
    sitl.tx.set(FTUNE_SWITCH_CH, 1000)
    sitl.fly_for(3)
    sitl.set_mode("LOITER")
    sitl.set_param("FTUN_AXES", AXIS_BIT[fault.axis])
    sitl.fly_for(10)
    old = rate_gains(sitl, fault.axis)
    print(f"[faults] {fault.name}: tuning switch high", flush=True)
    sitl.tx.set(FTUNE_SWITCH_CH, 2000)
    if not wait_candidate(sitl, fault.axis, old, fault.b_run):
        return {"fault": fault.name, "axis": fault.axis, "pass": False, "why": "no check run with new gains"}
    seen = len(sitl.messages)
    undo = fault.inject(sitl)
    sitl.fly_for(STOP_WAIT_S)
    stops = [m for m in sitl.messages[seen:] if "FTUN: stopped" in m]
    right_after = rate_gains(sitl, fault.axis)
    undo(sitl)
    sitl.fly_for(STAY_S)
    later = rate_gains(sitl, fault.axis)
    result = {
        "fault": fault.name,
        "axis": fault.axis,
        "stop": stops[0] if stops else None,
        "reason_ok": bool(stops) and fault.expect in stops[0],
        "back_within_stop_wait": same_gains(right_after, old),
        "stay_back": same_gains(later, old),
        "old": old, "right_after": right_after, "later": later,
    }
    back_ok = result["back_within_stop_wait"] or not fault.tuner_owns_after
    result["pass"] = result["reason_ok"] and back_ok and result["stay_back"]
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workdir", required=True)
    add_vehicle_args(ap)
    add_floor_arg(ap)
    ap.add_argument("--axis", choices=tuple(AXIS_PREFIX),
                    help="fly every case on this axis (default: each case on its own axis, six roll and one pitch)")
    args = ap.parse_args()
    sitl = open_vehicle(args, [write_run_params(args.workdir, BOOT_PARAMS)], [], FIRMWARE_BINARY)
    results: list[dict] = []
    record = {"axis": args.axis, "results": results}

    def flight() -> None:
        sitl.tx.set(FTUNE_SWITCH_CH, 1000)
        sitl.fly_for(5)
        takeoff_to_fbwa(sitl)
        with HeightFloor(sitl, args.min_height, record):
            for fault in cases(args.axis):
                results.append(run_fault(sitl, fault))
                print(f"[faults] {fault.name} ({fault.axis}): {'pass' if results[-1]['pass'] else 'FAIL'}",
                      flush=True)
    try:
        link_failed = fly_and_record(sitl, flight, record, run_file(args.workdir, ".json"))
    except FlightCheckFailed as exc:
        print(f"[faults] run FAILED: {exc}", flush=True)
        sys.exit(1)
    print(json.dumps([{k: r.get(k) for k in ("fault", "axis", "stop", "pass", "why")} for r in results], indent=1))
    sys.exit(0 if results and all(r["pass"] for r in results) and not link_failed else 1)


if __name__ == "__main__":
    main()
