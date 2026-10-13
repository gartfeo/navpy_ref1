"""Check the Python controller port against a log with PIDR/PIDP at loop rate.

Feeds the logged rate demand (Tar), gyro rate (Act) and airspeed into
ap_controller.RateController with the logged gains, and compares the computed
FF, P, I, D and Dmod with the logged values.

Usage:
    python -I tools/tuning/port_check.py LOG.BIN [--axis roll|pitch]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.tuning.ap_controller import Gains, RateController, speed_scaler  # noqa: E402
from tools.tuning.logdata import FlightLog, standard_log  # noqa: E402

WARMUP_S = 3.0          # filter and slew-limiter states are unknown at replay start
MAX_GAP_S = 0.02        # a longer gap between PID rows restarts the replay


def flying_segments(t_us: np.ndarray, log: FlightLog, min_len_s: float = 10.0) -> list[tuple[int, int]]:
    """Index ranges of continuous rows while STAT.isFlying is set."""
    if log.has("STAT"):
        fly = np.interp(t_us, log.get("STAT", "TimeUS"), log.get("STAT", "isFlying")) > 0.5
    else:
        fly = np.ones(len(t_us), dtype=bool)
    gaps = np.diff(t_us) > MAX_GAP_S * 1e6
    segments, start = [], None
    for k in range(len(t_us)):
        if fly[k] and start is None:
            start = k
        end_here = (not fly[k]) or (k < len(gaps) and gaps[k]) or k == len(t_us) - 1
        if start is not None and end_here:
            if (t_us[k] - t_us[start]) / 1e6 >= min_len_s:
                segments.append((start, k + 1))
            start = None
    return segments


def replay(log: FlightLog, axis: str, only_mode: int | None = None) -> dict:
    mtype = "PIDR" if axis == "roll" else "PIDP"
    t = log.get(mtype, "TimeUS")
    prm = log.params
    loop_hz = prm.get("SCHED_LOOP_RATE", 50.0)
    dt = 1.0 / loop_hz
    as_t, as_v = log.get("CTUN", "TimeUS"), log.get("CTUN", "As")
    e2t_v = log.get("CTUN", "E2T")
    a_min, a_max = prm["AIRSPEED_MIN"], prm["AIRSPEED_MAX"]
    results = {"segments": []}
    terms = ("FF", "P", "I", "D", "Dmod")
    errs = {k: [] for k in terms}
    refs = {k: [] for k in terms}
    worst = []
    mode_ok = None
    if only_mode is not None and log.has("MODE"):
        mt, mn = log.get("MODE", "TimeUS"), log.get("MODE", "ModeNum")
        idx = np.searchsorted(mt, t, side="right") - 1
        mode_ok = (idx >= 0) & (mn[np.maximum(idx, 0)] == only_mode)
    for s, e in flying_segments(t, log):
        g = Gains.from_params({k: log.param_at(k, t[s], prm.get(k)) for k in prm}, axis)
        ctl = RateController(axis, g, 1, a_min, a_max)
        ctl.pid.integrator = np.array([log.get(mtype, "I")[s] / (180 / np.pi)])
        seeded = False
        for k in range(s, e):
            aspeed = float(np.interp(t[k], as_t, as_v))
            e2t = float(np.interp(t[k], as_t, e2t_v))
            scaler = speed_scaler(aspeed, prm.get("SCALING_SPEED", 15.0), a_min, a_max)
            in_warmup = (t[k] - t[s]) / 1e6 < WARMUP_S
            if not in_warmup and not seeded:
                # the integrator is logged, so take it from the log once the filters have settled
                ctl.pid.integrator = np.array([log.get(mtype, "I")[k - 1] / (180 / np.pi)])
                seeded = True
            ctl.rate_out(np.array([log.get(mtype, "Tar")[k]]), np.radians(log.get(mtype, "Act")[k]),
                         scaler, e2t, np.array([aspeed]), dt, int(t[k] // 1000))
            if in_warmup or (mode_ok is not None and not mode_ok[k]):
                continue
            info = ctl.info
            for name in terms:
                ref = log.get(mtype, name)[k]
                errs[name].append(float(getattr(info, name)[0]) - ref)
                refs[name].append(ref)
            worst.append((abs(float(info.FF[0]) - log.get(mtype, "FF")[k]), t[k] / 1e6, aspeed,
                          float(info.FF[0]), log.get(mtype, "FF")[k]))
        results["segments"].append(((t[s] / 1e6), (t[e - 1] / 1e6)))
    for name in terms:
        err, ref = np.array(errs[name]), np.array(refs[name])
        if len(ref):
            results[name] = {"rms_err": float(np.sqrt(np.mean(err ** 2))),
                             "rms_ref": float(np.sqrt(np.mean(ref ** 2))),
                             "max_abs_err": float(np.max(np.abs(err))), "n": int(len(ref))}
    results["worst_ff"] = sorted(worst, reverse=True)[:5]
    return results


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--axis", choices=("roll", "pitch", "both"), default="both")
    ap.add_argument("--mode", type=int, default=None, help="compare only in this flight mode number")
    a = ap.parse_args()
    log = standard_log(a.log)
    for axis in (("roll", "pitch") if a.axis == "both" else (a.axis,)):
        r = replay(log, axis, a.mode)
        print(f"{axis}: segments {[(round(x), round(y)) for x, y in r['segments']]}")
        for name in ("FF", "P", "I", "D", "Dmod"):
            if name in r:
                v = r[name]
                print(f"  {name:4s} rms err {v['rms_err']:.5f}  (signal rms {v['rms_ref']:.4f})  "
                      f"max err {v['max_abs_err']:.5f}  n={v['n']}")
        for err, ts, aspeed, ff_port, ff_log in r["worst_ff"]:
            print(f"  worst FF: t={ts:.2f}s airspeed={aspeed:.1f} port={ff_port:.3f} log={ff_log:.3f}")


if __name__ == "__main__":
    main()
