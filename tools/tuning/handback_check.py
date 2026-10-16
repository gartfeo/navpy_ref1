"""Hand-back gate for the flight tuner's pitch holds: TECS's throttle when control comes back.

While the tuner flies pitch (AP_FlightTune::flies_pitch: the pitch axis from the end of its levelling stage to
the end of the manoeuvre: FTUN records with Ax 1 and Ph 2-5; Ph 6, the hand-back of the bank after a manoeuvre,
flies TECS's pitch again) ArduPlane holds TECS instead of updating it
(AP_TECS::hold_states from ArduPlane.cpp:646-654): no TECS record is written and the throttle stays at the
tuner's held value. TECS updates at 10 Hz (ArduPlane.cpp:72), so samples taken between the hand-back and
TECS's first update still show the held throttle. Round 2's check sampled there and reported no step
while TECS's first output dipped far below the held throttle.

This gate therefore starts at the first TECS record after each hand-back and takes the extremes of TECS.th
and CTUN.ThD (TECS's throttle demand, Log.cpp:94-114) over the next `updates` TECS updates. A bumpless
hand-back leaves the held throttle (CTUN.ThO at the hand-back) no faster than TECS's own slew limit on its
throttle (AP_TECS.cpp:792-798): one step of update interval x (THR_MAX - max(THR_MIN, 0)) x THR_SLEWRATE / 100
per update, the first included, plus TOL_PCT. A sample belongs to the update its time after the first record
rounds to, in update intervals.

Usage (exit 1 when a hand-back fails or none can be checked):
    python -m tools.tuning.handback_check --log <log.BIN> [--updates N] [--max-step-pct P] [--out result.json]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from tools.tuning.logdata import FlightLog, read_log

LOG_TYPES = {"FTUN": ("TimeUS", "Ax", "Ph"), "TECS": ("TimeUS", "th"), "CTUN": ("TimeUS", "ThO", "ThD")}
PITCH_AXIS = 1                 # FTUN.Ax: cur_axis(), 0 roll, 1 pitch
# FTUN.Ph (AP_FlightTune.cpp record_outputs): 1 LEVEL, 2 SETTLE, 3 a test's RUN, 4 and 5 a check's RUN with old
# and new gains, 6 the hand-back after a manoeuvre (the bank moved back to the mode's, on TECS's pitch and
# throttle). flies_pitch() holds from the end of LEVEL to the end of the RUN; TECS is back from the hand-back on.
HOLD_PHASES = (2, 3, 4, 5)
# Hold records are 5 ms apart (FTUN is written every main loop at 200 Hz while a manoeuvre runs) and manoeuvres
# start at least GAP_BETWEEN_MS = 3 s apart (AP_FlightTune.cpp:28): a larger gap separates two holds.
HOLD_GAP_S = 0.1
# TECS's first update comes within one update interval of the hand-back; two allow for a late slot.
FIRST_UPDATE_INTERVALS = 2
# On top of the slew steps: the throttle integrator's change within an update (AP_TECS.cpp:772, added after
# the slew limit at :802; under 1 % per update after round 2's pitch hand-backs) and rounding.
TOL_PCT = 1.0
DEFAULT_UPDATES = 1
US = 1e6


def hold_ends(log: FlightLog) -> list[float]:
    """TimeUS of every hand-back: the last FTUN record of each run of pitch-hold records."""
    if not log.has("FTUN"):
        return []
    t = log.get("FTUN", "TimeUS")
    held = (log.get("FTUN", "Ax") == PITCH_AXIS) & np.isin(log.get("FTUN", "Ph"), HOLD_PHASES)
    times = t[held]
    if not len(times):
        return []
    last = np.flatnonzero(np.diff(times) > HOLD_GAP_S * US)
    return [float(times[i]) for i in last] + [float(times[-1])]


def update_interval_s(log: FlightLog) -> float | None:
    """TECS's update interval: the median spacing of its records (holds leave gaps; the median ignores them)."""
    t = log.get("TECS", "TimeUS") if log.has("TECS") else np.zeros(0)
    return float(np.median(np.diff(t))) / US if len(t) > 1 else None


def slew_step_pct(params: dict, interval_s: float) -> float | None:
    """TECS's largest throttle change in one update, percent (AP_TECS.cpp:792-798); None without a slew limit."""
    rate, thr_max, thr_min = (params.get(n) for n in ("THR_SLEWRATE", "THR_MAX", "THR_MIN"))
    if rate is None or thr_max is None or thr_min is None or rate <= 0:
        return None
    return interval_s * (thr_max - max(thr_min, 0.0)) * rate / 100.0


def check_handback(log: FlightLog, end_us: float, interval_s: float, step_pct: float, updates: int,
                   tol_pct: float) -> dict:
    """One hand-back at end_us: the held throttle, the first TECS record after it, and the extremes of TECS.th and
    CTUN.ThD over `updates` updates from that record against the slew envelope. "problem" says why it fails."""
    out: dict = {"t_s": end_us / US}
    ctun_t = log.get("CTUN", "TimeUS")
    i = int(np.searchsorted(ctun_t, end_us, side="right")) - 1
    tecs_t = log.get("TECS", "TimeUS")
    j = int(np.searchsorted(tecs_t, end_us, side="right"))
    if i < 0:
        return {**out, "problem": "no CTUN record before the hand-back"}
    out["held_pct"] = held = float(log.get("CTUN", "ThO")[i])
    if j >= len(tecs_t) or tecs_t[j] - end_us > FIRST_UPDATE_INTERVALS * interval_s * US:
        return {**out, "problem": f"no TECS update within {FIRST_UPDATE_INTERVALS * interval_s:.2f} s"}
    t0 = tecs_t[j]
    out["first_update_after_s"] = (t0 - end_us) / US
    window_end = t0 + updates * interval_s * US
    in_tecs = (tecs_t >= t0) & (tecs_t < window_end)
    in_ctun = (ctun_t >= t0) & (ctun_t < window_end) & np.isfinite(log.get("CTUN", "ThD"))
    times = np.concatenate([tecs_t[in_tecs], ctun_t[in_ctun]])
    values = np.concatenate([log.get("TECS", "th")[in_tecs] * 100.0, log.get("CTUN", "ThD")[in_ctun]])
    allowed = (np.floor((times - t0) / (interval_s * US) + 0.5) + 1) * step_pct + tol_pct
    excess = np.abs(values - held) - allowed
    worst = int(np.argmax(excess))
    tecs, thd = values[: in_tecs.sum()], values[in_tecs.sum():]
    out.update({"tecs_pct": [float(tecs.min()), float(tecs.max())],
                "thd_pct": [float(thd.min()), float(thd.max())] if len(thd) else None,
                "deviation_pct": float(np.max(np.abs(values - held))), "excess_pct": float(excess[worst])})
    if excess[worst] > 0:
        out["problem"] = (f"throttle {values[worst]:.1f} % against {held:.1f} % held, "
                          f"{(times[worst] - end_us) / US:.2f} s after the hand-back: more than TECS's slew allows "
                          f"({allowed[worst]:.1f} %)")
    return out


def check_log(log: FlightLog, updates: int = DEFAULT_UPDATES, step_pct: float | None = None,
              tol_pct: float = TOL_PCT) -> dict:
    """The gate over every pitch hand-back in one log; ok only when there is one and all pass."""
    if updates < 1:
        raise ValueError(f"updates must be 1 or more, not {updates}")
    problems: list[str] = []
    interval = update_interval_s(log)
    ends = hold_ends(log)
    step = step_pct
    if step is None and interval is not None:
        step = slew_step_pct(log.params, interval)
    if not ends:
        problems.append("no pitch hold in the log (FTUN records with Ax 1 past levelling, Ph 2-5)")
    if interval is None:
        problems.append("fewer than 2 TECS records (LOG_BITMASK must log TECS)")
    elif step is None:
        problems.append("no TECS throttle slew limit (THR_SLEWRATE 0 or not logged): give --max-step-pct")
    handbacks = []
    if not problems:
        for end in ends:
            handbacks.append(check_handback(log, end, interval, step, updates, tol_pct))
            if "problem" in handbacks[-1]:
                problems.append(f"hand-back at {handbacks[-1]['t_s']:.2f} s: {handbacks[-1]['problem']}")
    return {"updates": updates, "update_interval_s": interval, "step_pct": step, "tol_pct": tol_pct,
            "handbacks": handbacks, "problems": problems, "ok": not problems}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--log", action="append", required=True, help="a DataFlash log (.BIN); repeatable")
    ap.add_argument("--updates", type=int, default=DEFAULT_UPDATES,
                    help="TECS updates checked from the first one after each hand-back (default %(default)s)")
    ap.add_argument("--max-step-pct", type=float,
                    help="throttle step allowed per update (default: TECS's slew limit from the log's parameters)")
    ap.add_argument("--tol-pct", type=float, default=TOL_PCT, help="added to the allowed change (default %(default)g)")
    ap.add_argument("--out", type=Path, help="also write the result here as JSON")
    args = ap.parse_args()
    result = {Path(p).name: check_log(read_log(p, LOG_TYPES), args.updates, args.max_step_pct, args.tol_pct)
              for p in args.log}
    text = json.dumps(result, indent=1)
    print(text)
    if args.out:
        args.out.write_text(text)
    sys.exit(0 if all(r["ok"] for r in result.values()) else 1)


if __name__ == "__main__":
    main()
