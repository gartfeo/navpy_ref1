"""Score attitude tracking on a flown test card from a DataFlash log.

Each attitude demand step (ATT DesRoll/DesPitch jumps) is compared with the ideal
response of ArduPlane's angle loop: rate = error / TCONST limited to RMAX, with a
perfect rate loop. The score is the mean absolute error against that ideal over a
window after the step, plus the overshoot. Windows between gain changes (PARM records)
are scored separately, so gain sets flown in turn (A-B-A-B) can be compared.

Usage (WSL or Windows python with pymavlink):
    python tools/tuning/card_eval.py <log.BIN> --axis roll --from-us 0 --to-us 9e18
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass

import numpy as np

from tools.tuning.logdata import read_log

STEP_MIN_DEG = {"roll": 8.0, "pitch": 3.0}     # a demand jump this large in one sample starts a step
STEP_WINDOW_S = 2.5                              # scored time after each step
MODE_SETTLE_S = 0.5                              # a step this soon after a mode change is the mode's transient
AXIS_FIELDS = {"roll": ("DesRoll", "Roll"), "pitch": ("DesPitch", "Pitch")}
AXIS_PARAMS = {"roll": ("RLL2SRV_TCONST", "RLL2SRV_RMAX", "RLL2SRV_RMAX"),
               "pitch": ("PTCH2SRV_TCONST", "PTCH2SRV_RMAX_UP", "PTCH2SRV_RMAX_DN")}


@dataclass
class StepScore:
    t_us: float
    size_deg: float
    mae_deg: float
    overshoot: float


def ideal_response(t_s: np.ndarray, demand: np.ndarray, start: float, tconst: float,
                   rmax_up: float, rmax_dn: float) -> np.ndarray:
    """Angle loop with a perfect rate loop, integrated on the log time base."""
    out = np.empty_like(demand)
    a = start
    tau = max(tconst, 0.05)
    up = rmax_up if rmax_up > 0 else np.inf
    dn = -rmax_dn if rmax_dn > 0 else -np.inf
    out[0] = a
    for k in range(1, len(t_s)):
        dt = t_s[k] - t_s[k - 1]
        n = max(1, int(np.ceil(dt / 0.0025)))
        h = dt / n
        for _ in range(n):
            a += h * min(max((demand[k - 1] - a) / tau, dn), up)
        out[k] = a
    return out


def score_steps(t_us: np.ndarray, demand: np.ndarray, angle: np.ndarray, axis: str,
                tconst: float, rmax_up: float, rmax_dn: float,
                mode_change_us: np.ndarray | None = None) -> list[StepScore]:
    """Steps whose window contains a mode change (from MODE_SETTLE_S before the jump to the
    end of the window) are skipped: their demand jump comes from the mode, not the pilot."""
    t_s = (t_us - t_us[0]) * 1e-6
    changes = np.asarray(mode_change_us if mode_change_us is not None else [], dtype=float)
    jumps = np.flatnonzero(np.abs(np.diff(demand)) >= STEP_MIN_DEG[axis]) + 1
    scores = []
    last = -np.inf
    for k in jumps:
        if t_s[k] - last < STEP_WINDOW_S:
            continue                    # one score per step, not per sample of a ramp
        end = np.searchsorted(t_s, t_s[k] + STEP_WINDOW_S)
        if end >= len(t_s):
            break
        lo, hi = t_us[k] - MODE_SETTLE_S * 1e6, t_us[k] + STEP_WINDOW_S * 1e6
        if np.any((changes >= lo) & (changes <= hi)):
            last = t_s[k]
            continue
        seg = slice(k - 1, end)
        ideal = ideal_response(t_s[seg], demand[seg], angle[k - 1], tconst, rmax_up, rmax_dn)
        err = angle[seg] - ideal
        size = demand[k] - demand[k - 1]
        over = max(0.0, float(np.max((angle[seg] - demand[k]) * np.sign(size))) / abs(size))
        scores.append(StepScore(float(t_us[k]), float(size), float(np.mean(np.abs(err))), over))
        last = t_s[k]
    return scores


def card_scores(path: str, axis: str, windows: list[tuple[float, float]]) -> list[dict]:
    """Score every step of axis inside each (start_us, end_us) window of the log."""
    log = read_log(path, {"ATT": ("TimeUS", "DesRoll", "Roll", "DesPitch", "Pitch"), "MODE": ("TimeUS", "ModeNum")})
    t = log.get("ATT", "TimeUS")
    mode_changes = log.get("MODE", "TimeUS")
    dem_f, ang_f = AXIS_FIELDS[axis]
    tc_n, up_n, dn_n = AXIS_PARAMS[axis]
    out = []
    for start, end in windows:
        m = (t >= start) & (t < end)
        if m.sum() < 10:
            out.append({"window": [start, end], "steps": []})
            continue
        tconst = log.param_at(tc_n, start)
        steps = score_steps(t[m], log.get("ATT", dem_f)[m], log.get("ATT", ang_f)[m], axis, tconst,
                            log.param_at(up_n, start), log.param_at(dn_n, start), mode_changes)
        out.append({"window": [start, end], "steps": [s.__dict__ for s in steps],
                    "mae": float(np.mean([s.mae_deg for s in steps])) if steps else None,
                    "overshoot": float(np.max([s.overshoot for s in steps])) if steps else None})
    return out


def marker_windows(path: str, marker: str, values: tuple[float, ...]) -> list[tuple[float, float, float]]:
    """(start_us, end_us, value) from each PARM record of marker with one of values to the next record."""
    log = read_log(path, {})
    changes = [(t, v) for t, n, v in log.param_history if n == marker]
    out = []
    for (t0, v0), (t1, _) in zip(changes, changes[1:] + [(9e18, None)]):
        if v0 in values:
            out.append((t0, t1, v0))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log")
    ap.add_argument("--axis", choices=("roll", "pitch"), required=True)
    ap.add_argument("--from-us", type=float, default=0)
    ap.add_argument("--to-us", type=float, default=9e18)
    ap.add_argument("--marker", help="score the windows marked by this parameter (sitl_driver vs-autotune: FTUN_REPS)")
    ap.add_argument("--values", type=float, nargs="*", default=[11, 12, 13, 14])
    args = ap.parse_args()
    if args.marker:
        wins = marker_windows(args.log, args.marker, tuple(args.values))
        scores = card_scores(args.log, args.axis, [(a, b) for a, b, _ in wins])
        for (_, _, v), sc in zip(wins, scores):
            sc["marker"] = v
    else:
        scores = card_scores(args.log, args.axis, [(args.from_us, args.to_us)])
    print(json.dumps(scores, indent=1))


if __name__ == "__main__":
    main()
