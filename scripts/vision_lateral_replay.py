#!/usr/bin/env python
"""Offline diagnostics for no-yaw pure-vision lateral navigation candidates.

This is not a flight model and must not feed navigation.  It uses existing
compact logs plus final closest-point scoring to reject obviously harmful
lateral-bias ideas before spending SITL time.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class FinalCase:
    name: str
    wind_speed: float
    wind_dir: float
    compact: Path
    signed_lateral_m: float
    dist_3d_m: Optional[float]
    passed_accuracy: Optional[bool]


@dataclass(frozen=True)
class CompactSample:
    ts: str
    t_s: float
    dist: float
    yaw_err: float
    cmd_r: float
    act_r: float
    cmd_p: Optional[float]
    act_p: Optional[float]


@dataclass(frozen=True)
class TerminalFeatures:
    count: int
    yaw_mean_deg: float
    yaw_integral_deg_s: float
    yaw_sign_flips: int
    cmd_roll_mean_deg: float
    act_roll_mean_deg: float
    cmd_roll_saturation_frac: float


@dataclass(frozen=True)
class CandidateScore:
    name: str
    mean_deg: float
    final_deg: float
    mean_abs_deg: float
    sign_agree_frac: Optional[float]
    harmful_frac: Optional[float]


@dataclass(frozen=True)
class CaseDiagnostics:
    case: FinalCase
    features: TerminalFeatures
    need_roll_sign: int
    baseline: CandidateScore
    candidates: List[CandidateScore]
    gates: List[Tuple[float, CompactSample, Dict[str, float]]]


def _to_float(value: object) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _to_bool(value: object) -> Optional[bool]:
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"true", "1", "yes"}:
        return True
    if text in {"false", "0", "no"}:
        return False
    return None


def _sign(value: float, deadband: float = 0.0) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


def _clip(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def _parse_ts_seconds(value: object) -> Optional[float]:
    numeric = _to_float(value)
    if numeric is not None:
        return numeric
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    parts = text.split(":")
    if len(parts) != 3:
        return None
    try:
        hours = int(parts[0])
        minutes = int(parts[1])
        seconds = float(parts[2])
    except ValueError:
        return None
    return hours * 3600.0 + minutes * 60.0 + seconds


def parse_final_csv(path: Path) -> List[FinalCase]:
    cases: List[FinalCase] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            compact_text = (row.get("compact") or "").strip()
            signed_lateral = _to_float(row.get("signed_lateral_m"))
            wind_speed = _to_float(row.get("wind_speed"))
            wind_dir = _to_float(row.get("wind_dir"))
            if not compact_text or signed_lateral is None or wind_speed is None or wind_dir is None:
                continue
            compact = Path(compact_text)
            if not compact.is_absolute():
                compact = (path.parent / compact).resolve()
            cases.append(
                FinalCase(
                    name=(row.get("name") or f"wind-{wind_speed:g}-dir-{wind_dir:g}").strip(),
                    wind_speed=wind_speed,
                    wind_dir=wind_dir,
                    compact=compact,
                    signed_lateral_m=signed_lateral,
                    dist_3d_m=_to_float(row.get("dist_3d_m")),
                    passed_accuracy=_to_bool(row.get("passed_accuracy")),
                )
            )
    return cases


def parse_compact_csv(path: Path) -> List[CompactSample]:
    rows: List[CompactSample] = []
    first_abs_s: Optional[float] = None
    previous_abs_s: Optional[float] = None
    day_offset_s = 0.0
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            dist = _to_float(row.get("dist"))
            yaw_err = _to_float(row.get("yaw_err"))
            cmd_r = _to_float(row.get("cmd_r"))
            act_r = _to_float(row.get("act_r"))
            ts_abs = _parse_ts_seconds(row.get("ts"))
            if dist is None or yaw_err is None or cmd_r is None or act_r is None or ts_abs is None:
                continue
            if previous_abs_s is not None and ts_abs + day_offset_s < previous_abs_s - 3600.0:
                day_offset_s += 24.0 * 3600.0
            abs_s = ts_abs + day_offset_s
            if first_abs_s is None:
                first_abs_s = abs_s
            previous_abs_s = abs_s
            rows.append(
                CompactSample(
                    ts=(row.get("ts") or "").strip(),
                    t_s=abs_s - first_abs_s,
                    dist=dist,
                    yaw_err=yaw_err,
                    cmd_r=cmd_r,
                    act_r=act_r,
                    cmd_p=_to_float(row.get("cmd_p")),
                    act_p=_to_float(row.get("act_p")),
                )
            )
    return rows


def terminal_window(samples: Sequence[CompactSample], terminal_dist_m: float) -> List[CompactSample]:
    selected = [sample for sample in samples if sample.dist <= terminal_dist_m]
    if selected:
        return selected
    tail_count = max(1, len(samples) // 5)
    return list(samples[-tail_count:])


def sign_flips(values: Iterable[float], deadband: float = 0.2) -> int:
    previous = 0
    flips = 0
    for value in values:
        current = _sign(value, deadband)
        if current == 0:
            continue
        if previous and current != previous:
            flips += 1
        previous = current
    return flips


def integrate_by_time(samples: Sequence[CompactSample], selector: Callable[[CompactSample], float]) -> float:
    total = 0.0
    for prev, cur in zip(samples, samples[1:]):
        dt = max(0.0, cur.t_s - prev.t_s)
        total += 0.5 * (selector(prev) + selector(cur)) * dt
    return total


def compute_terminal_features(samples: Sequence[CompactSample], terminal_dist_m: float = 300.0) -> TerminalFeatures:
    window = terminal_window(samples, terminal_dist_m)
    if not window:
        return TerminalFeatures(0, 0.0, 0.0, 0, 0.0, 0.0, 0.0)
    return TerminalFeatures(
        count=len(window),
        yaw_mean_deg=statistics.fmean(sample.yaw_err for sample in window),
        yaw_integral_deg_s=integrate_by_time(window, lambda sample: sample.yaw_err),
        yaw_sign_flips=sign_flips(sample.yaw_err for sample in window),
        cmd_roll_mean_deg=statistics.fmean(sample.cmd_r for sample in window),
        act_roll_mean_deg=statistics.fmean(sample.act_r for sample in window),
        cmd_roll_saturation_frac=sum(1 for sample in window if abs(sample.cmd_r) >= 44.0) / len(window),
    )


def candidate_baseline_cmd(samples: Sequence[CompactSample]) -> List[float]:
    return [sample.cmd_r for sample in samples]


def candidate_visual_p(samples: Sequence[CompactSample], gain: float, cap_deg: float) -> List[float]:
    return [_clip(gain * sample.yaw_err, -cap_deg, cap_deg) for sample in samples]


def candidate_terminal_leaky_i(
    samples: Sequence[CompactSample],
    gain: float = 1.1,
    leak_s: float = 2.5,
    cap_deg: float = 18.0,
    active_dist_m: float = 300.0,
    min_stable_s: float = 0.6,
    yaw_deadband_deg: float = 0.4,
    max_integrating_yaw_deg: float = 8.0,
) -> List[float]:
    out: List[float] = []
    bias = 0.0
    stable_s = 0.0
    previous_t: Optional[float] = None
    previous_sign = 0
    for sample in samples:
        dt = 0.0 if previous_t is None else max(0.0, sample.t_s - previous_t)
        previous_t = sample.t_s
        if dt:
            bias *= math.exp(-dt / max(0.1, leak_s))
        current_sign = _sign(sample.yaw_err, yaw_deadband_deg)
        if current_sign and current_sign == previous_sign:
            stable_s += dt
        elif current_sign:
            stable_s = 0.0
            previous_sign = current_sign
        else:
            stable_s = 0.0
            previous_sign = 0
        if (
            sample.dist <= active_dist_m
            and stable_s >= min_stable_s
            and yaw_deadband_deg <= abs(sample.yaw_err) <= max_integrating_yaw_deg
        ):
            bias += sample.yaw_err * dt
        out.append(_clip(gain * bias, -cap_deg, cap_deg))
    return out


def default_candidates(samples: Sequence[CompactSample]) -> Dict[str, List[float]]:
    return {
        "visual_p_pos": candidate_visual_p(samples, gain=1.0, cap_deg=20.0),
        "visual_p_neg": candidate_visual_p(samples, gain=-1.0, cap_deg=20.0),
        "terminal_leaky_i": candidate_terminal_leaky_i(samples),
        "terminal_leaky_i_neg": [-value for value in candidate_terminal_leaky_i(samples)],
    }


def score_values(
    name: str,
    samples: Sequence[CompactSample],
    values: Sequence[float],
    need_roll_sign: int,
    terminal_dist_m: float = 300.0,
) -> CandidateScore:
    selected_pairs = [(sample, value) for sample, value in zip(samples, values) if sample.dist <= terminal_dist_m]
    if not selected_pairs:
        selected_pairs = list(zip(samples[-max(1, len(samples) // 5) :], values[-max(1, len(values) // 5) :]))
    selected_values = [value for _, value in selected_pairs]
    if not selected_values:
        return CandidateScore(name, 0.0, 0.0, 0.0, None, None)
    agree: Optional[float]
    harmful: Optional[float]
    if need_roll_sign:
        signs = [_sign(value, 0.5) for value in selected_values]
        agree = sum(1 for sign in signs if sign == need_roll_sign) / len(signs)
        harmful = sum(1 for sign in signs if sign == -need_roll_sign) / len(signs)
    else:
        agree = None
        harmful = None
    return CandidateScore(
        name=name,
        mean_deg=statistics.fmean(selected_values),
        final_deg=selected_values[-1],
        mean_abs_deg=statistics.fmean(abs(value) for value in selected_values),
        sign_agree_frac=agree,
        harmful_frac=harmful,
    )


def gate_samples(
    samples: Sequence[CompactSample],
    candidates: Dict[str, Sequence[float]],
    gates_m: Sequence[float],
) -> List[Tuple[float, CompactSample, Dict[str, float]]]:
    gated: List[Tuple[float, CompactSample, Dict[str, float]]] = []
    indexed = list(enumerate(samples))
    for gate in gates_m:
        idx, sample = min(indexed, key=lambda pair: abs(pair[1].dist - gate))
        gated.append((gate, sample, {name: values[idx] for name, values in candidates.items()}))
    return gated


def analyze_case(
    case: FinalCase,
    samples: Sequence[CompactSample],
    gates_m: Sequence[float],
    terminal_dist_m: float = 300.0,
    lateral_deadband_m: float = 0.1,
) -> CaseDiagnostics:
    need_roll_sign = -_sign(case.signed_lateral_m, lateral_deadband_m)
    candidates = default_candidates(samples)
    baseline_values = candidate_baseline_cmd(samples)
    return CaseDiagnostics(
        case=case,
        features=compute_terminal_features(samples, terminal_dist_m),
        need_roll_sign=need_roll_sign,
        baseline=score_values("baseline_cmd", samples, baseline_values, need_roll_sign, terminal_dist_m),
        candidates=[
            score_values(name, samples, values, need_roll_sign, terminal_dist_m)
            for name, values in candidates.items()
        ],
        gates=gate_samples(samples, {"baseline_cmd": baseline_values, **candidates}, gates_m),
    )


def _fmt_float(value: Optional[float], digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def _fmt_frac(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    return f"{100.0 * value:.0f}%"


def _format_sign(sign_value: int) -> str:
    if sign_value > 0:
        return "+"
    if sign_value < 0:
        return "-"
    return "0"


def print_diagnostics(diags: Sequence[CaseDiagnostics], show_gates: bool = True) -> None:
    print("Case summary")
    header = (
        "name",
        "wind",
        "signed_lat_m",
        "need_roll",
        "yaw_mean",
        "yaw_int",
        "yaw_flips",
        "cmd_mean",
        "cmd_sat",
        "baseline_agree",
    )
    print(",".join(header))
    for diag in diags:
        f = diag.features
        row = (
            diag.case.name,
            f"{diag.case.wind_speed:g}@{diag.case.wind_dir:g}",
            _fmt_float(diag.case.signed_lateral_m, 3),
            _format_sign(diag.need_roll_sign),
            _fmt_float(f.yaw_mean_deg, 2),
            _fmt_float(f.yaw_integral_deg_s, 2),
            str(f.yaw_sign_flips),
            _fmt_float(f.cmd_roll_mean_deg, 2),
            _fmt_frac(f.cmd_roll_saturation_frac),
            _fmt_frac(diag.baseline.sign_agree_frac),
        )
        print(",".join(row))

    print()
    print("Candidate summary")
    print("candidate,failed_agree,failed_harm,pass_mean_abs_deg,notes")
    candidate_names = sorted({score.name for diag in diags for score in diag.candidates})
    for name in candidate_names:
        failed_scores: List[CandidateScore] = []
        pass_scores: List[CandidateScore] = []
        for diag in diags:
            score = next(score for score in diag.candidates if score.name == name)
            if diag.need_roll_sign:
                failed_scores.append(score)
            else:
                pass_scores.append(score)
        agree_values = [score.sign_agree_frac for score in failed_scores if score.sign_agree_frac is not None]
        harm_values = [score.harmful_frac for score in failed_scores if score.harmful_frac is not None]
        quiet_values = [score.mean_abs_deg for score in pass_scores]
        notes = []
        if quiet_values and statistics.fmean(quiet_values) > 8.0:
            notes.append("pass-case risk")
        if harm_values and statistics.fmean(harm_values) > 0.35:
            notes.append("harmful sign")
        if agree_values and statistics.fmean(agree_values) > 0.65 and (not quiet_values or statistics.fmean(quiet_values) <= 8.0):
            notes.append("worth SITL discriminator")
        row = (
            name,
            _fmt_frac(statistics.fmean(agree_values) if agree_values else None),
            _fmt_frac(statistics.fmean(harm_values) if harm_values else None),
            _fmt_float(statistics.fmean(quiet_values) if quiet_values else None, 2),
            "; ".join(notes) or "-",
        )
        print(",".join(row))

    if not show_gates:
        return
    print()
    print("Gate samples")
    print("name,gate_m,dist_m,yaw_err,cmd_r,act_r,visual_p_pos,visual_p_neg,terminal_leaky_i,terminal_leaky_i_neg")
    for diag in diags:
        for gate, sample, candidates in diag.gates:
            row = (
                diag.case.name,
                f"{gate:g}",
                _fmt_float(sample.dist, 1),
                _fmt_float(sample.yaw_err, 2),
                _fmt_float(sample.cmd_r, 2),
                _fmt_float(sample.act_r, 2),
                _fmt_float(candidates["visual_p_pos"], 2),
                _fmt_float(candidates["visual_p_neg"], 2),
                _fmt_float(candidates["terminal_leaky_i"], 2),
                _fmt_float(candidates["terminal_leaky_i_neg"], 2),
            )
            print(",".join(row))


def _parse_gates(value: str) -> List[float]:
    gates: List[float] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        gates.append(float(part))
    return gates


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("final_csv", type=Path, help="SITL evaluation final.csv")
    parser.add_argument(
        "--gates",
        default="500,300,200,150,100,80,60,50,40,30,20",
        help="Comma-separated distance gates for trace samples.",
    )
    parser.add_argument("--del-dist", type=float, default=300.0, help="Delivery window distance in meters.")
    parser.add_argument("--no-gates", action="store_true", help="Only print case and candidate summaries.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    cases = parse_final_csv(args.final_csv)
    if not cases:
        raise SystemExit(f"no usable cases in {args.final_csv}")
    gates = _parse_gates(args.gates)
    diagnostics: List[CaseDiagnostics] = []
    missing_logs: List[Path] = []
    for case in cases:
        if not case.compact.exists():
            missing_logs.append(case.compact)
            continue
        samples = parse_compact_csv(case.compact)
        if not samples:
            continue
        diagnostics.append(analyze_case(case, samples, gates, terminal_dist_m=args.del_dist))
    if missing_logs:
        for path in missing_logs:
            print(f"warning: missing compact log: {path}")
    if not diagnostics:
        raise SystemExit("no cases had readable compact samples")
    print_diagnostics(diagnostics, show_gates=not args.no_gates)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
