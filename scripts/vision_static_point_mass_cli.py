"""Command-line interface for the static point-mass diagnostic."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from scripts.vision_static_point_mass_run import default_cases, run_case


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--winds", default="0,5,8")
    parser.add_argument(
        "--directions",
        default="0,30,60,90,120,150,180,210,240,270",
    )
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--command-interval", type=float, default=None)
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument("--bearing-noise-deg", type=float, default=None)
    parser.add_argument("--noise-seed", type=int, default=None)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cases = default_cases(
        _parse_csv_floats(args.winds),
        _parse_csv_floats(args.directions),
        dt_s=args.dt,
        command_interval_s=args.command_interval,
        max_t_s=args.max_time,
        bearing_noise_deg=args.bearing_noise_deg,
        noise_seed=args.noise_seed,
    )
    print(
        "name,wind_speed,wind_dir,lateral_m,longitudinal_m,vertical_m,"
        "slant_m,t_s,passed_target,timed_out"
    )
    for case in cases:
        miss = run_case(case)
        print(
            f"{case.name},{case.wind_speed_mps:g},{case.wind_dir_from_deg:g},"
            f"{miss.lateral_m:.6f},{miss.longitudinal_m:.6f},"
            f"{miss.vertical_m:.6f},{miss.slant_m:.6f},{miss.t_s:.3f},"
            f"{int(miss.passed_target)},{int(miss.timed_out)}"
        )
    return 0


def _parse_csv_floats(value: str) -> list[float]:
    return [float(part) for part in value.split(",") if part.strip()]


__all__ = ["build_parser", "main"]
