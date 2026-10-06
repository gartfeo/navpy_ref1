"""Command-line contract and certificate-mode validation."""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Sequence
from pathlib import Path

from scripts import eval_certificate as cert

from eval_navigation_matrix import (
    parse_float_list,
    parse_navigation_speedups,
    parse_int_list,
    parse_target_alts,
)


DEFAULT_PYTHON = Path(sys.executable).resolve()
DEFAULT_MAX_ATTEMPTS = 2
DEFAULT_MAX_DISTANCE_M = 0.5


def build_parser() -> argparse.ArgumentParser:
    """Build the evaluator CLI without performing filesystem checks."""
    parser = argparse.ArgumentParser(
        description=(
            "Run one-aircraft final approach SITL cases with target identity, "
            "mission-coordinate, altitude, and independent closest-approach checks."
        )
    )
    parser.add_argument("--python", type=Path, default=DEFAULT_PYTHON)
    parser.add_argument(
        "--target-alts",
        type=parse_target_alts,
        default=None,
        metavar="ALTS",
    )
    parser.add_argument("--target-wp", type=int, default=4)
    parser.add_argument(
        "--max-distance",
        "--max-dist",
        dest="max_distance",
        type=float,
        default=None,
    )
    parser.add_argument("--coordinate-tolerance-m", type=float, default=0.5)
    parser.add_argument("--altitude-tolerance-m", type=float, default=0.5)
    parser.add_argument(
        "--speedups",
        type=parse_float_list,
        default=parse_float_list("10"),
    )
    parser.add_argument(
        "--navigation-speedups",
        type=parse_navigation_speedups,
        default=None,
        metavar="match|VALUES",
    )
    parser.add_argument(
        "--winds",
        type=parse_float_list,
        default=parse_float_list("0"),
    )
    parser.add_argument(
        "--directions",
        type=parse_int_list,
        default=parse_int_list("0,90,180,270"),
    )
    parser.add_argument("--max-attempts", type=int, default=None)
    parser.add_argument("--repetitions", type=int, default=None, metavar="N")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--heartbeat-timeout", type=float, default=120.0)
    parser.add_argument("--guid-options", type=int, default=None)
    parser.add_argument(
        "--stall-prevention",
        type=int,
        choices=(0, 1),
        default=None,
    )
    parser.add_argument("--pitch-controller", default="vision-nav-pn")
    parser.add_argument("--del-throttle", type=float, default=-1)
    parser.add_argument("--del-angle", type=float, default=0)
    parser.add_argument("--vision-profile", default="ideal_360")
    parser.add_argument("--detector-type", default="sim")
    parser.add_argument("--network-type", default="none")
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse and validate ordinary and repeatability-certificate modes."""
    parser = build_parser()
    args = parser.parse_args(argv)
    args.python = args.python.resolve()
    if not args.python.is_file():
        parser.error(f"Python runtime does not exist: {args.python}")
    if args.target_wp < 1:
        parser.error("--target-wp must be >= 1")
    if args.target_alts is None:
        args.target_alts = [0]
    explicit_max_distance = args.max_distance is not None
    if not explicit_max_distance:
        args.max_distance = DEFAULT_MAX_DISTANCE_M
    _validate_positive_values(parser, args)
    if args.max_attempts is not None and args.max_attempts < 1:
        parser.error("--max-attempts must be >= 1")
    if any(speedup <= 0 for speedup in args.speedups):
        parser.error("--speedups values must be > 0")
    if args.repetitions is None:
        args.max_attempts = (
            DEFAULT_MAX_ATTEMPTS
            if args.max_attempts is None
            else args.max_attempts
        )
        return args
    _validate_certificate_mode(
        parser,
        args,
        explicit_max_distance=explicit_max_distance,
    )
    return args


def _validate_positive_values(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
) -> None:
    for option in (
        "max_distance",
        "coordinate_tolerance_m",
        "altitude_tolerance_m",
        "timeout",
        "heartbeat_timeout",
    ):
        value = getattr(args, option)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{option.replace('_', '-')} must be finite and > 0")


def _validate_certificate_mode(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    *,
    explicit_max_distance: bool,
) -> None:
    if args.repetitions < 1:
        parser.error("--repetitions must be >= 1")
    if args.max_attempts not in (None, 1):
        parser.error(
            "--repetitions is certificate mode and allows exactly one attempt "
            "per run; pass --max-attempts 1 or drop it"
        )
    args.max_attempts = 1
    if args.speedups != [cert.CERTIFICATE_SPEEDUP]:
        parser.error(
            "--repetitions certifies one canonical speed: use "
            f"--speedups {cert.CERTIFICATE_SPEEDUP}"
        )
    if any(abs(wind) > 0 for wind in args.winds):
        parser.error("--repetitions certifies zero wind: use --winds 0")
    if explicit_max_distance:
        parser.error(
            "--repetitions certifies the pinned "
            f"{DEFAULT_MAX_DISTANCE_M:g} m criterion; drop --max-distance"
        )
