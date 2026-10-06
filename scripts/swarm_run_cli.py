"""Validated CLI inputs for the isolated swarm supervisor."""
from __future__ import annotations

import argparse
import math
from gcs.backend import instance_ports as ip

def _positive_speedup(raw: str) -> float:
    try:
        speedup = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError("speedup must be numeric") from error
    if not math.isfinite(speedup) or speedup <= 0.0:
        raise argparse.ArgumentTypeError("speedup must be finite and > 0")
    return speedup


def _home_coords(raw: str) -> str:
    """Validate LAT,LON,ALT,HEADING and re-emit it as four plain numbers.

    This value is interpolated into a Bash command line (see
    ``swarm_run_wsl.launch_command``), so it must not be able to carry shell
    metacharacters.  Rebuilding the string from parsed floats rather than
    passing the caller's text through means only digits, signs, dots and
    commas can ever reach the shell.
    """
    parts = raw.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "home must be LAT,LON,ALT,HEADING (four comma-separated numbers)"
        )
    try:
        lat, lon, alt, heading = (float(part) for part in parts)
    except ValueError as error:
        raise argparse.ArgumentTypeError("home fields must be numeric") from error
    if not all(math.isfinite(value) for value in (lat, lon, alt, heading)):
        raise argparse.ArgumentTypeError("home fields must be finite")
    if not -90.0 <= lat <= 90.0:
        raise argparse.ArgumentTypeError("home latitude must be -90..90")
    if not -180.0 <= lon <= 180.0:
        raise argparse.ArgumentTypeError("home longitude must be -180..180")
    return f"{lat!r},{lon!r},{alt!r},{heading!r}"


def build_parser(default_speedup: float) -> argparse.ArgumentParser:
    """Build the stable swarm supervisor command-line contract."""
    parser = argparse.ArgumentParser(
        description="Launch an isolated SITL swarm for a chat."
    )
    parser.add_argument("--chat", type=int, default=None)
    parser.add_argument("--single-boot", action="store_true",
                        help="experimental launch: no parameter healing or automatic relaunch")
    parser.add_argument(
        "--eval",
        action="store_true",
        help="reserve an eval-band slot instead of an interactive slot",
    )
    parser.add_argument(
        "-n",
        "--instances",
        type=int,
        default=ip.VEHICLES_PER_CHAT,
    )
    parser.add_argument("-d", "--dist", type=int, default=50)
    parser.add_argument(
        "--home",
        type=_home_coords,
        default=None,
        metavar="LAT,LON,ALT,HEADING",
        help=(
            "pin the swarm start point instead of inheriting the launcher "
            "default (which has been observed to shift with the slot index). "
            "Heading is degrees true; 0 is north."
        ),
    )
    parser.add_argument(
        "--defaults",
        default=None,
        metavar="WSL_PATH",
        help=(
            "pre-boot parameter file for the SITL instances, as a WSL path. "
            "Defaults to run_swarm.sh's shared models/plane.parm. Use this "
            "for values consumed at allocation or calibration time "
            "(EK3_HGT_DELAY, SIM_BARO_*, GPS1_DELAY_MS), which a post-boot "
            "parameter push sets too late to matter."
        ),
    )
    parser.add_argument("--speedup", type=_positive_speedup, default=default_speedup)
    parser.add_argument("--launch-token", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--sitl-root", default=None, metavar="WSL_PATH",
                        help="isolated firmware/state root for simulator experiments")
    parser.add_argument("--sitl-binary", default=None, metavar="WSL_PATH",
                        help="explicit simulator executable (requires --sitl-root)")
    return parser
