"""``gcs_launch.py --cpu-share-s``: a wall run's CPU share (``bounded_cpu_share.md``)."""

from __future__ import annotations

import argparse
import math

from gcs.backend.cpu_share_background import apply_background
from gcs.backend.cpu_share_holder import CPU_SHARE_ENV


def _positive_seconds(text: str) -> float:
    seconds = float(text)
    if not (math.isfinite(seconds) and seconds > 0):
        raise argparse.ArgumentTypeError("must be a positive number of seconds")
    return seconds


def add_cpu_share_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--cpu-share-s", type=_positive_seconds, default=None, metavar="SECONDS",
        help="the backend claims a CPU share for this long (wall evals, the hand "
             "demo); without it this stack runs as background",
    )


def scheduling_env(args: argparse.Namespace) -> dict[str, str]:
    """The backend's share request. Without one, this launcher, and so every
    child it starts from now on, runs as background."""
    if args.cpu_share_s is None:
        apply_background()
        return {}
    return {CPU_SHARE_ENV: repr(args.cpu_share_s)}


__all__ = ["add_cpu_share_argument", "scheduling_env"]
