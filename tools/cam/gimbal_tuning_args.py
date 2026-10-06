"""CLI contract for the detector-backed SIYI tuning tool."""

from __future__ import annotations

import argparse
from collections.abc import Sequence


MIN_ZOOM = 1.0
MAX_ZOOM = 30.0
DEFAULT_MAX_RATE = 100.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detector-backed SIYI gimbal controller for zoom tuning",
    )
    parser.add_argument("--profile", default="siyi_zr10")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--camera", default=None)
    parser.add_argument("--model", default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--zoom", type=float, default=1.0)
    parser.add_argument("--anchor", type=int, choices=range(1, 10), default=5)
    parser.add_argument("--max-rate", type=float, default=DEFAULT_MAX_RATE)
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


__all__ = [
    "DEFAULT_MAX_RATE",
    "MAX_ZOOM",
    "MIN_ZOOM",
    "build_parser",
    "parse_args",
]
