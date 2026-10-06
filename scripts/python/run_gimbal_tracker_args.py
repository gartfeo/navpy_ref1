"""Command-line contract and stable demo presets for the SIYI runner."""

from __future__ import annotations

import argparse
from collections.abc import Sequence


SIYI_PROFILE_NAME = "siyi_zr10"
ANCHOR_PADDING_PX = 50

ANCHOR_NAMES = {
    1: "BL", 2: "BC", 3: "BR",
    4: "ML", 5: "C", 6: "MR",
    7: "TL", 8: "TC", 9: "TR",
}
DETECTOR_PRESETS = {
    "faces": ("yolov8n-face-lindevs.pt", None),
    "people": ("yolov8s.pt", [0]),
    "vehicles": ("yolov8s.pt", [2]),
    "all": ("yolov8s.pt", None),
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SIYI gimbal tracking demo")
    parser.add_argument("--ip", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--stream", default=None)
    parser.add_argument(
        "--preset",
        choices=list(DETECTOR_PRESETS),
        default="vehicles",
    )
    parser.add_argument("--model", default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--detect-hz", type=float, default=None)
    parser.add_argument("--track-hz", type=float, default=None)
    parser.add_argument("--anchor", type=int, choices=range(1, 10), default=5)
    parser.add_argument("--pitch", type=float, default=15.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--zoom", type=float, default=3.0)
    parser.add_argument("--max-rate", type=float, default=None)
    parser.add_argument(
        "--deep-search",
        dest="deep_search",
        action="store_true",
        default=None,
    )
    parser.add_argument(
        "--no-deep-search",
        dest="deep_search",
        action="store_false",
    )
    parser.add_argument("--deep-search-hz", type=float, default=None)
    parser.add_argument("--deep-search-imgsz", type=int, default=None)
    parser.add_argument("--deep-search-conf", type=float, default=None)
    parser.add_argument(
        "--tracker-backend",
        choices=["custom", "botsort", "strongsort"],
        default=None,
    )
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


__all__ = [
    "ANCHOR_NAMES",
    "ANCHOR_PADDING_PX",
    "DETECTOR_PRESETS",
    "SIYI_PROFILE_NAME",
    "build_parser",
    "parse_args",
]
