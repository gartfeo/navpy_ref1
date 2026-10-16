r"""The dock's ChArUco board texture for Unreal, from navpy's own detector board definition.

The image is drawn by OpenCV from the same CharucoBoardSpec the default detector builds
(navpy.plugins.detection.charuco), so dictionary, squares and marker/square ratio match by
construction. Detection is scale-free (a homography), so the physical size in Unreal is free; the
sidecar JSON records it, and a profile that needs metric board geometry sets
detector.charuco square_mm/marker_mm to the sidecar's values.

  .venv\Scripts\python.exe -m tools.unreal.charuco_board_texture --out C:\repos\unreal-projects\AasSimProject\Scripts\charuco_board.png

The Unreal project's Scripts\create_charuco_board_assets.py imports the PNG as a texture and
builds the board material; the plugin's UNavPyDockBoardSubsystem places it on the dock.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec
from navpy.plugins.detection.charuco.charuco_detector import build_charuco_board

# 256 px per square keeps the texture's sides at whole multiples of 128 px.
DEFAULT_SQUARE_PX = 256
# Half a square of white around the board: the outer markers keep a light quiet zone on any deck colour.
MARGIN_SQUARES = 0.5
# 40 cm squares: the board with its margin (2.4 x 3.2 m) fits the 4 m deck clear of the crate corner.
DEFAULT_SQUARE_CM = 40.0


def board_image(spec: CharucoBoardSpec, square_px: int = DEFAULT_SQUARE_PX) -> np.ndarray:
    """Grey image of the board with its white margin; rows run along the board's squares_y."""
    margin_px = int(round(MARGIN_SQUARES * square_px))
    size = (spec.squares_x * square_px + 2 * margin_px, spec.squares_y * square_px + 2 * margin_px)
    return build_charuco_board(spec).generateImage(size, marginSize=margin_px, borderBits=1)


def sidecar(spec: CharucoBoardSpec, square_px: int, square_cm: float) -> dict:
    margin_cm = MARGIN_SQUARES * square_cm
    return {
        "dictionary": spec.dictionary, "squares_x": spec.squares_x, "squares_y": spec.squares_y,
        "square_px": square_px, "margin_squares": MARGIN_SQUARES,
        "unreal_square_cm": square_cm,
        "unreal_board_width_cm": spec.squares_x * square_cm + 2 * margin_cm,
        "unreal_board_height_cm": spec.squares_y * square_cm + 2 * margin_cm,
        "profile_charuco_override": {"square_mm": square_cm * 10.0,
                                     "marker_mm": square_cm * 10.0 * spec.marker_mm / spec.square_mm},
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, required=True, help="PNG to write (a .json sidecar is written next to it)")
    parser.add_argument("--square-px", type=int, default=DEFAULT_SQUARE_PX)
    parser.add_argument("--square-cm", type=float, default=DEFAULT_SQUARE_CM, help="square size on the dock in Unreal")
    args = parser.parse_args(argv)
    spec = CharucoBoardSpec()
    image = board_image(spec, args.square_px)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    # Three channels: Unreal imports an 8-bit grey PNG as a G8 texture, which a default (colour)
    # texture sampler in the board material refuses.
    if not cv2.imwrite(str(args.out), cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)):
        raise SystemExit(f"cannot write {args.out}")
    info = sidecar(spec, args.square_px, args.square_cm)
    args.out.with_suffix(".json").write_text(json.dumps(info, indent=1), encoding="utf-8")
    print(f"{args.out}: {image.shape[1]}x{image.shape[0]} px, {info['unreal_board_width_cm']:g} x "
          f"{info['unreal_board_height_cm']:g} cm in Unreal")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
