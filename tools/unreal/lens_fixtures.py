r"""Ray fixtures for the Unreal lens table (HITL stage 1 step 5, hitl_final_approach.md section 9).

The Unreal camera actor renders each frame through the pinhole of one row of navpy's siyi_zr10
``camera.reading_intrinsics`` (no distortion: the encoder adds it, step 8). This writes the fixture its automation
test (NavPy.Lens) checks against: the profile section verbatim (its identity: the file's hash also changes with
unrelated profiles), and for every row and for
synthetic rows (skew of both signs, fx != fy, an off-centre principal point) rays in the OpenCV camera frame (x right,
y down, z forward) with the pixels K gives them. The pixels come from the explicit homogeneous K, skew included;
OpenCV's projectPoints ignores skew, so it only cross-checks the zero-skew rows.

  .venv\Scripts\python.exe -m tools.unreal.lens_fixtures write --out <aas-sim-unreal>\Plugins\NavPyUnrealBridge\Resources\Tests\lens_fixtures.json
  .venv\Scripts\python.exe -m tools.unreal.lens_fixtures check --fixture <same path>

``check`` fails when the fixture is not what ``write`` makes from the current profile.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from navpy.core.perception.camera.reading_intrinsics import ReadingIntrinsicsTable

SCHEMA = "navpy.unreal.lens_fixtures.v1"
PROFILE = Path(__file__).resolve().parents[2] / "src/navpy/core/perception/profile/vision_profiles.json"
PROFILE_NAME = "siyi_zr10"
DEVICE_NAME = "siyi_zr10"
# Image points as fractions of the frame (corners, edge middles, centre), each at three depths in metres: the test
# covers the whole frame and the perspective division.
GRID = (0.02, 0.5, 0.98)
DEPTHS_M = (1.0, 37.0, 950.0)
# Synthetic pinholes for what the measured rows do not exercise (all their skews are 0).
SYNTHETIC = (
    {"name": "skew_positive", "width": 1920, "height": 1080, "fx": 2100.0, "fy": 2080.0, "cx": 1001.3, "cy": 530.7,
     "skew": 35.0},
    {"name": "skew_negative", "width": 1920, "height": 1080, "fx": 3300.0, "fy": 3350.0, "cx": 930.2, "cy": 571.9,
     "skew": -42.5},
    {"name": "off_centre_small", "width": 1280, "height": 720, "fx": 1407.42, "fy": 1401.91, "cx": 670.95,
     "cy": 369.78, "skew": 0.0},
)


def load_section(profile_path: Path, profile: str = PROFILE_NAME, device: str = DEVICE_NAME) -> dict:
    """The device's camera.reading_intrinsics, parsed by navpy's own rules first (a bad table is refused)."""
    root = json.loads(profile_path.read_text(encoding="utf-8"))
    devices = [d for d in root["profiles"][profile]["devices"] if d.get("name") == device]
    if len(devices) != 1:
        raise ValueError(f"{profile}: expected one device {device!r}, found {len(devices)}")
    section = devices[0]["camera"]["reading_intrinsics"]
    ReadingIntrinsicsTable.from_profile(section)
    return section


def pinhole_pixels(fx: float, fy: float, cx: float, cy: float, skew: float, rays: np.ndarray) -> np.ndarray:
    """u = fx x/z + skew y/z + cx, v = fy y/z + cy (OpenCV camera frame, pixel centres at integers)."""
    k = np.array([[fx, skew, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    projected = rays @ k.T
    return projected[:, :2] / projected[:, 2:3]


def grid_rays(fx: float, fy: float, cx: float, cy: float, skew: float, width: int, height: int) -> np.ndarray:
    """Rays through the GRID image points at every DEPTHS_M (inverse K)."""
    k_inv = np.linalg.inv(np.array([[fx, skew, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]))
    rays = []
    for gy in GRID:
        for gx in GRID:
            direction = k_inv @ np.array([gx * width, gy * height, 1.0])
            rays.extend(direction * depth for depth in DEPTHS_M)
    return np.array(rays)


def _case(fx: float, fy: float, cx: float, cy: float, skew: float, width: int, height: int) -> dict:
    rays = grid_rays(fx, fy, cx, cy, skew, width, height)
    pixels = pinhole_pixels(fx, fy, cx, cy, skew, rays)
    if skew == 0.0:
        cross, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3),
                                     np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]), np.zeros(5))
        if not np.allclose(cross.reshape(-1, 2), pixels, atol=1e-6):
            raise AssertionError("explicit K and projectPoints disagree on a zero-skew row")
    return {"rays_cv": rays.round(9).tolist(), "pixels": pixels.round(9).tolist()}


def build(profile_path: Path = PROFILE) -> dict:
    """The fixture for the profile at ``profile_path``."""
    section = load_section(profile_path)
    rows = []
    for stream in section["streams"]:
        width, height = stream["image_width"], stream["image_height"]
        for row in stream["rows"]:
            case = _case(row["fx"], row["fy"], row["cx"], row["cy"], row["skew"], width, height)
            rows.append({"stream": stream["mode"], "reading": row["reading"], "set_by": row["set_by"], **case})
    synthetic = [{**spec, **_case(spec["fx"], spec["fy"], spec["cx"], spec["cy"], spec["skew"], spec["width"],
                                  spec["height"])} for spec in SYNTHETIC]
    return {"schema": SCHEMA, "profile_file": "src/navpy/core/perception/profile/vision_profiles.json",
            "profile": PROFILE_NAME, "device": DEVICE_NAME, "reading_intrinsics": section, "rows": rows,
            "synthetic": synthetic}


def fixture_text(fixture: Mapping[str, Any]) -> str:
    return json.dumps(fixture, indent=1, sort_keys=True) + "\n"


def check(fixture_path: Path, profile_path: Path = PROFILE) -> list[str]:
    """Problems with the fixture against the current profile; empty when it is exactly what ``build`` makes."""
    try:
        have = json.loads(fixture_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return [f"{fixture_path}: {exc}"]
    want = build(profile_path)
    problems = [f"{key} differs" for key in sorted(set(want) | set(have)) if have.get(key) != want.get(key)]
    return problems


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    write = sub.add_parser("write", help="write the fixture")
    write.add_argument("--out", type=Path, required=True)
    chk = sub.add_parser("check", help="compare a fixture with the current profile")
    chk.add_argument("--fixture", type=Path, required=True)
    for p in (write, chk):
        p.add_argument("--profile", type=Path, default=PROFILE)
    args = parser.parse_args(argv)
    if args.command == "write":
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w", encoding="utf-8", newline="\n") as out:
            out.write(fixture_text(build(args.profile)))
        print(f"wrote {args.out}")
        return 0
    problems = check(args.fixture, args.profile)
    for problem in problems:
        print(problem, file=sys.stderr)
    print("lens fixture: " + ("OK" if not problems else f"{len(problems)} problem(s)"))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
