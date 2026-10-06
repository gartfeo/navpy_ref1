"""Render one- and two-dock confirmation examples using the simulator renderer.

Usage: python scripts/python/demo_confirmation_image.py --assets .sim
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import cv2

from navpy.modules.vision.sim.sim_frame_generator import SimFrameGenerator
from navpy.utils.image_utils import create_detection_thumbnail, save_confirmation_image, PoiInfo


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=ROOT / ".sim")
    parser.add_argument("--output", type=Path, default=ROOT / ".runtime" / "dock-demo")
    args = parser.parse_args(argv)
    background = cv2.imread(str(args.assets / "background.png"))
    if background is None:
        parser.error(f"Cannot read background.png from {args.assets}")
    height, width = background.shape[:2]
    renderer = SimFrameGenerator(str(args.assets), (width, height))
    args.output.mkdir(parents=True, exist_ok=True)
    scenarios = [
        ("1dock", [(0.50, 0.52, 0.55)], [5]),
        ("2docks", [(0.30, 0.52, 0.50), (0.63, 0.52, 0.45)], [1, 2]),
    ]
    for name, positions, ids in scenarios:
        frame, boxes = renderer.generate_frame(positions)
        frame_path = args.output / f"sim_frame_{name}.jpg"
        if not cv2.imwrite(str(frame_path), frame):
            raise OSError(f"Cannot write {frame_path}")
        pois = [PoiInfo(poi_id=i, bbox=box, class_name="Dock")
                   for i, box in zip(ids, boxes)]
        thumbnail = create_detection_thumbnail(frame, pois)
        save_confirmation_image(thumbnail, str(args.output / f"sim_detect_{name}.jpg"))
        print(f"Rendered {name}: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
