"""The demo runs the actual simulator renderer without external sprite files."""

from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np


def test_demo_writes_readable_one_and_two_dock_frames(tmp_path):
    assets = tmp_path / "assets"
    assets.mkdir()
    assert cv2.imwrite(str(assets / "background.png"), np.zeros((1080, 1920, 3), np.uint8))
    output = tmp_path / "output"
    root = Path(__file__).resolve().parents[2]
    result = subprocess.run(
        [sys.executable, str(root / "scripts/python/demo_confirmation_image.py"),
         "--assets", str(assets), "--output", str(output)],
        cwd=root, capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stderr
    for name in ("1dock", "2docks"):
        frame = cv2.imread(str(output / f"sim_frame_{name}.jpg"))
        thumbnail = cv2.imread(str(output / f"sim_detect_{name}.jpg"))
        assert frame.shape == (1080, 1920, 3)
        assert thumbnail.shape == (480, 640, 3)
        assert np.any(frame)
        assert np.any(thumbnail)
