"""A profile for a camera flown later (hitl_final_approach.md section 4, "Other cameras").

The 18x is a placeholder, not calibrated, and says so in `calibration`, so results from it are labelled.
"""

from __future__ import annotations

import math

from navpy.core.perception.profile.vision_profile_loader import get_devices, load_profiles


def _camera(name: str) -> dict:
    profiles, _, _ = load_profiles()
    (device,) = get_devices(profiles[name])
    return device["camera"]


def _fov_deg(pixels: int, fx: float) -> float:
    return math.degrees(2 * math.atan(pixels / (2 * fx)))


def test_placeholder_18x_is_the_zr10_wide_end_scaled_to_18x():
    camera = _camera("placeholder_18x")
    zr10 = _camera("siyi_zr10")
    zooms = camera["intrinsics"]["zooms"]
    assert camera["calibration"] == "placeholder"
    assert camera["max_optical_zoom"] == 18 and sorted(map(int, zooms)) == list(range(1, 19))
    assert (camera["image_width"], camera["image_height"]) == (zr10["image_width"], zr10["image_height"])
    assert zooms["1"] == zr10["intrinsics"]["zooms"]["1"]
    assert math.isclose(zooms["18"]["fx"], 18 * zooms["1"]["fx"], rel_tol=1e-6)
