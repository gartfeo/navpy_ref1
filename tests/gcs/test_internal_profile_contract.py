"""Renamed profile fields must not silently discard persisted settings."""

import json
from unittest.mock import patch

import pytest

from navpy.modules.vision.vision_profile_loader import load_profiles


@pytest.mark.parametrize("current_present", [False, True])
@pytest.mark.parametrize("scope", ["catalog", "detector"])
def test_retired_profile_fields_fail_without_rewriting(tmp_path, scope, current_present):
    presets = {"small": {"altitude_m": 150, "min_pixel_size": 20}}
    dimensions = {"0": {"width_m": 3.5, "height_m": 2.5}}
    data = {"profiles": {"fixture": {"detector": {}}}}
    if scope == "catalog":
        data["target_classes"] = dimensions
        if current_present:
            data["detector_class_dimensions"] = dimensions
    else:
        data["profiles"]["fixture"]["detector"]["target_presets"] = presets
        if current_present:
            data["profiles"]["fixture"]["detector"]["dock_presets"] = presets
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps(data))
    before = path.read_bytes()
    with patch("navpy.modules.vision.vision_profile_loader._profiles_path", return_value=path):
        with pytest.raises(ValueError, match="retired profile field"):
            load_profiles()
    assert path.read_bytes() == before
