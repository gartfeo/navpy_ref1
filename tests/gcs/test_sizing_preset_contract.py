"""Sizing names cross the development contracts without silent fallbacks."""

import json
from unittest.mock import patch

import pytest
from pydantic import ValidationError

from gcs.backend.models import VehicleAssignment
from gcs.backend.routes.vision_profiles import ProfileUpdate
from navpy.modules.vision.vision_profile_loader import load_profiles
from navpy.modules.vision.vision_class_profile import DOCK_PRESET_NAME


def test_all_shipped_profiles_use_the_single_dock_preset():
    profiles, _, _ = load_profiles()
    for profile in profiles.values():
        assert set((profile.get("detector") or {}).get("dock_presets") or {}) <= {DOCK_PRESET_NAME}
    assert DOCK_PRESET_NAME == "dock"


def test_upload_dock_class_selection_is_rejected():
    with pytest.raises(ValidationError):
        VehicleAssignment(sys_id=1, zone_index=0, waypoints=[], altitude_m=150,
                          dock_classes=["dock"])


@pytest.mark.parametrize("retired", ["small", "medium", "large"])
def test_retired_profile_edit_preset_is_rejected(retired):
    with pytest.raises(ValidationError):
        ProfileUpdate(dock_presets={retired: {"min_pixel_size": 45}})


def test_dock_profile_edit_is_accepted():
    update = ProfileUpdate(dock_presets={"dock": {"min_pixel_size": 45}})
    assert update.dock_presets == {"dock": {"min_pixel_size": 45}}


@pytest.mark.parametrize("retired", ["small", "medium", "large"])
def test_profile_file_retired_preset_fails(tmp_path, retired):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({"profiles": {"fixture": {"detector": {
        "dock_presets": {retired: {"min_pixel_size": 45}}
    }}}}))
    with patch("navpy.modules.vision.vision_profile_loader._profiles_path", return_value=path):
        with pytest.raises(ValueError, match="unknown sizing presets"):
            load_profiles()


def test_unknown_profile_edit_is_rejected_before_route_execution():
    with pytest.raises(ValidationError):
        ProfileUpdate(dock_presets={"obsolete-preset": {"min_pixel_size": 45}})


def test_profile_file_unknown_preset_fails_without_rewriting(tmp_path):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({"profiles": {"fixture": {"detector": {
        "dock_presets": {"obsolete-preset": {"min_pixel_size": 45}}
    }}}}))
    before = path.read_bytes()
    with patch("navpy.modules.vision.vision_profile_loader._profiles_path", return_value=path):
        with pytest.raises(ValueError, match="unknown sizing presets"):
            load_profiles()
    assert path.read_bytes() == before


@pytest.mark.parametrize("detector", [None, {}, {"dock_presets": None}])
def test_profile_loader_preserves_optional_empty_detector_blocks(tmp_path, detector):
    path = tmp_path / "profiles.json"
    profile = {"detector": detector}
    path.write_text(json.dumps({"profiles": {"fixture": profile}}))
    with patch("navpy.modules.vision.vision_profile_loader._profiles_path", return_value=path):
        profiles, _, _ = load_profiles()
    assert profiles["fixture"] == profile


def test_malformed_detector_has_a_clear_error(tmp_path):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({"profiles": {"fixture": {"detector": []}}}))
    with patch("navpy.modules.vision.vision_profile_loader._profiles_path", return_value=path):
        with pytest.raises(ValueError, match="detector must be a mapping"):
            load_profiles()
