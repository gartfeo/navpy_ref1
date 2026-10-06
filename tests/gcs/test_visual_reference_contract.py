"""Reference-height rename fails closed at direct and file entrypoints."""
import json
from unittest.mock import patch

import pytest

from navpy.modules.vision.vision_detector_profile import get_detector_settings
from navpy.modules.vision.vision_profile_loader import load_profiles
from navpy.modules.vision.real_detector_factory import RealDetectorFactory


@pytest.mark.parametrize("also_current", [False, True])
def test_retired_height_rejected_by_direct_settings(also_current):
    settings = {"poi_height": 3.25}
    if also_current:
        settings["reference_height_m"] = 4.5
    before = dict(settings)
    with pytest.raises(ValueError, match="retired.*poi_height"):
        get_detector_settings({"detector": settings})
    assert settings == before


@pytest.mark.parametrize("also_current", [False, True])
def test_retired_height_file_rejected_without_rewrite(tmp_path, also_current):
    settings = {"poi_height": 3.25}
    if also_current:
        settings["reference_height_m"] = 4.5
    path = tmp_path / "profile.json"
    path.write_text(json.dumps({"profiles": {"fixture": {"detector": settings}}}))
    before = path.read_bytes()
    with patch("navpy.modules.vision.vision_profile_loader._profiles_path", return_value=path):
        with pytest.raises(ValueError, match="retired.*poi_height"):
            load_profiles()
    assert path.read_bytes() == before


@pytest.mark.parametrize("also_current", [False, True])
def test_retired_height_rejected_before_factory_construction(also_current):
    settings = {"poi_height": 3.25}
    if also_current:
        settings["reference_height_m"] = 4.5
    # No model, vehicle, or mount is needed: reject before consuming any of them.
    factory = object.__new__(RealDetectorFactory)
    with pytest.raises(ValueError, match="retired.*poi_height"):
        factory.create(None, settings, 0)
