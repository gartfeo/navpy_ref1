"""The thumbnail label names the dock class; other class IDs stay numbers."""

import pytest

from navpy.core.perception.profile.vision_class_profile import DOCK_DETECT_CLASS_ID
from navpy.core.review.confirmation_media import poi_class_name


def test_dock_class_is_named_dock():
    assert poi_class_name(DOCK_DETECT_CLASS_ID) == "Dock"


@pytest.mark.parametrize("class_id", [1, 2, 3, 4, 17])
def test_other_model_class_is_identified_by_number(class_id):
    assert poi_class_name(class_id) == f"Detection class {class_id}"
