"""Names must not imply that arbitrary model IDs identify a dock or vehicle."""

import pytest

from navpy.modules.nav.confirmation_media import poi_class_name


@pytest.mark.parametrize("class_id", [0, 1, 2, 3, 4, 17])
def test_unverified_model_class_is_identified_by_number(class_id):
    assert poi_class_name(class_id) == f"Detection class {class_id}"


def test_delivery_hub_keeps_its_operator_defined_type():
    assert poi_class_name(0, "vehicle") == "Vehicle"
    assert poi_class_name(0, "operations_site") == "Operations site"
