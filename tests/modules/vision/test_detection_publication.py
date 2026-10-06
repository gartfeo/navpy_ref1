from dataclasses import FrozenInstanceError

import pytest

from navpy.modules.vision.models.detection_publication import DetectionPublication


def _publication(pois):
    return DetectionPublication(
        pois,
        source_timestamp_s=10.0,
        source_receipt_timestamp_s=100.0,
        source_name="gimbal_0",
        source_discontinuity=False,
    )


def test_normalizes_input_list_to_detached_tuple():
    poi = object()
    source = [poi]

    publication = _publication(source)
    source.clear()

    assert publication.detected_pois == (poi,)


def test_publication_fields_cannot_be_reassigned():
    publication = _publication([])

    with pytest.raises(FrozenInstanceError):
        publication.source_name = "other"


def test_primary_poi_is_first_ordered_detection():
    first = object()
    second = object()

    assert _publication([first, second]).primary_poi is first


def test_empty_publication_has_no_primary_poi():
    assert _publication([]).primary_poi is None


def test_with_pois_reorders_without_mutating_original():
    first = object()
    second = object()
    original = _publication([first, second])

    reordered = original.with_pois([second, first])

    assert original.detected_pois == (first, second)
    assert reordered.detected_pois == (second, first)
    assert reordered.source_timestamp_s == original.source_timestamp_s
    assert reordered.source_receipt_timestamp_s == (
        original.source_receipt_timestamp_s
    )
    assert reordered.source_name == original.source_name
    assert reordered.source_discontinuity is original.source_discontinuity
