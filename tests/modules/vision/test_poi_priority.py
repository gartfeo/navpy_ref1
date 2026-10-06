import unittest

import numpy as np

from navpy.modules.vision.poi_priority import (
    find_poi_by_id,
    prioritize_pois,
    select_most_centered_poi,
)
from tests.detection_factory import make_detected_poi


def _create_poi(obj_id: int, x_error: float, y_error: float, k=None):
    return make_detected_poi(
        obj_id=obj_id,
        x_error=x_error,
        y_error=y_error,
        k=k,
    )


class TestPoiPriority(unittest.TestCase):
    def test_select_most_centered_poi_uses_principal_point(self):
        k = np.array([
            [1000.0, 0.0, 1000.0],
            [0.0, 1000.0, 500.0],
            [0.0, 0.0, 1.0],
        ])
        far_poi = _create_poi(1, 1450.0, 800.0, k=k)
        centered_poi = _create_poi(2, 1010.0, 505.0, k=k)

        result = select_most_centered_poi([far_poi, centered_poi])

        self.assertIs(result, centered_poi)

    def test_select_most_centered_poi_falls_back_for_invalid_pixels(self):
        # A grouped visual observation always carries calibration; malformed
        # measurements still fail closed to deterministic input ordering.
        poi1 = _create_poi(1, np.nan, np.nan)
        poi2 = _create_poi(2, np.nan, np.nan)

        result = select_most_centered_poi([poi1, poi2])

        self.assertIs(result, poi1)

    def test_prioritize_pois_moves_primary_to_front(self):
        poi1 = _create_poi(1, 0.0, 0.0)
        poi2 = _create_poi(2, 0.0, 0.0)
        poi3 = _create_poi(3, 0.0, 0.0)

        result = prioritize_pois([poi1, poi2, poi3], poi2)

        self.assertEqual(
            [poi.identity.obj_id for poi in result],
            [2, 1, 3],
        )

    def test_find_poi_by_id_returns_match(self):
        poi1 = _create_poi(1, 0.0, 0.0)
        poi2 = _create_poi(2, 0.0, 0.0)

        result = find_poi_by_id([poi1, poi2], 2)

        self.assertIs(result, poi2)


if __name__ == "__main__":
    unittest.main()
