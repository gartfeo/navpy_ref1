import unittest

import numpy as np

from navpy.modules.vision.target_priority import (
    find_target_by_id,
    prioritize_targets,
    select_most_centered_target,
)
from tests.detection_factory import make_detected_target


def _create_target(obj_id: int, x_error: float, y_error: float, k=None):
    return make_detected_target(
        obj_id=obj_id,
        x_error=x_error,
        y_error=y_error,
        k=k,
    )


class TestTargetPriority(unittest.TestCase):
    def test_select_most_centered_target_uses_principal_point(self):
        k = np.array([
            [1000.0, 0.0, 1000.0],
            [0.0, 1000.0, 500.0],
            [0.0, 0.0, 1.0],
        ])
        far_target = _create_target(1, 1450.0, 800.0, k=k)
        centered_target = _create_target(2, 1010.0, 505.0, k=k)

        result = select_most_centered_target([far_target, centered_target])

        self.assertIs(result, centered_target)

    def test_select_most_centered_target_falls_back_for_invalid_pixels(self):
        # A grouped visual observation always carries calibration; malformed
        # measurements still fail closed to deterministic input ordering.
        target1 = _create_target(1, np.nan, np.nan)
        target2 = _create_target(2, np.nan, np.nan)

        result = select_most_centered_target([target1, target2])

        self.assertIs(result, target1)

    def test_prioritize_targets_moves_primary_to_front(self):
        target1 = _create_target(1, 0.0, 0.0)
        target2 = _create_target(2, 0.0, 0.0)
        target3 = _create_target(3, 0.0, 0.0)

        result = prioritize_targets([target1, target2, target3], target2)

        self.assertEqual(
            [target.identity.obj_id for target in result],
            [2, 1, 3],
        )

    def test_find_target_by_id_returns_match(self):
        target1 = _create_target(1, 0.0, 0.0)
        target2 = _create_target(2, 0.0, 0.0)

        result = find_target_by_id([target1, target2], 2)

        self.assertIs(result, target2)


if __name__ == "__main__":
    unittest.main()
