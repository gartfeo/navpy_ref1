"""Unit tests for detector algorithms: IoU, Hungarian, and Kalman filter."""
import unittest
import numpy as np

from navpy.modules.vision.detector import iou_xyxy, hungarian, KalmanCV


class TestIoUXYXY(unittest.TestCase):
    """Test cases for iou_xyxy function."""

    def test_identical_boxes(self):
        """Identical boxes should have IoU = 1.0."""
        box = np.array([10, 10, 50, 50])
        self.assertAlmostEqual(iou_xyxy(box, box), 1.0)

    def test_no_overlap(self):
        """Non-overlapping boxes should have IoU = 0.0."""
        a = np.array([0, 0, 10, 10])
        b = np.array([20, 20, 30, 30])
        self.assertAlmostEqual(iou_xyxy(a, b), 0.0)

    def test_partial_overlap(self):
        """Test partial overlap."""
        a = np.array([0, 0, 10, 10])  # area = 100
        b = np.array([5, 5, 15, 15])  # area = 100
        # intersection: [5,5] to [10,10] = 5x5 = 25
        # union: 100 + 100 - 25 = 175
        expected = 25 / 175
        self.assertAlmostEqual(iou_xyxy(a, b), expected, places=4)

    def test_one_inside_other(self):
        """Box b fully inside box a."""
        a = np.array([0, 0, 100, 100])  # area = 10000
        b = np.array([25, 25, 75, 75])  # area = 2500
        # intersection = 2500, union = 10000
        expected = 2500 / 10000
        self.assertAlmostEqual(iou_xyxy(a, b), expected, places=4)

    def test_touching_boxes(self):
        """Boxes that touch but don't overlap."""
        a = np.array([0, 0, 10, 10])
        b = np.array([10, 0, 20, 10])
        self.assertAlmostEqual(iou_xyxy(a, b), 0.0)

    def test_malformed_box_x2_less_than_x1(self):
        """Malformed box with x2 < x1 should return 0."""
        a = np.array([50, 10, 10, 50])  # x2 < x1
        b = np.array([10, 10, 50, 50])
        self.assertAlmostEqual(iou_xyxy(a, b), 0.0)

    def test_malformed_box_y2_less_than_y1(self):
        """Malformed box with y2 < y1 should return 0."""
        a = np.array([10, 50, 50, 10])  # y2 < y1
        b = np.array([10, 10, 50, 50])
        self.assertAlmostEqual(iou_xyxy(a, b), 0.0)


class TestHungarian(unittest.TestCase):
    """Test cases for Hungarian algorithm."""

    def test_simple_assignment(self):
        """Simple 2x2 assignment."""
        cost = np.array([
            [1, 2],
            [3, 4]
        ], dtype=float)
        result = hungarian(cost)
        # Optimal: (0,0)=1, (1,1)=4, total=5
        # Or: (0,1)=2, (1,0)=3, total=5
        # Both are valid for this symmetric case
        self.assertEqual(len(result), 2)
        rows = [r for r, c in result]
        cols = [c for r, c in result]
        self.assertEqual(sorted(rows), [0, 1])
        self.assertEqual(sorted(cols), [0, 1])

    def test_rectangular_more_rows(self):
        """More rows than columns."""
        cost = np.array([
            [1, 2],
            [3, 4],
            [5, 6]
        ], dtype=float)
        result = hungarian(cost)
        # Should assign 2 pairs
        self.assertEqual(len(result), 2)
        cols = [c for r, c in result]
        self.assertEqual(sorted(cols), [0, 1])

    def test_rectangular_more_cols(self):
        """More columns than rows."""
        cost = np.array([
            [1, 2, 3],
            [4, 5, 6]
        ], dtype=float)
        result = hungarian(cost)
        # Should assign 2 pairs
        self.assertEqual(len(result), 2)
        rows = [r for r, c in result]
        self.assertEqual(sorted(rows), [0, 1])

    def test_single_element(self):
        """1x1 matrix."""
        cost = np.array([[5.0]])
        result = hungarian(cost)
        self.assertEqual(result, [(0, 0)])

    def test_clear_optimal(self):
        """Clear optimal assignment."""
        cost = np.array([
            [1, 100, 100],
            [100, 2, 100],
            [100, 100, 3]
        ], dtype=float)
        result = hungarian(cost)
        # Optimal is diagonal: (0,0), (1,1), (2,2) = 6
        self.assertEqual(len(result), 3)
        self.assertIn((0, 0), result)
        self.assertIn((1, 1), result)
        self.assertIn((2, 2), result)


class TestKalmanCV(unittest.TestCase):
    """Test cases for Kalman constant-velocity filter (cx, cy tracking)."""

    def test_initialization(self):
        """Test filter initialization with center coordinates."""
        kf = KalmanCV(cx=100.0, cy=200.0)

        # State should be initialized with position + zero velocities
        state = kf.x
        self.assertAlmostEqual(state[0], 100.0)  # cx
        self.assertAlmostEqual(state[1], 200.0)  # cy
        self.assertAlmostEqual(state[2], 0.0)    # vx
        self.assertAlmostEqual(state[3], 0.0)    # vy

    def test_predict_stationary(self):
        """Stationary object should stay in place."""
        kf = KalmanCV(cx=100.0, cy=200.0)

        # Predict one step with dt=1.0 (1 second)
        kf.predict(dt=1.0)

        # Should be near original position (no velocity)
        # State x = [cx, cy, vx, vy]
        self.assertAlmostEqual(kf.x[0], 100.0, places=0)
        self.assertAlmostEqual(kf.x[1], 200.0, places=0)

    def test_predict_with_velocity(self):
        """Object with velocity should move."""
        kf = KalmanCV(cx=100.0, cy=200.0)

        # Manually set velocity (pixels/second)
        kf.x[2] = 10.0  # vx = 10 pixels/s
        kf.x[3] = -5.0  # vy = -5 pixels/s

        # Predict with dt=1.0 second
        kf.predict(dt=1.0)

        # Should have moved according to velocity * dt
        # State x = [cx, cy, vx, vy]
        self.assertAlmostEqual(kf.x[0], 110.0, places=0)  # cx + vx*dt
        self.assertAlmostEqual(kf.x[1], 195.0, places=0)  # cy + vy*dt

    def test_update_corrects_state(self):
        """Update with measurement should correct state."""
        kf = KalmanCV(cx=100.0, cy=200.0)

        # Predict (drifts slightly due to noise model)
        kf.predict(dt=0.033)  # ~30fps

        # Update with new measurement
        z = np.array([105.0, 198.0], dtype=np.float32)
        kf.update(z)

        # State should be close to measurement
        self.assertAlmostEqual(kf.x[0], 105.0, delta=5.0)
        self.assertAlmostEqual(kf.x[1], 198.0, delta=5.0)

    def test_velocity_estimation(self):
        """Test that velocity is estimated from consecutive updates."""
        kf = KalmanCV(cx=100.0, cy=200.0)

        # Simulate moving object at 30fps
        dt = 0.033
        for i in range(10):
            kf.predict(dt=dt)
            z = np.array([100.0 + (i+1)*3, 200.0 - (i+1)*1.5], dtype=np.float32)
            kf.update(z)

        # After several updates, velocity should be estimated
        # vx should be positive (moving right)
        # vy should be negative (moving up in image coords)
        self.assertGreater(kf.x[2], 0)  # vx > 0
        self.assertLess(kf.x[3], 0)     # vy < 0


if __name__ == '__main__':
    unittest.main()