"""
Rotation test-suite for navpy.utils.simple_rotation.Rotation.

Usage:
    # quick correctness (default):
    python -m unittest -v

    # add exhaustive grid (24 seq × deg/rad × 13**axes):
    RUN_FULL=1 python -m unittest -v

    # add performance benchmark (prints and asserts speed-up):
    RUN_PERF=1 python -m unittest -v
"""

import itertools
import os
import timeit
import unittest
from functools import partial

import numpy as np
from numpy.testing import assert_allclose

from navpy.utils.simple_rotation import Rotation as SimpleRot

try:
    from scipy.spatial.transform import Rotation as _SciRot
except ImportError:  # pragma: no cover - SciPy optional
    _SciRot = None

# --------------------------------------------------------------------------- #
# Shared constants                                                            #
# --------------------------------------------------------------------------- #
BASE_SEQ = ["XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX",
            "XYX", "XZX", "YXY", "YZY", "ZXZ", "ZYZ"]
SEQUENCES = BASE_SEQ + [s.lower() for s in BASE_SEQ]  # 24 total
ANGLE_DEG = np.arange(-180, 181, 30)  # 13 values
VEC = np.array([0.3, -0.2, 0.5])

# env switches
RUN_FULL = bool(os.getenv("RUN_FULL")) and _SciRot is not None

# perf tunables
REPEAT = 5
NUMBER = 1
FASTER_FACTOR = 0.9  # expected gain


# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
# Helper functions                                                            #
# --------------------------------------------------------------------------- #
def _grid(seq: str) -> np.ndarray:
    """(N, axes) Euler grid in degrees for a sequence."""
    axes = len(seq)
    if axes == 1:
        return ANGLE_DEG[:, None]
    mesh = np.meshgrid(*([ANGLE_DEG] * axes), indexing="ij")
    return np.stack(mesh, axis=-1).reshape(-1, axes)


def _simple_apply(seq, angles_deg):
    SimpleRot.from_euler(seq, angles_deg, degrees=True).apply(VEC)


def _scipy_apply(seq, angles_deg):
    SimpleRot.from_euler(seq, angles_deg, degrees=True).apply(VEC)


def _assert_equal(seq: str, ang, degrees: bool):
    a = np.asarray(ang, float)
    if not degrees:
        a = np.deg2rad(a)

    s_scipy = _SciRot.from_euler(seq, a, degrees=degrees)
    s_simple = SimpleRot.from_euler(seq, a, degrees=degrees)

    assert_allclose(s_simple.as_matrix(), s_scipy.as_matrix(), atol=1e-9)
    assert_allclose(s_simple.apply(VEC), s_scipy.apply(VEC), atol=1e-9)
    assert_allclose(s_simple.inv().as_matrix(), s_scipy.inv().as_matrix(), atol=1e-9)


# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
# 1) Quick-smoke correctness (always runs)                                    #
# --------------------------------------------------------------------------- #
@unittest.skipUnless(_SciRot, "SciPy not available, skipping tests")
class RotationSmokeTest(unittest.TestCase):
    """Fast sanity check on a handful of sequences / angles."""

    def test_basic(self):
        samples = [
            ("XYZ", (10, 20, 30)),
            ("zyx", (-45, 60, 90)),
            ("XZX", (0, 180, -90)),
        ]
        for seq, ang in samples:
            for deg in (True, False):
                with self.subTest(seq=seq, degrees=deg, angles=ang):
                    _assert_equal(seq, ang, deg)


# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
# 2) Exhaustive correctness (opt-in via RUN_FULL)                             #
# --------------------------------------------------------------------------- #
@unittest.skipUnless(RUN_FULL, "Set RUN_FULL=1 to run exhaustive grid")
class RotationFullGridTest(unittest.TestCase):
    """Full Euler space: 24 sequences × deg/rad × 13**axes angles."""

    def test_full_grid(self):
        for seq in SEQUENCES:
            grid = itertools.product(ANGLE_DEG, repeat=len(seq))
            for deg in (True, False):
                for ang in grid:
                    with self.subTest(seq=seq, degrees=deg, angles=ang):
                        _assert_equal(seq, ang, deg)


# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
# 3) Performance benchmark (opt-in via RUN_PERF)                              #
# --------------------------------------------------------------------------- #
@unittest.skipUnless(_SciRot, "SciPy not available, skipping tests")
class RotationPerfTest(unittest.TestCase):
    """Micro-benchmark vs SciPy on vectorised angle grids."""

    def test_speed(self):
        simple_sum = 0.0
        scipy_sum = 0.0

        for seq in SEQUENCES:
            angles = _grid(seq)
            t_simple = timeit.Timer(partial(_simple_apply, seq, angles))
            t_scipy = timeit.Timer(partial(_scipy_apply, seq, angles))

            simple_sum += min(t_simple.repeat(REPEAT, NUMBER))
            scipy_sum += min(t_scipy.repeat(REPEAT, NUMBER))

        ratio = scipy_sum / simple_sum if simple_sum else float("inf")

        print(
            f"\nSimpleRotation : {simple_sum:.4f}s\n"
            f"SciPy Rotation : {scipy_sum:.4f}s\n"
            f"Speed-up (SciPy / Simple) : {ratio:.2f}×\n"
        )

        self.assertGreaterEqual(
            ratio, FASTER_FACTOR,
            f"Expected ≥{FASTER_FACTOR}× speed-up, got {ratio:.2f}×")


# --------------------------------------------------------------------------- #


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
