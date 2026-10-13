"""Sim-only OpenCV thread pinning is opt-in and validated."""

import cv2
import pytest

from navpy.sim.sim_cv_threads import SIM_CV_THREADS_ENV, apply_sim_cv_threads


@pytest.fixture
def restore_cv_threads():
    before = cv2.getNumThreads()
    yield
    cv2.setNumThreads(before)


def test_unset_keeps_the_host_default(restore_cv_threads):
    before = cv2.getNumThreads()
    assert apply_sim_cv_threads({}) is None
    assert cv2.getNumThreads() == before


def test_requested_count_is_applied(restore_cv_threads):
    assert apply_sim_cv_threads({SIM_CV_THREADS_ENV: "1"}) == 1
    assert cv2.getNumThreads() == 1


@pytest.mark.parametrize("raw", ["0", "-2", "one"])
def test_invalid_count_is_refused(raw, restore_cv_threads):
    with pytest.raises(ValueError):
        apply_sim_cv_threads({SIM_CV_THREADS_ENV: raw})
