"""Detector frame freshness for the camera HEARTBEAT (display only).

docs/design/gcs_preflight_camera_row.md, NavPy: ready only while the capture
runs and its newest publication is a frame of this run, at most
DETECTOR_FRAME_FRESH_S old on the publication clock.
"""

from __future__ import annotations

import numpy as np

from navpy.core.perception.camera.frame_provider import (
    DETECTOR_FRAME_FRESH_S,
    FrameProvider,
)
from navpy.core.perception.camera.frame_publication import FramePublicationStore

from tests.modules.vision.test_frame_provider import _Logger

FRAME = np.zeros((2, 2, 3), dtype=np.uint8)


class _Clock:
    def __init__(self) -> None:
        self.now_s = 1000.0

    def __call__(self) -> float:
        return self.now_s


def _running_store() -> tuple[FramePublicationStore, _Clock]:
    clock = _Clock()
    store = FramePublicationStore(wall_time=clock)
    store.set_running(True)
    return store, clock


def test_fresh_up_to_the_threshold_and_stale_after_it():
    store, clock = _running_store()
    store.publish(FRAME)

    clock.now_s += DETECTOR_FRAME_FRESH_S
    assert store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)
    clock.now_s += 1e-3
    assert not store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)


def test_no_frame_yet_is_not_fresh():
    store, _ = _running_store()

    assert not store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)


def test_a_none_publication_is_not_a_frame():
    store, _ = _running_store()
    store.publish(FRAME)
    store.publish(None)

    assert not store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)


def test_a_stopped_capture_is_not_ready_even_with_a_new_frame():
    store, _ = _running_store()
    store.publish(FRAME)
    store.set_running(False)

    assert not store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)


def test_a_restart_needs_a_frame_of_its_own_run():
    store, _ = _running_store()
    store.publish(FRAME)
    store.set_running(False)
    store.set_running(True)

    assert not store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)  # old run's frame
    store.publish(FRAME)
    assert store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)


def test_running_again_while_running_keeps_the_current_frame():
    store, _ = _running_store()
    store.publish(FRAME)
    store.set_running(True)

    assert store.has_fresh_frame(DETECTOR_FRAME_FRESH_S)


def test_frame_provider_answers_with_the_detector_threshold():
    provider = FrameProvider(None, _Logger())
    provider.start()
    try:
        assert not provider.has_fresh_frame()
        provider.push_frame(FRAME)
        assert provider.has_fresh_frame()
    finally:
        provider.stop()
    assert not provider.has_fresh_frame()
