"""Focused telemetry-boundary tests for hardware SIYI collaborators."""

from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.hardware.polling import (
    SiyiTelemetryPoller,
)
from navpy.modules.vision.peripheral.siyi.hardware.session import SiyiSdkSession
from navpy.modules.vision.peripheral.siyi.hardware.state import SiyiReadbackStore


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def _store() -> SiyiReadbackStore:
    return SiyiReadbackStore(GimbalData(att=Attitude(0.0, 0.0, 0.0)))


def test_zero_roll_sample_sets_readiness_and_keeps_exact_attitude_stamp() -> None:
    store = _store()

    assert store.accept_attitude((9, 123.125, 10.0, -20.0, 0.0))

    snapshot = store.snapshot()
    assert store.attitude_ready.is_set()
    assert snapshot.data.att == Attitude(-20.0, 10.0, 0.0)
    assert snapshot.data.timestamp_s == 123.125
    assert snapshot.attitude_sample_id == 9


def test_zoom_admission_uses_sdk_receipt_and_rejects_duplicate_or_future() -> None:
    store = _store()

    assert store.accept_zoom((10, 3.0, 100.0), now_monotonic_s=200.0)
    assert not store.accept_zoom((10, 4.0, 199.0), now_monotonic_s=200.0)
    assert not store.accept_zoom((11, 4.0, 201.0), now_monotonic_s=200.0)

    snapshot = store.snapshot()
    assert snapshot.zoom_level == 3.0
    assert snapshot.zoom_receipt_monotonic_s == 100.0
    assert snapshot.zoom_sample_id == 10


def test_new_zoom_sequence_replaces_value_without_rewriting_receipt_time() -> None:
    store = _store()
    assert store.accept_zoom((10, 3.0, 100.0), now_monotonic_s=200.0)

    assert store.accept_zoom((11, 4.0, 150.0), now_monotonic_s=200.0)

    snapshot = store.snapshot()
    assert snapshot.zoom_level == 4.0
    assert snapshot.zoom_receipt_monotonic_s == 150.0
    assert snapshot.zoom_sample_id == 11


def test_poll_commit_publishes_attitude_and_zoom_in_one_snapshot() -> None:
    store = _store()

    accepted = store.commit_poll(
        (9, 123.125, 10.0, -20.0, 5.0),
        (11, 4.0, 150.0),
        now_monotonic_s=200.0,
    )

    snapshot = store.snapshot()
    assert accepted == (True, True)
    assert snapshot.data.att == Attitude(-20.0, 10.0, 5.0)
    assert snapshot.data.timestamp_s == 123.125
    assert snapshot.attitude_sample_id == 9
    assert snapshot.zoom_level == 4.0
    assert snapshot.zoom_receipt_monotonic_s == 150.0
    assert snapshot.zoom_sample_id == 11


@pytest.mark.parametrize(
    "sample",
    [
        None,
        (True, 1.0, 0.0, 0.0, 0.0),
        (1, 0.0, 0.0, 0.0, 0.0),
        (1, 1.0, float("nan"), 0.0, 0.0),
    ],
)
def test_malformed_attitude_samples_do_not_claim_readiness(sample) -> None:
    store = _store()

    assert not store.accept_attitude(sample)
    assert not store.attitude_ready.is_set()
    assert store.snapshot().data.timestamp_s is None


def test_reset_session_clears_all_dynamic_provenance() -> None:
    store = _store()
    assert store.accept_attitude((7, 10.0, 1.0, 2.0, 180.0))
    assert store.accept_zoom((11, 4.0, 9.0), now_monotonic_s=10.0)
    store.set_mount_orientation()

    store.reset_session()

    snapshot = store.snapshot()
    assert snapshot.data.timestamp_s is None
    assert snapshot.attitude_sample_id is None
    assert snapshot.zoom_level is None
    assert snapshot.zoom_receipt_monotonic_s is None
    assert snapshot.zoom_sample_id is None
    assert snapshot.pitch_sign == 1.0
    assert not store.attitude_ready.is_set()


def test_poller_logs_expected_io_error_but_does_not_suppress_programming_error() -> None:
    sdk = Mock()
    sdk.getAttitudeSample.side_effect = OSError("socket closed")
    sdk.getCurrentZoomLevelSample.return_value = (10, 2.0, 90.0)
    sdk.requestCurrentZoomLevel.return_value = True
    session = SiyiSdkSession()
    session.install(sdk)
    store = _store()
    logger = _Logger()
    poller = SiyiTelemetryPoller(session, store, lambda: 100.0, logger)

    poller.poll_once()

    assert logger.warnings == ["SIYI attitude poll error: socket closed"]
    assert store.snapshot().zoom_receipt_monotonic_s == 90.0

    sdk.getAttitudeSample.side_effect = RuntimeError("driver contract bug")
    with pytest.raises(RuntimeError, match="driver contract bug"):
        poller.poll_once()


def test_poller_routes_both_samples_through_atomic_store_commit() -> None:
    attitude = (9, 123.125, 10.0, -20.0, 5.0)
    zoom = (11, 4.0, 150.0)
    sdk = Mock()
    sdk.getAttitudeSample.return_value = attitude
    sdk.getCurrentZoomLevelSample.return_value = zoom
    session = SiyiSdkSession()
    session.install(sdk)
    store = Mock()
    poller = SiyiTelemetryPoller(session, store, lambda: 200.0, _Logger())

    poller.poll_once()

    store.commit_poll.assert_called_once_with(attitude, zoom, 200.0)
    sdk.requestCurrentZoomLevel.assert_called_once_with()
