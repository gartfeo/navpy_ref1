"""A frame rendered at capture and committed at publish (``deterministic_dev_sim.md`` step 5)."""

from __future__ import annotations

from tests.modules.vision.test_sim_frame_transactions import ATTITUDE, LOCATION, _coordinator, _detection, _pipeline


def _capture(pipeline):
    return pipeline.capture_frame(
        LOCATION, ATTITUDE, attitude_time_boot_s=10.0,
        frame_receipt_timestamp_s=5.0, frame_air_speed_mps=17.0,
    )


def test_a_captured_frame_renders_now_and_commits_only_when_published():
    coordinator, store, _outcomes = _coordinator()
    renders = []
    pipeline, capture, tracking, _gap = _pipeline(
        coordinator, lambda *args, **kwargs: renders.append(kwargs["timestamp_s"]) or _detection(),
        tracking_id=None,
    )
    captured = _capture(pipeline)
    assert renders == [10.0] and captured.timestamp_s == 10.0
    tracking.update.assert_not_called()
    capture.capture.assert_not_called()
    assert store.snapshot().detected_pois == ()

    assert captured.publish(7.5) is True
    assert renders == [10.0]  # no second render at publish
    (detections, _loc, _att, timestamp_s), _ = tracking.update.call_args
    assert timestamp_s == 10.0  # the capture time is kept
    published = store.snapshot().detected_pois
    assert len(published) == 1 and published == tuple(detections)
    timing = published[0].timing
    # The receipt is the publication, not the pose's receipt at capture.
    assert timing.source_receipt_timestamp_s == 7.5
    assert timing.source_air_speed_mps == 17.0


def test_a_reset_between_capture_and_publish_rejects_the_frame():
    coordinator, store, _outcomes = _coordinator()
    pipeline, capture, tracking, _gap = _pipeline(coordinator, lambda *a, **k: _detection(), tracking_id=None)
    captured = _capture(pipeline)
    with coordinator.reset() as resetting:
        assert resetting
    assert captured.publish(7.5) is False
    tracking.update.assert_not_called()
    capture.capture.assert_not_called()
    assert store.snapshot().detected_pois == ()
