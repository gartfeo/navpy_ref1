"""One camera's discovery state machine (gcs_video_streams.md, section 1)."""
from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import (
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_UNSUPPORTED,
    MAVLINK_MSG_ID_CAMERA_INFORMATION as INFO,
    MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION as STREAMS,
)

from gcs.backend.video_camera_discovery import (
    PHASE_DONE, PHASE_FAILED, CameraDiscovery, CameraRequest, VideoStream,
)
from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_LOST_AFTER_S, CAMERA_REQUEST_MAX_SENDS, CAMERA_REQUEST_TIMEOUT_S,
)

TIMEOUT = CAMERA_REQUEST_TIMEOUT_S


def _stream(stream_id: int) -> VideoStream:
    return VideoStream(stream_id, f"s{stream_id}", "H264", "192.168.144.10", f"p{stream_id}")


def _at_streams(now: float = 0.0) -> CameraDiscovery:
    camera = CameraDiscovery(100, now)
    camera.poll(now)
    camera.on_camera_information(CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)
    return camera


def test_first_poll_requests_camera_information() -> None:
    camera = CameraDiscovery(100, 0.0)
    assert camera.poll(0.0) == CameraRequest(INFO, 0, 0)


def test_requests_are_serialized_one_outstanding() -> None:
    camera = CameraDiscovery(100, 0.0)
    assert camera.poll(0.0) is not None
    # Nothing new while the first request waits for its payload.
    assert camera.poll(0.5) is None
    assert camera.poll(TIMEOUT - 0.001) is None


def test_request_timeout_boundary() -> None:
    camera = CameraDiscovery(100, 0.0)
    camera.poll(0.0)
    assert camera.poll(TIMEOUT - 1e-6) is None
    assert camera.poll(TIMEOUT) == CameraRequest(INFO, 0, 1)


def test_ack_without_payload_still_retries() -> None:
    camera = CameraDiscovery(100, 0.0)
    camera.poll(0.0)
    camera.on_ack(MAV_RESULT_ACCEPTED)
    assert camera.poll(TIMEOUT - 1e-6) is None
    assert camera.poll(TIMEOUT) == CameraRequest(INFO, 0, 1)


def test_rejecting_ack_fails_the_send_at_once() -> None:
    camera = CameraDiscovery(100, 0.0)
    camera.poll(0.0)
    camera.on_ack(MAV_RESULT_UNSUPPORTED)
    assert camera.poll(0.1) == CameraRequest(INFO, 0, 1)


def test_retry_exhaustion_fails_and_stays_failed() -> None:
    camera = CameraDiscovery(100, 0.0)
    sends = [camera.poll(0.0)]
    for k in range(1, CAMERA_REQUEST_MAX_SENDS + 1):
        sends.append(camera.poll(k * TIMEOUT))
    assert [s.confirmation for s in sends[:-1]] == list(range(CAMERA_REQUEST_MAX_SENDS))
    assert sends[-1] is None
    assert camera.phase == PHASE_FAILED
    assert camera.poll(100.0) is None
    # A late answer does not revive a failed generation.
    camera.on_camera_information(CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)
    assert camera.phase == PHASE_FAILED and camera.streams() == []


def test_camera_without_video_needs_no_stream_request() -> None:
    camera = CameraDiscovery(100, 0.0)
    camera.poll(0.0)
    camera.on_camera_information(0)
    assert camera.poll(0.1) is None
    assert camera.phase == PHASE_DONE and camera.streams() == []


def test_video_flag_requests_all_streams_then_collects_against_count() -> None:
    camera = _at_streams()
    assert camera.poll(0.1) == CameraRequest(STREAMS, 0, 0)
    camera.on_stream_information(2, 2, _stream(2))
    assert camera.streams() == []  # not done until every id arrived
    camera.on_stream_information(1, 2, _stream(1))
    assert camera.phase == PHASE_DONE
    assert [s.stream_id for s in camera.streams()] == [1, 2]
    assert camera.poll(10.0) is None


def test_missing_id_is_requested_singly_and_recovers() -> None:
    camera = _at_streams()
    camera.poll(0.0)
    camera.on_stream_information(1, 3, _stream(1))
    camera.on_stream_information(3, 3, _stream(3))
    # The all-streams send times out with id 2 missing: ask for id 2 only.
    assert camera.poll(TIMEOUT) == CameraRequest(STREAMS, 2, 0)
    assert camera.poll(TIMEOUT + TIMEOUT / 2) is None
    assert camera.poll(2 * TIMEOUT) == CameraRequest(STREAMS, 2, 1)
    camera.on_stream_information(2, 3, _stream(2))
    assert camera.phase == PHASE_DONE
    assert [s.stream_id for s in camera.streams()] == [1, 2, 3]


def test_missing_id_retry_exhaustion_fails() -> None:
    camera = _at_streams()
    camera.poll(0.0)
    camera.on_stream_information(1, 2, _stream(1))
    now = TIMEOUT
    assert camera.poll(now) == CameraRequest(STREAMS, 2, 0)
    for _ in range(CAMERA_REQUEST_MAX_SENDS):
        now += TIMEOUT
        camera.poll(now)
    assert camera.phase == PHASE_FAILED and camera.streams() == []


def test_all_streams_without_any_answer_resends_all() -> None:
    camera = _at_streams()
    camera.poll(0.0)
    assert camera.poll(TIMEOUT) == CameraRequest(STREAMS, 0, 1)


def test_rejected_uri_counts_as_received_but_is_not_shown() -> None:
    camera = _at_streams()
    camera.poll(0.0)
    camera.on_stream_information(1, 2, None)
    camera.on_stream_information(2, 2, _stream(2))
    assert camera.phase == PHASE_DONE
    assert [s.stream_id for s in camera.streams()] == [2]


def test_out_of_range_and_inconsistent_count_are_ignored() -> None:
    camera = _at_streams()
    camera.poll(0.0)
    camera.on_stream_information(1, 2, _stream(1))
    camera.on_stream_information(3, 2, _stream(3))  # id above count
    camera.on_stream_information(2, 5, _stream(2))  # different count
    assert camera.phase != PHASE_DONE


def test_lost_boundary() -> None:
    camera = CameraDiscovery(100, 0.0)
    camera.heartbeat(1.0)
    assert not camera.is_lost(1.0 + CAMERA_LOST_AFTER_S - 1e-6)
    assert camera.is_lost(1.0 + CAMERA_LOST_AFTER_S)


def test_preflight_fields_start_empty_and_follow_the_answers() -> None:
    camera = CameraDiscovery(100, 0.0)
    assert (camera.system_status, camera.vendor, camera.has_video) == (None, None, None)
    camera.heartbeat(0.5, 4)
    assert camera.system_status == 4
    camera.poll(0.5)
    assert camera.on_camera_information(0, "NavPy") is True
    assert (camera.vendor, camera.has_video, camera.phase) == ("NavPy", False, PHASE_DONE)
    # One CAMERA_INFORMATION per generation: a second is ignored.
    assert camera.on_camera_information(CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM, "other") is False
    assert (camera.vendor, camera.has_video) == ("NavPy", False)


def test_rejected_reason_is_kept_until_a_valid_answer_replaces_it() -> None:
    camera = _at_streams()
    camera.poll(0.0)
    camera.on_stream_information(1, 2, None, "userinfo")
    assert camera.rejected() == [(1, "userinfo")]
    camera.on_stream_information(1, 2, _stream(1))
    assert camera.rejected() == []
