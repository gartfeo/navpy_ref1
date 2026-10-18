"""Profile validation of a camera device's optional ``video_streams``."""

from __future__ import annotations

from unittest.mock import Mock

import pytest

from navpy.app.perception.vision_camera_factory import build_camera_mounts
from navpy.core.perception.profile.vision_video_streams import parse_video_streams
from navpy.core.ports.video_stream import VideoStreamSpec


def _stream(**overrides) -> dict:
    stream = {
        "stream_id": 1,
        "name": "tracking overlay",
        "uri": "rtsp://192.168.144.30:8554/overlay",
        "encoding": "H264",
        "resolution_h": 1280,
        "resolution_v": 720,
        "framerate": 30,
    }
    stream.update(overrides)
    return stream


def _device(streams) -> dict:
    return {
        "name": "cam",
        "camera": {
            "image_width": 640,
            "image_height": 480,
            "intrinsics": {
                "zooms": {"1": {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}}
            },
        },
        "gimbal": {"camera_pitch": -30},
        "video_streams": streams,
    }


def test_no_key_means_no_streams():
    assert parse_video_streams({"name": "cam"}, "cam") == ()


def test_valid_streams_parse_in_stream_id_order():
    streams = parse_video_streams(
        {"video_streams": [_stream(stream_id=2, encoding="H265"), _stream()]}, "cam"
    )

    assert streams == (
        VideoStreamSpec(1, "tracking overlay", "rtsp://192.168.144.30:8554/overlay",
                        "H264", 1280, 720, 30.0),
        VideoStreamSpec(2, "tracking overlay", "rtsp://192.168.144.30:8554/overlay",
                        "H265", 1280, 720, 30.0),
    )


@pytest.mark.parametrize(
    "streams, message",
    [
        pytest.param({"stream_id": 1}, "must be a list", id="not-a-list"),
        pytest.param(["rtsp://x"], r"\[0\] must be a mapping", id="entry-not-mapping"),
        pytest.param([_stream(stream_id=0)], "stream_id must be an integer in 1..255", id="id-zero"),
        pytest.param([_stream(stream_id=256)], "stream_id", id="id-too-big"),
        pytest.param([_stream(stream_id=True)], "stream_id", id="id-bool"),
        pytest.param([_stream(stream_id=1.0)], "stream_id", id="id-float"),
        pytest.param([_stream(), _stream()], "duplicate stream_id 1", id="duplicate-id"),
        pytest.param([_stream(stream_id=2)], r"1\.\.1 without gaps", id="id-gap-single"),
        pytest.param([_stream(), _stream(stream_id=3)], r"1\.\.2 without gaps", id="id-gap"),
        pytest.param([_stream(encoding="MJPEG")], "encoding must be one of", id="encoding"),
        pytest.param([_stream(uri="http://192.168.144.30/x")], "rtsp://", id="uri-scheme"),
        pytest.param([_stream(uri="rtsp://")], "must name a host", id="uri-no-host"),
        pytest.param([_stream(uri="rtsp://a b")], "without spaces", id="uri-space"),
        pytest.param([_stream(uri="rtsp://" + "a" * 160)], "at most 160", id="uri-too-long"),
        pytest.param([_stream(name="")], "name must be a non-empty", id="name-empty"),
        pytest.param([_stream(name="n" * 33)], "at most 32", id="name-too-long"),
        pytest.param([_stream(resolution_h=0)], "resolution_h", id="resolution-zero"),
        pytest.param([_stream(resolution_v=70000)], "resolution_v", id="resolution-too-big"),
        pytest.param([_stream(framerate=0)], "framerate must be a positive", id="framerate-zero"),
        pytest.param([_stream(framerate=float("nan"))], "framerate", id="framerate-nan"),
        pytest.param(
            [{k: v for k, v in _stream().items() if k != "uri"}], "missing uri", id="missing-key"
        ),
        pytest.param([_stream(bitrate=1)], "unknown keys bitrate", id="unknown-key"),
    ],
)
def test_bad_entries_fail_with_a_clear_error(streams, message):
    with pytest.raises(ValueError, match=message) as error:
        parse_video_streams({"video_streams": streams}, "cam")
    assert "device 'cam' video_streams" in str(error.value)


def test_unique_ids_are_per_camera():
    profile = {"devices": [_device([_stream()]), {**_device([_stream()]), "name": "cam2"}]}

    specs = build_camera_mounts(profile, Mock(), Mock(), use_sim_gimbal=False)

    assert len(specs) == 2


def test_profile_load_rejects_bad_streams():
    profile = {"devices": [_device([_stream(), _stream()])]}

    with pytest.raises(ValueError, match="duplicate stream_id 1"):
        build_camera_mounts(profile, Mock(), Mock(), use_sim_gimbal=False)
