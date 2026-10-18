"""Per-vehicle video-stream discovery over MAVLink and the snapshot field."""
from __future__ import annotations

import ipaddress
import logging
from types import SimpleNamespace
from typing import Any

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,
    MAV_CMD_REQUEST_MESSAGE,
    MAV_COMP_ID_AUTOPILOT1,
    MAV_COMP_ID_CAMERA,
    MAV_COMP_ID_CAMERA2,
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_RESULT_UNSUPPORTED,
    MAV_TYPE_CAMERA,
    MAV_TYPE_FIXED_WING,
    MAVLINK_MSG_ID_CAMERA_INFORMATION as INFO,
    MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION as STREAMS,
    VIDEO_STREAM_ENCODING_H264,
    VIDEO_STREAM_ENCODING_H265,
)

from gcs.backend import video_stream_policy, video_streams
from gcs.backend.settings_model import GcsSettings
from gcs.backend.telemetry_loop import _has_changed
from gcs.backend.vehicle_entry import VehicleEntry
from gcs.backend.video_stream_policy import VideoPolicy
from gcs.backend.video_streams import VehicleVideoStreams
from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_LOST_AFTER_S, CAMERA_REQUEST_MAX_SENDS, CAMERA_REQUEST_TIMEOUT_S,
)

SYS = 4
GCS_SYS = 255
CAM1, CAM2 = MAV_COMP_ID_CAMERA, MAV_COMP_ID_CAMERA2
POLICY = VideoPolicy((ipaddress.ip_network("192.168.144.0/24"),), 8889)


class Clock:
    def __init__(self) -> None:
        self.now = 50.0

    def __call__(self) -> float:
        return self.now


class Sink:
    transport_source_system = GCS_SYS

    def __init__(self) -> None:
        self.sent: list[Any] = []

    def send_mavlink_message(self, message: Any) -> None:
        self.sent.append(message)

    def requests(self) -> list[tuple[int, int, int, int]]:
        """(target component, param1 message id, param2, confirmation)."""
        return [(m.target_component, int(m.param1), int(m.param2), m.confirmation)
                for m in self.sent]


def _msg(component: int, src_system: int = SYS, **fields: Any) -> SimpleNamespace:
    return SimpleNamespace(
        get_srcSystem=lambda: src_system, get_srcComponent=lambda: component, **fields,
    )


def _hb(component: int, src_system: int = SYS, mav_type: int = MAV_TYPE_CAMERA) -> SimpleNamespace:
    return _msg(component, src_system, type=mav_type)


def _info(component: int, flags: int = CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) -> SimpleNamespace:
    return _msg(component, flags=flags)


def _stream(component: int, stream_id: int, count: int, uri: str,
            name: str = "tracking overlay", encoding: int = VIDEO_STREAM_ENCODING_H264,
            ) -> SimpleNamespace:
    return _msg(component, stream_id=stream_id, count=count, uri=uri, name=name,
                encoding=encoding)


def _ack(component: int, result: int, target_system: int = GCS_SYS) -> SimpleNamespace:
    return _msg(component, command=MAV_CMD_REQUEST_MESSAGE, result=result,
                target_system=target_system)


@pytest.fixture()
def clock(monkeypatch: pytest.MonkeyPatch) -> Clock:
    clock = Clock()
    monkeypatch.setattr(video_streams.rt, "monotonic", clock)
    return clock


@pytest.fixture()
def rig(clock: Clock) -> tuple[VehicleVideoStreams, Sink]:
    sink = Sink()
    return VehicleVideoStreams(SYS, sink, policy=lambda: POLICY), sink


def _discover(streams: VehicleVideoStreams, component: int, uri: str, stream_id: int = 1,
              ) -> None:
    streams.on_heartbeat(_hb(component))
    streams.on_camera_information(_info(component))
    streams.on_video_stream_information(_stream(component, stream_id, 1, uri))


def test_camera_heartbeat_starts_generation_with_info_request(rig) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))
    assert sink.requests() == [(CAM1, INFO, 0, 0)]
    message = sink.sent[0]
    assert message.target_system == SYS and message.command == MAV_CMD_REQUEST_MESSAGE
    # A further heartbeat of a known camera starts nothing new.
    streams.on_heartbeat(_hb(CAM1))
    assert len(sink.sent) == 1


@pytest.mark.parametrize("message", [
    _hb(CAM1, src_system=SYS + 1),                  # another vehicle's camera
    _hb(MAV_COMP_ID_AUTOPILOT1),                    # autopilot-proxied camera
    _hb(MAV_COMP_ID_ONBOARD_COMPUTER),              # the companion itself
    _hb(CAM1, mav_type=MAV_TYPE_FIXED_WING),        # not a camera heartbeat
])
def test_only_own_independent_camera_heartbeats_count(rig, message) -> None:
    streams, sink = rig
    streams.on_heartbeat(message)
    assert sink.sent == []


def test_full_discovery_publishes_snapshot(rig) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_camera_information(_info(CAM1))
    assert sink.requests()[-1] == (CAM1, STREAMS, 0, 0)
    streams.on_video_stream_information(
        _stream(CAM1, 2, 2, "rtsp://192.168.144.10:8554/raw", name="raw",
                encoding=VIDEO_STREAM_ENCODING_H265))
    streams.on_video_stream_information(
        _stream(CAM1, 1, 2, "rtsp://192.168.144.10:8554/tracking", encoding=0))
    assert streams.snapshot() == [
        {"key": f"{CAM1}:1", "camera_component": CAM1, "stream_id": 1,
         "name": "tracking overlay", "encoding": None,
         "whep_url": "http://192.168.144.10:8889/tracking/whep"},
        {"key": f"{CAM1}:2", "camera_component": CAM1, "stream_id": 2,
         "name": "raw", "encoding": "H265",
         "whep_url": "http://192.168.144.10:8889/raw/whep"},
    ]


def test_tick_resends_after_timeout_and_ack_alone_does_not_complete(rig, clock) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_command_ack(_ack(CAM1, 0))
    clock.now += CAMERA_REQUEST_TIMEOUT_S - 0.01
    streams.tick()
    assert len(sink.sent) == 1
    clock.now += 0.01
    streams.tick()
    assert sink.requests() == [(CAM1, INFO, 0, 0), (CAM1, INFO, 0, 1)]


def test_rejecting_ack_for_another_gcs_is_ignored(rig) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_command_ack(_ack(CAM1, MAV_RESULT_UNSUPPORTED, target_system=GCS_SYS - 1))
    assert len(sink.sent) == 1
    streams.on_command_ack(_ack(CAM1, MAV_RESULT_UNSUPPORTED))
    assert sink.requests()[-1] == (CAM1, INFO, 0, 1)


def test_retry_exhaustion_logs_once_and_waits_for_lost_then_seen(rig, clock, caplog) -> None:
    streams, sink = rig
    caplog.set_level(logging.WARNING, logger="gcs.backend.video_streams")
    streams.on_heartbeat(_hb(CAM1))
    for _ in range(CAMERA_REQUEST_MAX_SENDS + 2):
        clock.now += CAMERA_REQUEST_TIMEOUT_S
        streams.on_heartbeat(_hb(CAM1))  # still alive: stays failed
        streams.tick()
    assert len(sink.sent) == CAMERA_REQUEST_MAX_SENDS
    assert sum("discovery failed" in r.getMessage() for r in caplog.records) == 1
    clock.now += CAMERA_LOST_AFTER_S
    streams.tick()
    streams.on_heartbeat(_hb(CAM1))
    assert sink.requests()[-1] == (CAM1, INFO, 0, 0)


def test_two_cameras_same_stream_id_one_lost_other_stays(rig, clock) -> None:
    streams, _ = rig
    _discover(streams, CAM1, "rtsp://192.168.144.10:8554/front")
    _discover(streams, CAM2, "rtsp://192.168.144.10:8554/down")
    assert [s["key"] for s in streams.snapshot()] == [f"{CAM1}:1", f"{CAM2}:1"]
    clock.now += CAMERA_LOST_AFTER_S - 1.0
    streams.on_heartbeat(_hb(CAM1))  # CAM2 stays silent
    clock.now += 1.0
    streams.tick()
    assert [s["key"] for s in streams.snapshot()] == [f"{CAM1}:1"]


def test_companion_restart_starts_a_new_generation(rig, clock) -> None:
    streams, sink = rig
    _discover(streams, CAM1, "rtsp://192.168.144.10:8554/old")
    clock.now += CAMERA_LOST_AFTER_S
    # The companion came back before any tick noticed the loss: the
    # heartbeat itself sees the camera as lost and drops the old streams.
    streams.on_heartbeat(_hb(CAM1))
    assert streams.snapshot() == []
    assert sink.requests()[-1] == (CAM1, INFO, 0, 0)
    streams.on_camera_information(_info(CAM1))
    streams.on_video_stream_information(_stream(CAM1, 1, 1, "rtsp://192.168.144.10:8554/new"))
    assert [s["whep_url"] for s in streams.snapshot()] == [
        "http://192.168.144.10:8889/new/whep"]


def test_rejected_uri_is_logged_and_not_shown(rig, caplog) -> None:
    streams, _ = rig
    caplog.set_level(logging.WARNING, logger="gcs.backend.video_streams")
    _discover(streams, CAM1, "rtsp://admin:pw@192.168.144.10:8554/main")
    assert streams.snapshot() == []
    assert any("URI rejected (userinfo)" in r.getMessage() for r in caplog.records)


def test_answers_from_unseen_camera_are_ignored(rig) -> None:
    streams, sink = rig
    streams.on_camera_information(_info(CAM1))
    streams.on_video_stream_information(_stream(CAM1, 1, 1, "rtsp://192.168.144.10/a"))
    assert sink.sent == [] and streams.snapshot() == []


def test_entry_snapshot_carries_video_streams(monkeypatch, clock) -> None:
    from tests.gcs.test_gimbal_snapshot import FakeVehicle

    monkeypatch.setattr(video_stream_policy.settings_store, "get", GcsSettings)
    vehicle = FakeVehicle()
    vehicle.sent = []
    vehicle.send_mavlink_message = vehicle.sent.append
    entry = VehicleEntry(SYS, vehicle, "uav")
    assert entry.snapshot()["video_streams"] == []
    vehicle.emit("HEARTBEAT", _hb(CAM1))
    vehicle.emit("CAMERA_INFORMATION", _info(CAM1))
    vehicle.emit("VIDEO_STREAM_INFORMATION",
                 _stream(CAM1, 1, 1, "rtsp://127.0.0.1:20550/uav4"))  # sim mode default
    assert [s["whep_url"] for s in entry.snapshot()["video_streams"]] == [
        "http://127.0.0.1:8889/uav4/whep"]
    assert len(vehicle.sent) == 2


def test_video_stream_change_is_broadcast() -> None:
    stream = {"key": "100:1"}
    assert _has_changed({"video_streams": []}, {"video_streams": [stream]})
    assert not _has_changed({"video_streams": [stream]}, {"video_streams": [dict(stream)]})
