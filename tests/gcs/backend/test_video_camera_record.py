"""Camera records: boot rules, Lost state and the preflight ``cameras`` list.

docs/design/gcs_preflight_camera_row.md, Backend.
"""
from __future__ import annotations

import logging

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav
from pymavlink.dialects.v20.ardupilotmega import (
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM,
    MAV_COMP_ID_MISSIONPLANNER,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_STANDBY,
    MAV_TYPE_CAMERA,
    MAVLINK_MSG_ID_CAMERA_INFORMATION as INFO,
    MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION as STREAMS,
)

from gcs.backend import video_stream_policy
from gcs.backend.settings_model import GcsSettings
from gcs.backend.vehicle_entry import VehicleEntry
from navpy.core.ports.camera_health import (
    NAVPY_CAMERA_VENDOR, NAVPY_SIM_CAMERA_VENDOR, CameraHealth,
)
from navpy.core.ports.video_stream import VideoStreamSpec
from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_LOST_AFTER_S, CAMERA_REQUEST_MAX_SENDS, CAMERA_REQUEST_TIMEOUT_S,
)
from navpy.plugins.vehicle.ardupilot.camera_stream_messages import (
    CameraStreamResponder, StreamingCamera, camera_heartbeat,
)
from tests.gcs.backend.test_video_streams import (  # noqa: F401  (clock, rig: fixtures)
    CAM1, CAM2, GCS_SYS, SYS, _msg, _stream, clock, rig,
)

URI = "rtsp://192.168.144.10:8554/front"
WHEP = "http://192.168.144.10:8889/front/whep"
BOOT_A, BOOT_B = 0x1234, 0xFFFFFFFF


def _hb(component: int, boot_id: int = 0, status: int = MAV_STATE_ACTIVE):
    return _msg(component, type=MAV_TYPE_CAMERA, custom_mode=boot_id, system_status=status)


def _info(component: int, vendor: str = NAVPY_CAMERA_VENDOR,
          flags: int = CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM):
    # pymavlink gives a uint8_t[32] field as a list of ints.
    return _msg(component, flags=flags, vendor_name=list(vendor.encode().ljust(32, b"\0")))


def _navpy_camera(streams, boot_id: int = BOOT_A, uri: str = URI) -> None:
    """CAM1 answers as NavPy's camera with one stream."""
    streams.on_heartbeat(_hb(CAM1, boot_id))
    streams.on_camera_information(_info(CAM1))
    streams.on_video_stream_information(_stream(CAM1, 1, 1, uri))


def _camera(streams, component: int = CAM1) -> dict:
    (camera,) = [c for c in streams.cameras() if c["component"] == component]
    return camera


def _lose(streams, clock) -> None:
    clock.now += CAMERA_LOST_AFTER_S
    streams.tick()


# --- the cameras list -------------------------------------------------------

def test_cameras_list_follows_discovery(rig) -> None:
    streams, _ = rig
    assert streams.cameras() == []
    streams.on_heartbeat(_hb(CAM1))
    assert streams.cameras() == [{
        "component": CAM1, "lost": False, "vendor": None, "status": "active",
        "discovery": "running", "has_video": None, "rejected": [],
    }]
    streams.on_camera_information(_info(CAM1))
    assert _camera(streams)["vendor"] == "navpy"
    assert _camera(streams)["has_video"] is True
    assert _camera(streams)["discovery"] == "running"
    streams.on_video_stream_information(_stream(CAM1, 1, 1, URI))
    assert _camera(streams)["discovery"] == "done"


@pytest.mark.parametrize("vendor, kind", [
    (NAVPY_CAMERA_VENDOR, "navpy"),
    (NAVPY_SIM_CAMERA_VENDOR, "navpy_sim"),
    ("SIYI", "other"),
    ("navpy", "other"),  # exact match only
    ("", "other"),  # older NavPy sends no vendor
])
def test_vendor_kind(rig, vendor, kind) -> None:
    streams, _ = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_camera_information(_info(CAM1, vendor))
    assert _camera(streams)["vendor"] == kind


def test_camera_without_video_reads_done_with_no_video(rig) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_camera_information(_info(CAM1, flags=0))
    camera = _camera(streams)
    assert (camera["has_video"], camera["discovery"]) == (False, "done")
    assert len(sink.sent) == 1  # no stream request


@pytest.mark.parametrize("status, name", [
    (MAV_STATE_ACTIVE, "active"),
    (MAV_STATE_CRITICAL, "critical"),
    (MAV_STATE_STANDBY, "other"),
])
def test_status_is_the_last_heartbeats(rig, status, name) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_heartbeat(_hb(CAM1, status=status))
    assert _camera(streams)["status"] == name
    assert len(sink.sent) == 1  # a status change starts nothing new


def test_heartbeat_without_status_reads_other(rig) -> None:
    streams, _ = rig
    streams.on_heartbeat(_msg(CAM1, type=MAV_TYPE_CAMERA))
    assert _camera(streams)["status"] == "other"


def test_rejected_stream_is_listed_with_its_reason(rig) -> None:
    streams, _ = rig
    _navpy_camera(streams, uri="rtsp://admin:pw@192.168.144.10:8554/main")
    camera = _camera(streams)
    assert camera["rejected"] == [{"stream_id": 1, "reason": "userinfo"}]
    assert camera["discovery"] == "done"
    assert streams.snapshot() == []


def test_failed_discovery_reads_failed(rig, clock) -> None:
    streams, _ = rig
    streams.on_heartbeat(_hb(CAM1))
    for _ in range(CAMERA_REQUEST_MAX_SENDS):
        clock.now += CAMERA_REQUEST_TIMEOUT_S
        streams.on_heartbeat(_hb(CAM1))
        streams.tick()
    assert _camera(streams)["discovery"] == "failed"


# --- Lost -------------------------------------------------------------------

def test_lost_camera_stays_listed_and_its_streams_leave(rig, clock, caplog) -> None:
    streams, _ = rig
    caplog.set_level(logging.INFO, logger="gcs.backend.video_streams")
    _navpy_camera(streams)
    assert [s["whep_url"] for s in streams.snapshot()] == [WHEP]
    clock.now += CAMERA_LOST_AFTER_S - 1e-3
    streams.tick()
    assert _camera(streams)["lost"] is False
    clock.now += 1e-3
    streams.tick()
    streams.tick()
    camera = _camera(streams)
    assert (camera["lost"], camera["vendor"], camera["has_video"]) == (True, "navpy", True)
    assert streams.snapshot() == []
    assert sum("lost" in r.getMessage() for r in caplog.records) == 1


def test_one_of_two_cameras_lost_stays_lost(rig, clock) -> None:
    streams, _ = rig
    streams.on_heartbeat(_hb(CAM1))
    streams.on_heartbeat(_hb(CAM2))
    clock.now += CAMERA_LOST_AFTER_S - 1.0
    streams.on_heartbeat(_hb(CAM1))  # CAM2 stays silent
    clock.now += 1.0
    streams.tick()
    assert [(c["component"], c["lost"]) for c in streams.cameras()] == [
        (CAM1, False), (CAM2, True)]


@pytest.mark.parametrize("answered_info", [False, True])
def test_lost_camera_takes_no_replies_and_sends_no_requests(rig, clock, answered_info) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1))  # the information request is outstanding
    if answered_info:
        streams.on_camera_information(_info(CAM1))  # now the stream request
    _lose(streams, clock)
    before = _camera(streams)
    sink.sent.clear()
    streams.on_command_ack(_msg(CAM1, command=mav.MAV_CMD_REQUEST_MESSAGE, result=0,
                                target_system=GCS_SYS))
    streams.on_camera_information(_info(CAM1, "SIYI"))
    streams.on_video_stream_information(
        _stream(CAM1, 1, 1, "rtsp://admin:pw@192.168.144.10:8554/main"))
    clock.now += CAMERA_REQUEST_TIMEOUT_S
    streams.tick()
    assert sink.sent == []
    assert _camera(streams) == before and streams.snapshot() == []


def test_same_boot_id_after_a_dropout_rediscovers(rig, clock) -> None:
    streams, sink = rig
    _navpy_camera(streams)
    _lose(streams, clock)
    streams.on_heartbeat(_hb(CAM1, BOOT_A))  # the radio is back, NavPy never stopped
    camera = _camera(streams)
    assert (camera["lost"], camera["discovery"], camera["vendor"]) == (False, "running", None)
    assert sink.requests()[-1] == (CAM1, INFO, 0, 0)


# --- boot ids ---------------------------------------------------------------

def test_navpy_restart_starts_a_new_generation_at_once(rig, clock) -> None:
    streams, sink = rig
    _navpy_camera(streams)
    clock.now += 1.0  # well inside CAMERA_LOST_AFTER_S
    streams.on_heartbeat(_hb(CAM1, BOOT_B))
    assert streams.snapshot() == []
    assert _camera(streams)["discovery"] == "running"
    assert sink.requests()[-1] == (CAM1, INFO, 0, 0)


def test_late_heartbeat_of_the_old_process_is_dropped_whole(rig, clock) -> None:
    streams, sink = rig
    _navpy_camera(streams, BOOT_A)
    streams.on_heartbeat(_hb(CAM1, BOOT_B))
    streams.on_camera_information(_info(CAM1))
    streams.on_video_stream_information(_stream(CAM1, 1, 1, URI))
    sent = len(sink.sent)
    clock.now += CAMERA_LOST_AFTER_S - 0.5
    streams.on_heartbeat(_hb(CAM1, BOOT_A, MAV_STATE_CRITICAL))
    camera = _camera(streams)
    assert (camera["status"], camera["discovery"]) == ("active", "done")
    assert len(sink.sent) == sent
    assert [s["whep_url"] for s in streams.snapshot()] == [WHEP]
    clock.now += 0.5  # the dropped heartbeat kept nothing alive
    streams.tick()
    assert _camera(streams)["lost"] is True


def test_camera_not_navpys_ignores_boot_ids(rig) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1, BOOT_A))
    streams.on_camera_information(_info(CAM1, "SIYI"))
    streams.on_video_stream_information(_stream(CAM1, 1, 1, URI))
    sent = len(sink.sent)
    streams.on_heartbeat(_hb(CAM1, BOOT_B))
    streams.on_heartbeat(_hb(CAM1, BOOT_A))
    assert len(sink.sent) == sent
    assert [s["whep_url"] for s in streams.snapshot()] == [WHEP]


def test_camera_qualifying_after_a_restart_catches_up(rig) -> None:
    streams, sink = rig
    streams.on_heartbeat(_hb(CAM1, BOOT_A))
    streams.on_heartbeat(_hb(CAM1, BOOT_B))  # restarted before it answered
    assert sink.requests() == [(CAM1, INFO, 0, 0)]
    streams.on_camera_information(_info(CAM1))  # the new process answers
    streams.on_heartbeat(_hb(CAM1, BOOT_B))
    assert sink.requests() == [(CAM1, INFO, 0, 0), (CAM1, STREAMS, 0, 0), (CAM1, INFO, 0, 0)]


def test_boot_id_zero_keeps_the_lost_rule(rig, clock) -> None:
    streams, sink = rig
    _navpy_camera(streams, boot_id=0)
    sent = len(sink.sent)
    clock.now += CAMERA_LOST_AFTER_S - 1e-3
    streams.on_heartbeat(_hb(CAM1, 0))
    assert len(sink.sent) == sent
    clock.now += CAMERA_LOST_AFTER_S
    streams.on_heartbeat(_hb(CAM1, 0))
    assert sink.requests()[-1] == (CAM1, INFO, 0, 0)


def test_answer_with_another_vendor_ends_navpy_qualification(rig, clock) -> None:
    streams, sink = rig
    _navpy_camera(streams, BOOT_A)
    _lose(streams, clock)
    streams.on_heartbeat(_hb(CAM1, BOOT_A))
    streams.on_camera_information(_info(CAM1, "SIYI", flags=0))
    sent = len(sink.sent)
    streams.on_heartbeat(_hb(CAM1, BOOT_B))
    assert len(sink.sent) == sent
    assert _camera(streams)["vendor"] == "other"


# --- NavPy's camera on the wire ---------------------------------------------

def _wire(message, system: int, component: int):
    sender = mav.MAVLink(None, srcSystem=system, srcComponent=component)
    return mav.MAVLink(None).decode(bytearray(message.pack(sender)))


@pytest.mark.parametrize("vendor, kind", [
    (NAVPY_CAMERA_VENDOR, "navpy"), (NAVPY_SIM_CAMERA_VENDOR, "navpy_sim"),
])
def test_navpy_camera_over_the_wire(rig, vendor, kind) -> None:
    streams, sink = rig
    spec = VideoStreamSpec(1, "front", URI, "H264", 1280, 720, 30.0)
    cameras = [
        StreamingCamera(1, "front", 1280, 720, (spec,), CameraHealth(vendor, lambda: True)),
        StreamingCamera(2, "down", 640, 480, (), CameraHealth(vendor, lambda: False)),
    ]
    responder = CameraStreamResponder(SYS, cameras)
    handlers = {
        "COMMAND_ACK": streams.on_command_ack,
        "CAMERA_INFORMATION": streams.on_camera_information,
        "VIDEO_STREAM_INFORMATION": streams.on_video_stream_information,
    }
    for camera in cameras:
        heartbeat = camera_heartbeat(camera.camera_health.ready(), BOOT_B)
        streams.on_heartbeat(_wire(heartbeat, SYS, camera.component_id))
    answered = 0
    while answered < len(sink.sent):
        request = _wire(sink.sent[answered], GCS_SYS, MAV_COMP_ID_MISSIONPLANNER)
        answered += 1
        for reply in responder.answer(request):
            message = _wire(reply, SYS, request.target_component)
            handlers[message.get_type()](message)
    assert streams.cameras() == [
        {"component": CAM1, "lost": False, "vendor": kind, "status": "active",
         "discovery": "done", "has_video": True, "rejected": []},
        {"component": CAM2, "lost": False, "vendor": kind, "status": "critical",
         "discovery": "done", "has_video": False, "rejected": []},
    ]
    assert [s["whep_url"] for s in streams.snapshot()] == [WHEP]


def test_entry_snapshot_carries_cameras(monkeypatch, clock) -> None:
    from tests.gcs.test_gimbal_snapshot import FakeVehicle

    monkeypatch.setattr(video_stream_policy.settings_store, "get", GcsSettings)
    vehicle = FakeVehicle()
    vehicle.sent = []
    vehicle.send_mavlink_message = vehicle.sent.append
    entry = VehicleEntry(SYS, vehicle, "uav")
    assert entry.snapshot()["cameras"] == []
    vehicle.emit("HEARTBEAT", _hb(CAM1, BOOT_A, MAV_STATE_CRITICAL))
    assert [(c["component"], c["status"]) for c in entry.snapshot()["cameras"]] == [
        (CAM1, "critical")]
