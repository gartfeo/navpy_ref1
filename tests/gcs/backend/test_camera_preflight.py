"""Preflight camera fields across the fleet: wait clock, expected flag, duplicates.

docs/design/gcs_preflight_camera_row.md, Backend.
"""
from __future__ import annotations

from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM, MAV_STATE_ACTIVE, MAV_STATE_CRITICAL,
)

from gcs.backend import sim_video_companion, video_stream_policy
from gcs.backend.camera_preflight import CAMERA_ADVERTISE_GRACE_S, CameraPreflight
from gcs.backend.routes.control_readiness import _check_launch_readiness
from gcs.backend.settings_model import GcsSettings
from gcs.backend.telemetry_loop import _has_changed
from gcs.backend.vehicle_entry import VehicleEntry
from gcs.backend.vehicle_manager import VehicleManager
from gcs.backend.video_streams import VehicleVideoStreams
from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_LOST_AFTER_S, CAMERA_REQUEST_MAX_SENDS, CAMERA_REQUEST_TIMEOUT_S,
)
from tests.gcs.backend.test_video_camera_record import BOOT_A, URI, _hb as _navpy_hb, _info
from tests.gcs.backend.test_video_streams import (  # noqa: F401  (clock: fixture)
    CAM1, POLICY, SYS, Sink, _hb, _msg, _stream, clock,
)
from tests.gcs.test_launch_gating import _mgr_for

URL_A = "http://192.168.144.10:8889/uav1/whep"
URL_B = "http://192.168.144.11:8889/uav2/whep"


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def _snap(sys_id: int, status: str = "ok", urls: tuple[str, ...] = ()) -> dict:
    return {
        "sys_id": sys_id,
        "companion_status": status,
        "video_streams": [
            {"key": f"{CAM1}:{i}", "whep_url": url} for i, url in enumerate(urls, 1)
        ],
    }


def _wait_over(preflight: CameraPreflight, status: str = "ok", sys_id: int = 1) -> bool:
    snap = _snap(sys_id, status)
    preflight.annotate([snap])
    return snap["camera_wait_over"]


# --- camera_wait_over -------------------------------------------------------

def test_wait_is_over_at_the_grace_after_the_companion_came_up() -> None:
    clock = Clock()
    preflight = CameraPreflight(clock, expected=lambda sys_id: False)
    assert _wait_over(preflight, "down") is False
    clock.now += 100.0
    assert _wait_over(preflight) is False  # came up now
    clock.now += CAMERA_ADVERTISE_GRACE_S - 1e-3
    assert _wait_over(preflight) is False
    clock.now += 1e-3
    assert _wait_over(preflight) is True


def test_checking_keeps_the_clock_and_down_resets_it() -> None:
    clock = Clock()
    preflight = CameraPreflight(clock, expected=lambda sys_id: False)
    _wait_over(preflight)
    clock.now += CAMERA_ADVERTISE_GRACE_S / 2
    assert _wait_over(preflight, "checking") is False
    clock.now += CAMERA_ADVERTISE_GRACE_S / 2
    assert _wait_over(preflight, "checking") is True
    assert _wait_over(preflight, "down") is False
    clock.now += 1.0
    assert _wait_over(preflight) is False
    clock.now += CAMERA_ADVERTISE_GRACE_S
    assert _wait_over(preflight) is True


def test_vehicle_leaving_the_fleet_resets_its_clock() -> None:
    clock = Clock()
    preflight = CameraPreflight(clock, expected=lambda sys_id: False)
    _wait_over(preflight, sys_id=1)
    clock.now += CAMERA_ADVERTISE_GRACE_S
    assert _wait_over(preflight, sys_id=1) is True
    preflight.annotate([_snap(2)])  # vehicle 1 removed
    assert _wait_over(preflight, sys_id=1) is False


# --- video_stream_expected --------------------------------------------------

def test_expected_flag_is_per_sys_id() -> None:
    preflight = CameraPreflight(Clock(), expected=lambda sys_id: sys_id == 2)
    snaps = [_snap(1), _snap(2)]
    preflight.annotate(snaps)
    assert [s["video_stream_expected"] for s in snaps] == [False, True]


def test_sim_start_records_whether_the_companion_got_the_stream(monkeypatch) -> None:
    monkeypatch.setattr(sim_video_companion, "_expected_by_sys_id", {})
    monkeypatch.setattr(sim_video_companion, "sim_video_env",
                        lambda settings, sys_id: {"URI": "x"} if sys_id == 7 else {})
    assert sim_video_companion.video_stream_expected(7) is False  # never started
    sim_video_companion.sim_video_start_kwargs(None, 7, subprocess_manager=True)
    sim_video_companion.sim_video_start_kwargs(None, 8, subprocess_manager=True)
    assert sim_video_companion.video_stream_expected(7) is True
    assert sim_video_companion.video_stream_expected(8) is False
    # A lockstep start never gets the stream, whatever the setting.
    sim_video_companion.sim_video_start_kwargs(None, 7, subprocess_manager=False)
    assert sim_video_companion.video_stream_expected(7) is False


# --- video_stream_duplicates ------------------------------------------------

def test_duplicates_name_the_other_uavs() -> None:
    preflight = CameraPreflight(Clock(), expected=lambda sys_id: False)
    snaps = [_snap(1, urls=(URL_A,)), _snap(2, urls=(URL_B, URL_A)), _snap(3, urls=(URL_A,)),
             _snap(4, urls=(URL_B,))]
    preflight.annotate(snaps)
    assert [s["video_stream_duplicates"] for s in snaps] == [
        [{"key": f"{CAM1}:1", "sys_ids": [2, 3]}],
        [{"key": f"{CAM1}:1", "sys_ids": [4]}, {"key": f"{CAM1}:2", "sys_ids": [1, 3]}],
        [{"key": f"{CAM1}:1", "sys_ids": [1, 2]}],
        [{"key": f"{CAM1}:1", "sys_ids": [2]}],
    ]


def test_one_uav_announcing_a_stream_twice_is_not_a_duplicate() -> None:
    preflight = CameraPreflight(Clock(), expected=lambda sys_id: False)
    snaps = [_snap(1, urls=(URL_A, URL_A)), _snap(2, urls=(URL_B,))]
    preflight.annotate(snaps)
    assert [s["video_stream_duplicates"] for s in snaps] == [[], []]


def test_lost_cameras_streams_leave_the_duplicate_check(clock) -> None:
    uri = "rtsp://192.168.144.10:8554/shared"
    fleet = {sys_id: VehicleVideoStreams(sys_id, Sink(), policy=lambda: POLICY)
             for sys_id in (1, 2)}
    for sys_id, vehicle in fleet.items():
        vehicle.on_heartbeat(_hb(CAM1, src_system=sys_id))
        vehicle.on_camera_information(
            _msg(CAM1, sys_id, flags=CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM))
        vehicle.on_video_stream_information(
            _msg(CAM1, sys_id, stream_id=1, count=1, uri=uri, name="n", encoding=0))
    preflight = CameraPreflight(Clock(), expected=lambda sys_id: False)

    def duplicates() -> list:
        snaps = [{"sys_id": s, "companion_status": "ok", "video_streams": v.snapshot()}
                 for s, v in fleet.items()]
        preflight.annotate(snaps)
        return [s["video_stream_duplicates"] for s in snaps]

    assert duplicates() == [[{"key": f"{CAM1}:1", "sys_ids": [2]}],
                            [{"key": f"{CAM1}:1", "sys_ids": [1]}]]
    clock.now += CAMERA_LOST_AFTER_S - 1.0
    fleet[1].on_heartbeat(_hb(CAM1, src_system=1))  # vehicle 2's camera stays silent
    clock.now += 1.0
    for vehicle in fleet.values():
        vehicle.tick()
    assert duplicates() == [[], []]


# --- assembly and broadcast -------------------------------------------------

def test_manager_snapshots_carry_the_preflight_fields(monkeypatch) -> None:
    monkeypatch.setattr(sim_video_companion, "_expected_by_sys_id", {})
    manager = VehicleManager()
    manager._vehicles = {
        sys_id: SimpleNamespace(
            sys_id=sys_id, snapshot=lambda sys_id=sys_id: _snap(sys_id, urls=(URL_A,)))
        for sys_id in (1, 2)
    }
    monkeypatch.setattr(manager, "companion_status", lambda sys_id: "ok")
    snaps = manager.get_all_snapshots()
    assert [(s["camera_wait_over"], s["video_stream_expected"]) for s in snaps] == [
        (False, False), (False, False)]
    assert [s["video_stream_duplicates"] for s in snaps] == [
        [{"key": f"{CAM1}:1", "sys_ids": [2]}], [{"key": f"{CAM1}:1", "sys_ids": [1]}]]


@pytest.mark.parametrize("key, before, after", [
    ("cameras", [], [{"component": CAM1, "lost": False}]),
    ("cameras", [{"component": CAM1, "lost": False}], [{"component": CAM1, "lost": True}]),
    ("camera_wait_over", False, True),
    ("video_stream_expected", False, True),
    ("video_stream_duplicates", [], [{"key": f"{CAM1}:1", "sys_ids": [2]}]),
])
def test_preflight_camera_fields_are_broadcast_on_change(key, before, after) -> None:
    assert _has_changed({key: before}, {key: after})
    assert not _has_changed({key: after}, {key: after})


# --- the backend launch gate ------------------------------------------------

def _camera_in(state: str, vehicle, entry, clock) -> dict:
    """Drive CAM1 into *state* through the entry's real handlers; its snapshot."""
    status = MAV_STATE_CRITICAL if state == "noFrames" else MAV_STATE_ACTIVE
    vehicle.emit("HEARTBEAT", _navpy_hb(CAM1, BOOT_A, status))
    if state == "discoveryFailed":
        for _ in range(CAMERA_REQUEST_MAX_SENDS):
            clock.now += CAMERA_REQUEST_TIMEOUT_S
            vehicle.emit("HEARTBEAT", _navpy_hb(CAM1, BOOT_A))
            entry.snapshot()
    else:
        vehicle.emit("CAMERA_INFORMATION", _info(CAM1))
        uri = "rtsp://8.8.8.8:8554/front" if state == "uriRejected" else URI
        vehicle.emit("VIDEO_STREAM_INFORMATION", _stream(CAM1, 1, 1, uri))
    if state == "lost":
        clock.now += CAMERA_LOST_AFTER_S
    (camera,) = entry.snapshot()["cameras"]
    return camera


@pytest.mark.parametrize("state, shown", [
    ("ready", lambda c: c["status"] == "active" and c["discovery"] == "done"),
    ("noFrames", lambda c: c["status"] == "critical"),
    ("lost", lambda c: c["lost"]),
    ("uriRejected", lambda c: c["rejected"]),
    ("discoveryFailed", lambda c: c["discovery"] == "failed"),
])
def test_launch_issues_ignore_every_camera_state(monkeypatch, clock, state, shown) -> None:
    from tests.gcs.test_gimbal_snapshot import FakeVehicle

    monkeypatch.setattr(video_stream_policy.settings_store, "get", GcsSettings)
    vehicle = FakeVehicle()
    vehicle.send_mavlink_message = lambda msg: None
    vehicle.rc3_raw = 2000  # one blocker, so a changed list shows either way
    entry = VehicleEntry(SYS, vehicle, "uav")
    manager = _mgr_for({SYS: entry})
    before = _check_launch_readiness([SYS], vehicle_mgr=manager)
    assert f"Vehicle {SYS}: throttle not zero" in before
    assert shown(_camera_in(state, vehicle, entry, clock))
    assert _check_launch_readiness([SYS], vehicle_mgr=manager) == before
