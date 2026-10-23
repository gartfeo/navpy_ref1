"""GCS mission transfers under one request budget (``docs/design/radio_budget.md``,
todo 105): 12 sends per step, a deadline that also bounds the mission-lock wait,
and an unconfirmed upload that counts only on an exact readback."""
from __future__ import annotations

import asyncio
import copy
import threading
import time
from dataclasses import replace
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_FRAME_GLOBAL,
    MAV_FRAME_MISSION,
    MAV_MISSION_ACCEPTED,
)

from gcs.backend.exchange_budget import GCS_EXCHANGE_ATTEMPTS, gcs_policy, held_within
from gcs.backend.planner.fence_builder import upload_fence_with_retry
from gcs.backend.planner.waypoint_builder import upload_mission_with_retry
from gcs.backend.routes.missions import MISSION_UPLOAD_REQUEST_S
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from tests.modules.vehicle.test_mission_transfer import TARGET, Autopilot, Receiver, _ack, _request

TRACK = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}]
TRIANGLE = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.01, "lon": 34.0}, {"lat": 32.0, "lon": 34.01}]


def test_a_gcs_download_sends_twelve_requests_while_the_autopilot_talks():
    autopilot = Autopilot(3, count_replies=False)
    link = HeartbeatState(TARGET, 5.0)
    link.observe_frame()
    assert autopilot.downloader(link).download(policy=gcs_policy(30.0), ack_timeout=0.001) == 0
    assert autopilot.connection.mav.mission_request_list_send.call_count == GCS_EXCHANGE_ATTEMPTS == 12


class StoredVehicle:
    """Stores every upload; the first ``lost_acks`` uploads report no final ACK."""

    def __init__(self, lost_acks: int, changed_field: str | None = None,
                 frameless_as_global: bool = False, changed_jump: bool = False) -> None:
        self.target_system = 1
        self.mission_lock = threading.Lock()
        self.link_ok = True
        self.lost_acks = lost_acks
        self.changed_field = changed_field
        self.frameless_as_global = frameless_as_global
        self.changed_jump = changed_jump
        self.uploads = 0
        self.policies = []
        self._loader = None
        self._stored = []

    def clear_mission(self) -> None:
        self._stored = []

    def load_mission_items(self, loader) -> None:
        self._loader = loader

    def upload_mission(self, on_wp_sent=None, policy=None) -> bool:
        self.uploads += 1
        self.policies.append(policy)
        self._stored = [copy.copy(self._loader.wp(i)) for i in range(self._loader.count())]
        if self.frameless_as_global:  # as ArduPilot returns a command without a location
            for item in self._stored:
                if item.frame == MAV_FRAME_MISSION:
                    item.frame = MAV_FRAME_GLOBAL
        if self.changed_field is not None:  # the autopilot normalized one field
            item = self._stored[self._jump() if self.changed_jump else 2]
            setattr(item, self.changed_field, getattr(item, self.changed_field) + 1)
        if self.uploads <= self.lost_acks:
            return False
        return True

    def _jump(self) -> int:
        return next(i for i in range(self._loader.count())
                    if self._loader.wp(i).frame == MAV_FRAME_MISSION)

    def download_mission(self, on_progress=None, policy=None) -> int:
        self.policies.append(policy)
        return len(self._stored)

    def get_mission_item(self, index):
        return self._stored[index] if index < len(self._stored) else None


def test_an_upload_whose_final_ack_was_lost_counts_on_an_exact_readback():
    vehicle = StoredVehicle(lost_acks=1)
    policy = gcs_policy(60.0)
    result = upload_mission_with_retry(vehicle, TRACK, 100.0, policy=policy)
    assert result.success and result.attempts == 1
    assert vehicle.uploads == 1
    assert vehicle.policies == [policy, policy]  # the upload and its readback


def test_a_jump_read_back_with_frame_zero_still_matches():
    vehicle = StoredVehicle(lost_acks=1, frameless_as_global=True)
    hub = {"lat": 32.002, "lon": 34.002}
    result = upload_mission_with_retry(vehicle, TRACK, 100.0, default_delivery_hub=hub,
                                       policy=gcs_policy(60.0))
    assert any(vehicle._loader.wp(i).frame == MAV_FRAME_MISSION
               for i in range(vehicle._loader.count()))  # the skip DO_JUMP
    assert result.success and vehicle.uploads == 1


@pytest.mark.parametrize("changed_field", ["command", "param1", "param2"])
def test_a_jump_read_back_with_frame_zero_and_another_change_uploads_again(changed_field):
    vehicle = StoredVehicle(lost_acks=1, changed_field=changed_field,
                            frameless_as_global=True, changed_jump=True)
    upload_mission_with_retry(vehicle, TRACK, 100.0, default_delivery_hub={"lat": 32.002, "lon": 34.002},
                              policy=gcs_policy(60.0))
    assert vehicle.uploads == 2


@pytest.mark.parametrize("changed_field", ["param2", "frame"])
def test_a_readback_that_differs_in_any_stored_field_uploads_again(changed_field):
    vehicle = StoredVehicle(lost_acks=1, changed_field=changed_field)
    result = upload_mission_with_retry(vehicle, TRACK, 100.0, policy=gcs_policy(60.0))
    assert vehicle.uploads == 2
    assert result.attempts == 2


def test_the_deadline_bounds_the_mission_lock_wait():
    vehicle = StoredVehicle(lost_acks=0)
    vehicle.mission_lock.acquire()  # a probe still holds the vehicle
    try:
        started = time.monotonic()
        result = upload_mission_with_retry(vehicle, TRACK, 100.0, policy=gcs_policy(0.05))
    finally:
        vehicle.mission_lock.release()
    assert not result.success and vehicle.uploads == 0
    assert time.monotonic() - started < 1.0


def _client(entry):
    manager = MagicMock()
    manager.get_vehicle.return_value = entry
    return patch("gcs.backend.routes.missions.vehicle_mgr", manager)


def _entry(vehicle):
    entry = MagicMock(sys_id=1, vehicle=vehicle, mission_uploaded=False, cached_mission=None)
    return entry


def test_get_mission_downloads_within_its_timeout_and_is_busy_past_it():
    vehicle = MagicMock()
    vehicle.mission_lock = threading.Lock()
    vehicle.download_mission.return_value = 0
    with _client(_entry(vehicle)):
        from gcs.backend.main import app
        client = TestClient(app)
        before = time.monotonic()
        assert client.get("/api/vehicles/1/mission?timeout_s=30").status_code == 404
        policy = vehicle.download_mission.call_args.kwargs["policy"]
        assert policy.attempts == 12 and policy.stop_on_silence
        assert before + 30 <= policy.deadline_s <= time.monotonic() + 30

        vehicle.mission_lock.acquire()
        try:
            started = time.monotonic()
            response = client.get("/api/vehicles/1/mission?timeout_s=0.05")
        finally:
            vehicle.mission_lock.release()
        assert response.status_code == 503
        assert time.monotonic() - started < 1.0
        assert client.get("/api/vehicles/1/mission?timeout_s=0").status_code == 400


def test_the_upload_route_gives_every_attempt_one_request_deadline():
    from gcs.backend.planner.waypoint_builder import UploadResult
    vehicle = MagicMock()
    with _client(_entry(vehicle)), patch(
        "gcs.backend.routes.missions.upload_mission_with_retry",
        return_value=UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1),
    ) as upload:
        from gcs.backend.main import app
        before = time.monotonic()
        response = TestClient(app).post("/api/vehicles/upload", json={
            "assignments": [{"sys_id": 1, "zone_index": 0, "altitude_m": 100.0, "waypoints": TRACK}],
        })
    assert response.status_code == 200, response.text
    policy = upload.call_args.kwargs["policy"]
    assert policy.attempts == 12
    assert before + MISSION_UPLOAD_REQUEST_S <= policy.deadline_s <= time.monotonic() + MISSION_UPLOAD_REQUEST_S


def _spent():
    """A request policy whose deadline has passed."""
    policy = gcs_policy(60.0)
    return replace(policy, deadline_s=time.monotonic() - 1.0)


def test_past_the_deadline_a_free_lock_is_not_taken_and_nothing_is_cleared():
    vehicle = StoredVehicle(lost_acks=0)
    vehicle.clear_mission = MagicMock()
    with held_within(vehicle.mission_lock, _spent()) as held:
        assert not held
        assert not vehicle.mission_lock.locked()
    result = upload_mission_with_retry(vehicle, TRACK, 100.0, policy=_spent())
    assert not result.success
    vehicle.clear_mission.assert_not_called()
    assert vehicle.uploads == 0


class LateLock:
    """Comes free only once the waiter's whole timeout has passed."""

    def __init__(self) -> None:
        self.released = False

    def acquire(self, timeout: float = -1) -> bool:
        time.sleep(timeout + 0.01)
        return True

    def release(self) -> None:
        self.released = True


def test_a_lock_won_after_the_deadline_is_given_back_unused():
    lock = LateLock()
    with held_within(lock, gcs_policy(0.02)) as held:
        assert not held
    assert lock.released


def test_past_the_deadline_no_fence_is_cleared():
    vehicle = MagicMock(target_system=1, mission_lock=threading.Lock())
    result = upload_fence_with_retry(vehicle, TRIANGLE, policy=_spent())
    assert not result.success
    vehicle.clear_fence.assert_not_called()
    vehicle.upload_fence.assert_not_called()


def test_an_upload_past_its_deadline_starts_no_reconnect_rtl_write_or_fence():
    from gcs.backend.planner.waypoint_builder import UploadResult
    vehicle = MagicMock()
    manager = MagicMock()
    manager.get_vehicle.return_value = _entry(vehicle)
    lost = UploadResult(success=False, uploaded_count=0, expected_count=4, attempts=1,
                        error="link lost", link_lost=True)
    stored = UploadResult(success=True, uploaded_count=4, expected_count=4, attempts=1)
    fence = {"enabled": True, "action": 1, "type": 4, "vertices": TRIANGLE}
    body = {"assignments": [{"sys_id": 1, "zone_index": 0, "altitude_m": 100.0, "waypoints": TRACK}],
            "fence": fence}
    from gcs.backend.main import app
    for result in (lost, stored):
        vehicle.reset_mock()
        manager.reconnect_vehicle.reset_mock()
        with patch("gcs.backend.routes.missions.vehicle_mgr", manager), \
             patch("gcs.backend.routes.missions.gcs_policy", return_value=_spent()), \
             patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=result), \
             patch("gcs.backend.routes.missions.upload_fence_with_retry") as upload_fence:
            response = TestClient(app).post("/api/vehicles/upload", json=body)
        assert response.status_code == 200, response.text
        manager.reconnect_vehicle.assert_not_called()
        vehicle.set_parameter.assert_not_called()
        upload_fence.assert_not_called()
    assert response.json()["fence_ok"] is False


def test_a_gcs_upload_logs_every_send():
    receiver = Receiver(2)
    receiver.script = {(0, 1): [_request(1)], (1, 1): [_ack(MAV_MISSION_ACCEPTED)]}
    link = HeartbeatState(TARGET, 5.0)
    link.observe_frame()
    uploader = receiver.uploader(link=link)
    logger = uploader._logger_ref.value
    assert uploader.upload(policy=gcs_policy(30.0), ack_timeout=0.1, abandon_s=0.5)
    lines = [call.args[0] for call in logger.info.call_args_list if call.args[1:] == ("mission",)]
    assert lines == ["MISSION_COUNT send 1/12: MISSION_REQUEST_INT",
                     "MISSION_ITEM_INT 0/2 sent", "MISSION_ITEM_INT 1/2 sent"]


def test_a_reconnect_waits_for_the_link_only_until_the_deadline():
    from gcs.backend.routes.mission_upload_helpers import RECONNECT_WAIT_S, reconnect_within
    manager = MagicMock()
    reconnect_within(manager, 1, gcs_policy(0.5))
    assert 0.0 < manager.reconnect_vehicle.call_args.kwargs["timeout"] <= 0.5
    reconnect_within(manager, 1, gcs_policy(60.0))
    assert manager.reconnect_vehicle.call_args.kwargs["timeout"] == RECONNECT_WAIT_S
    manager.reset_mock()
    assert reconnect_within(manager, 1, _spent()) is None  # the deadline passed while it was queued
    manager.reconnect_vehicle.assert_not_called()


def test_the_route_does_not_reconnect_when_the_deadline_passes_during_its_broadcast():
    from gcs.backend.planner.waypoint_builder import UploadResult
    manager = MagicMock()
    manager.get_vehicle.return_value = _entry(MagicMock())
    lost = UploadResult(success=False, uploaded_count=0, expected_count=4, attempts=1,
                        error="link lost", link_lost=True)

    async def slow_broadcast(message):
        if message.get("stage") == "reconnecting":
            await asyncio.sleep(0.3)  # the budget runs out while the operator is told

    from gcs.backend.main import app
    with patch("gcs.backend.routes.missions.vehicle_mgr", manager), \
         patch("gcs.backend.routes.missions.gcs_policy", return_value=gcs_policy(0.2)), \
         patch("gcs.backend.routes.missions.ws_manager.broadcast", side_effect=slow_broadcast), \
         patch("gcs.backend.routes.missions.upload_mission_with_retry", return_value=lost):
        response = TestClient(app).post("/api/vehicles/upload", json={
            "assignments": [{"sys_id": 1, "zone_index": 0, "altitude_m": 100.0, "waypoints": TRACK}],
        })
    assert response.status_code == 200, response.text
    manager.reconnect_vehicle.assert_not_called()
