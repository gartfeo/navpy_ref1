"""GCS mission transfers under one request budget (``docs/design/radio_budget.md``,
todo 105): 12 sends per step, a deadline that also bounds the mission-lock wait,
and an unconfirmed upload that counts only on an exact readback."""
from __future__ import annotations

import copy
import threading
import time
from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from gcs.backend.exchange_budget import GCS_EXCHANGE_ATTEMPTS, gcs_policy
from gcs.backend.planner.waypoint_builder import upload_mission_with_retry
from gcs.backend.routes.missions import MISSION_UPLOAD_REQUEST_S
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from tests.modules.vehicle.test_mission_transfer import TARGET, Autopilot

TRACK = [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}]


def test_a_gcs_download_sends_twelve_requests_while_the_autopilot_talks():
    autopilot = Autopilot(3, count_replies=False)
    link = HeartbeatState(TARGET, 5.0)
    link.observe_frame()
    assert autopilot.downloader(link).download(policy=gcs_policy(30.0), ack_timeout=0.001) == 0
    assert autopilot.connection.mav.mission_request_list_send.call_count == GCS_EXCHANGE_ATTEMPTS == 12


class StoredVehicle:
    """Stores every upload; the first ``lost_acks`` uploads report no final ACK."""

    def __init__(self, lost_acks: int, changed_field: str | None = None) -> None:
        self.target_system = 1
        self.mission_lock = threading.Lock()
        self.link_ok = True
        self.lost_acks = lost_acks
        self.changed_field = changed_field
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
        if self.changed_field is not None:  # the autopilot normalized one field
            item = self._stored[2]
            setattr(item, self.changed_field, getattr(item, self.changed_field) + 1)
        if self.uploads <= self.lost_acks:
            return False
        return True

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


def test_a_readback_that_differs_in_any_stored_field_uploads_again():
    vehicle = StoredVehicle(lost_acks=1, changed_field="param2")
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
