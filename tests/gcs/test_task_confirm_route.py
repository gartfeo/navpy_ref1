"""Tests for task confirmation POST endpoint."""
from __future__ import annotations

from unittest.mock import MagicMock, patch, AsyncMock

import pytest
from fastapi.testclient import TestClient


@pytest.fixture
def mock_vehicle():
    vehicle = MagicMock()
    vehicle.send_mavlink_message = MagicMock()
    return vehicle


@pytest.fixture
def mock_entry(mock_vehicle):
    entry = MagicMock()
    entry.sys_id = 1
    entry.vehicle = mock_vehicle
    return entry


@pytest.fixture
def client(mock_entry):
    mock_mgr = MagicMock()
    mock_mgr.get_vehicle = MagicMock(return_value=mock_entry)

    with patch("gcs.backend.routes.task_confirm.vehicle_mgr", mock_mgr), \
         patch("gcs.backend.routes.task_confirm.ws_manager") as mock_ws:
        mock_ws.broadcast = AsyncMock()
        from gcs.backend.routes.task_confirm import router
        from fastapi import FastAPI
        app = FastAPI()
        app.include_router(router, prefix="/api/control")
        yield TestClient(app), mock_mgr, mock_ws


class TestTaskConfirmRoute:
    def test_approve_sends_mavlink(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 5,
            "is_confirmed": True,
            "round_uid": "legacy",
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "approved"
        assert data["task_id"] == 5
        # Transport is unacknowledged and the companion resolves by timeout:
        # the response is blindly retransmitted (duplicates are ignored on
        # the companion) and the payload reports delivery truthfully.
        assert data["delivery"] == "sent"
        assert mock_vehicle.send_mavlink_message.call_count == 3

    def test_deny_sends_mavlink(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 3,
            "is_confirmed": False,
            "round_uid": "legacy",
        })
        assert resp.status_code == 200
        assert resp.json()["status"] == "denied"
        assert mock_vehicle.send_mavlink_message.call_count == 3

    def test_unknown_vehicle_returns_404(self, client):
        tc, mock_mgr, mock_ws = client
        mock_mgr.get_vehicle.return_value = None
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 99,
            "task_id": 1,
            "is_confirmed": True,
        })
        assert resp.status_code == 404

    def test_broadcasts_response(self, client):
        tc, mock_mgr, mock_ws = client
        tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 5,
            "is_confirmed": True,
            "round_uid": "legacy",
        })
        mock_ws.broadcast.assert_called_once()
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["type"] == "task_confirm_response"
        assert payload["sys_id"] == 1
        assert payload["task_id"] == 5
        assert payload["is_confirmed"] is True


class TestDecisionRecordedOnListener:
    """The listener must remember the decision, not just forget the round.

    All 3 response copies can be lost; the companion then keeps resending its
    request and the listener answers those repeats with this record instead
    of re-opening a popup for a POI the operator already settled.
    """

    def test_approve_records_the_decision_against_its_round(self, client):
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": True, "round_uid": "424242:11",
        })
        assert resp.status_code == 200
        listener = mock_mgr.get_task_confirm_listener.return_value
        listener.mark_decided.assert_called_once_with(1, 5, True, "424242:11")
        listener.clear_handled.assert_not_called()

    def test_deny_records_the_decision_against_its_round(self, client):
        tc, mock_mgr, mock_ws = client
        tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": False, "action": "deny",
            "round_uid": "424242:11",
        })
        mock_mgr.get_task_confirm_listener.return_value.mark_decided \
            .assert_called_once_with(1, 5, False, "424242:11")

    def test_broadcast_carries_the_round_uid(self, client):
        """Every client needs the uid: the same POI can be asked again
        while this response is going out, and the new round's card must not
        be marked decided (and its countdown stopped) by this one."""
        tc, mock_mgr, mock_ws = client
        tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": True, "round_uid": "424242:11",
        })
        assert mock_ws.broadcast.call_args[0][0]["round_uid"] == "424242:11"

    def test_response_without_a_round_uid_is_still_accepted(self, client):
        """Older clients still send a response but cannot bind a decision."""
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": True,
        })
        assert resp.status_code == 200
        mock_mgr.get_task_confirm_listener.return_value.mark_decided \
            .assert_called_once_with(1, 5, True, None)
        assert mock_mgr.get_vehicle.return_value.vehicle \
            .send_mavlink_message.call_count == 3
        assert mock_ws.broadcast.call_args.args[0]["round_uid"] is None

    def test_rejected_action_records_nothing(self, client):
        tc, mock_mgr, mock_ws = client
        tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": True, "action": "nuke",
        })
        mock_mgr.get_task_confirm_listener.return_value.mark_decided.assert_not_called()


class TestTaskConfirmActions:
    """action field validation + WS round-trip."""

    def test_action_in_payload_and_response(self, client):
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": False, "action": "cancel",
            "round_uid": "legacy",
        })
        assert resp.status_code == 200
        assert resp.json()["action"] == "cancel"
        payload = mock_ws.broadcast.call_args[0][0]
        assert payload["action"] == "cancel"
        assert payload["is_confirmed"] is False

    def test_abort_is_not_a_valid_confirm_action(self, client, mock_vehicle):
        """A per-UAV abort is the destructive E-STOP command, not a recoverable
        confirm reject -- 'abort' must be rejected here."""
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": False, "action": "abort",
        })
        assert resp.status_code == 400
        mock_vehicle.send_mavlink_message.assert_not_called()
        mock_ws.broadcast.assert_not_called()

    def test_unknown_action_400_no_mavlink_no_broadcast(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": True, "action": "nuke",
        })
        assert resp.status_code == 400
        mock_vehicle.send_mavlink_message.assert_not_called()
        mock_ws.broadcast.assert_not_called()

    def test_action_bool_mismatch_400_no_mavlink_no_broadcast(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client
        resp = tc.post("/api/control/task_confirm", json={
            "sys_id": 1, "task_id": 5, "is_confirmed": True, "action": "cancel",
        })
        assert resp.status_code == 400
        mock_vehicle.send_mavlink_message.assert_not_called()
        mock_ws.broadcast.assert_not_called()


class TestTaskConfirmResponseMessage:
    def test_response_echoes_exact_request_round_reference(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client

        response = tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 7,
            "is_confirmed": True,
            "round_uid": "424242:11",
        })

        assert response.status_code == 200
        sent = [call.args[0] for call in mock_vehicle.send_mavlink_message.call_args_list]
        assert len(sent) == 3
        assert {(message.boot_id, message.msg_seq) for message in sent} == {
            (424242, 11)
        }
        assert all(message.time_ms > 0 and message.ttl_ms > 0 for message in sent)

    def test_response_accepts_zero_boot_id_uint32_edge(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client

        response = tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 7,
            "is_confirmed": True,
            "round_uid": "0:11",
        })

        assert response.status_code == 200
        message = mock_vehicle.send_mavlink_message.call_args.args[0]
        assert (message.boot_id, message.msg_seq) == (0, 11)

    @pytest.mark.parametrize(
        "round_uid",
        (
            "0:0",
            "-1:2",
            "1:4294967296",
            "abc:1",
            "1",
            "1:2:3",
            "01:002",
            "00:1",
            "1:00",
            "١:٢",
        ),
    )
    def test_malformed_round_uid_is_400_before_send(
        self,
        client,
        mock_vehicle,
        round_uid,
    ):
        tc, mock_mgr, mock_ws = client

        response = tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 7,
            "is_confirmed": True,
            "round_uid": round_uid,
        })

        assert response.status_code == 400
        mock_vehicle.send_mavlink_message.assert_not_called()
        mock_ws.broadcast.assert_not_called()

    def test_mavlink_message_fields(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client
        tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 7,
            "is_confirmed": True,
            "round_uid": "legacy",
        })

        mav_msg = mock_vehicle.send_mavlink_message.call_args[0][0]
        # Addressed to the plain aircraft sysid: the companion shares it and is
        # told apart by component 191, and ArduPilot forwards a packet targeted
        # at its own system id onward to the companion on serial0.
        assert mav_msg.target_system == 1
        assert mav_msg.task_id == 7
        assert mav_msg.confirmed == 1

    def test_receiver_id_tracks_the_requested_sys_id(self, client, mock_vehicle, mock_entry):
        """receiver_id IS req.sys_id -- no companion-id derivation left, for
        any sys_id."""
        tc, mock_mgr, mock_ws = client
        for sys_id in (1, 2, 121, 255):
            mock_entry.sys_id = sys_id
            mock_vehicle.send_mavlink_message.reset_mock()
            tc.post("/api/control/task_confirm", json={
                "sys_id": sys_id,
                "task_id": 4,
                "is_confirmed": True,
                "round_uid": "legacy",
            })
            mav_msg = mock_vehicle.send_mavlink_message.call_args[0][0]
            assert mav_msg.target_system == sys_id

    def test_deny_sets_confirmed_zero(self, client, mock_vehicle):
        tc, mock_mgr, mock_ws = client
        tc.post("/api/control/task_confirm", json={
            "sys_id": 1,
            "task_id": 2,
            "is_confirmed": False,
            "round_uid": "legacy",
        })

        mav_msg = mock_vehicle.send_mavlink_message.call_args[0][0]
        assert mav_msg.confirmed == 0
