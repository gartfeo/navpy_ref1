"""Tests for the CONF-03 "Ask me anyway" gate-bypass override POST endpoint (D-18)."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from navpy.modules.comm.messages.swarm_request_msg import REQUEST_TYPE_FORCE_CONFIRM


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

    with patch("gcs.backend.routes.task_force_confirm.vehicle_mgr", mock_mgr):
        from gcs.backend.routes.task_force_confirm import router
        from fastapi import FastAPI
        app = FastAPI()
        app.include_router(router, prefix="/api/control")
        yield TestClient(app), mock_mgr


class TestTaskForceConfirmRoute:
    def test_sends_swarm_request_force_confirm(self, client, mock_vehicle):
        tc, mock_mgr = client
        resp = tc.post("/api/control/task_force_confirm", json={
            "sys_id": 1,
            "task_id": 5,
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "sent"
        assert data["delivery"] == "sent"
        assert data["task_id"] == 5
        # Unacknowledged transport, mirrors task_confirm.py's response resend.
        assert mock_vehicle.send_mavlink_message.call_count == 3

    def test_unknown_vehicle_returns_404(self, client):
        tc, mock_mgr = client
        mock_mgr.get_vehicle.return_value = None
        resp = tc.post("/api/control/task_force_confirm", json={
            "sys_id": 99,
            "task_id": 1,
        })
        assert resp.status_code == 404

    def test_mavlink_message_fields(self, client, mock_vehicle):
        tc, mock_mgr = client
        tc.post("/api/control/task_force_confirm", json={
            "sys_id": 1,
            "task_id": 7,
        })

        mav_msg = mock_vehicle.send_mavlink_message.call_args[0][0]
        # Addressed to the plain aircraft sysid -- the companion shares it and
        # its ConfirmOverrideListener filters on that same id.
        assert mav_msg.target_system == 1
        assert mav_msg.subject_id == 7
        assert mav_msg.request_type == REQUEST_TYPE_FORCE_CONFIRM

    def test_all_resends_share_one_valid_message_uid(self, client, mock_vehicle):
        """All 3 resend copies must carry the SAME non-zero (boot_id,
        msg_seq) so the receiver's dedup cache (message_filter.py's
        MessageFilter.should_process/_is_valid_meta) collapses them into a
        single delivery to ConfirmOverrideListener -- otherwise each copy is
        delivered separately and unconditionally re-adds task_id to the
        drone's one-shot force set, defeating the documented one-shot
        override invariant (a stale re-add can silently bypass the gate on
        a later, unrelated confirm round for the same task_id, e.g. after a
        D-14 bounded re-ask)."""
        tc, mock_mgr = client
        tc.post("/api/control/task_force_confirm", json={
            "sys_id": 1,
            "task_id": 7,
        })

        mav_msgs = [c[0][0] for c in mock_vehicle.send_mavlink_message.call_args_list]
        assert len(mav_msgs) == 3

        boot_ids = {m.boot_id for m in mav_msgs}
        msg_seqs = {m.msg_seq for m in mav_msgs}
        assert len(boot_ids) == 1, "all 3 sends must share one boot_id"
        assert len(msg_seqs) == 1, "all 3 sends must share one msg_seq"
        # msg_seq=0 (paired with boot_id=0) is message_filter.py's
        # "invalid/legacy meta" signal that bypasses dedup entirely.
        # MsgMetaProvider._next_seq() always returns >= 1 (pre-increments
        # from 0), so this is a deterministic, non-flaky check that
        # set_meta_from_provider() actually ran (unlike boot_id, which is
        # random and could theoretically -- astronomically unlikely -- be 0).
        assert next(iter(msg_seqs)) != 0
