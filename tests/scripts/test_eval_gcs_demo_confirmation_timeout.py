"""A failed three-UAV demo must retain each confirmation state."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.eval_gcs_demo_confirmation import ConfirmationTimeout, run_confirmation_workflow
from scripts.eval_gcs_demo_mission import approve_requests_until_snap
from scripts.eval_gcs_demo_models import RegressionError


class _Clock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now

    def unix(self) -> float:
        return 1_000.0 + self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class _Stream:
    def __init__(self) -> None:
        self.events = iter(
            json.dumps(
                {
                    "type": "task_confirm_request",
                    "sys_id": sys_id,
                    "task_id": 1,
                    "round_uid": "10:1",
                }
            )
            for sys_id in (7, 4)
        )

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def receive(self, _timeout_s: float):
        return next(self.events, None)


class _Api:
    websocket_url = "ws://unused"

    def post_json(self, path: str, _payload: dict, **_kwargs):
        if path == "/api/control/launch/trigger":
            return 200, {"status": "triggered"}
        assert path == "/api/control/task_confirm"
        return 200, {"status": "approved"}


def test_timeout_names_missing_request_and_retains_partial_approvals() -> None:
    with pytest.raises(TimeoutError) as captured:
        run_confirmation_workflow(
            event_stream=_Stream(),
            api=_Api(),
            expected_sys_ids={7, 4, 9},
            trigger=lambda: None,
            stop_when=lambda: False,
            requested_delay_s=0.0,
            timeout_s=0.3,
            clock=_Clock(),
        )

    error = captured.value
    assert "missing confirmation request sysids [9]" in str(error)
    assert {row["sys_id"] for row in error.approvals} == {7, 4}
    assert {request.sys_id for request in error.requests} == {7, 4}


@pytest.mark.parametrize("tail_fails", (False, True))
def test_mission_timeout_preserves_vehicle_states_and_backend_tail(
    tmp_path: Path, tail_fails: bool,
) -> None:
    class Tail:
        text = "backend before timeout\n"

        def read_new(self) -> None:
            if tail_fails:
                raise OSError("injected backend tail failure")
            self.text += "backend at timeout\n"

    (tmp_path / "uav_7_navigation.log").write_text(
        "SNAP(VISION-NAV example\n", encoding="utf-8"
    )
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=Tail(),
    )

    with pytest.raises(ConfirmationTimeout, match="missing confirmation request sysids \\[9\\]") as captured:
        approve_requests_until_snap(
            context,
            object(),
            tmp_path,
            timeout_s=0.2,
            confirmation_delay_s=0.0,
            event_stream_factory=lambda _url: _Stream(),
        )

    artifact = json.loads(
        (tmp_path / "demo_confirmation_failure.json").read_text(encoding="utf-8")
    )
    assert artifact["expected_sys_ids"] == [7, 4, 9]
    assert artifact["status"] == "timeout"
    assert captured.value.missing_snap_sysids == (4, 9)
    assert {row["sys_id"] for row in captured.value.approvals} == {7, 4}
    assert artifact["vehicles"] == [
        {"sys_id": 7, "request_observed": True, "approval_attempted": True, "approval_accepted": True, "snap_observed": True},
        {"sys_id": 4, "request_observed": True, "approval_attempted": True, "approval_accepted": True, "snap_observed": False},
        {"sys_id": 9, "request_observed": False, "approval_attempted": False, "approval_accepted": False, "snap_observed": False},
    ]
    assert {row["sys_id"] for row in artifact["approvals"]} == {7, 4}
    backend_text = (tmp_path / "gcs_backend.partial.log").read_text(encoding="utf-8")
    assert "backend before timeout" in backend_text
    if tail_fails:
        assert "injected backend tail failure" in artifact["backend_tail_error"]
    else:
        assert "backend at timeout" in backend_text
        assert "backend_tail_error" not in artifact


def test_missing_approval_after_snap_also_retains_vehicle_states(
    monkeypatch, tmp_path: Path,
) -> None:
    import scripts.eval_gcs_demo_mission as mission

    class Tail:
        text = "backend evidence\n"

        def read_new(self) -> None:
            return None

    for sys_id in (7, 4, 9):
        (tmp_path / f"uav_{sys_id}_navigation.log").write_text(
            "SNAP(VISION-NAV example\n", encoding="utf-8"
        )

    def workflow(**kwargs):
        for sys_id in (7, 4):
            kwargs["on_request_observed"](
                mission.ConfirmationRequest(sys_id, 1, "10:1")
            )
        return [{"sys_id": sys_id, "task_id": 1} for sys_id in (7, 4)]

    monkeypatch.setattr(mission, "run_confirmation_workflow", workflow)
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=Tail(),
    )
    with pytest.raises(RegressionError, match="SNAP approvals are not exactly one") as captured:
        mission.approve_requests_until_snap(
            context,
            object(),
            tmp_path,
            timeout_s=1.0,
            confirmation_delay_s=0.0,
            event_stream_factory=lambda _url: _Stream(),
        )

    artifact = json.loads(
        (tmp_path / "demo_confirmation_failure.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "approval_mismatch"
    assert "missing confirmation request sysids [9]" in str(captured.value)
    assert "missing SNAP sysids []" in str(captured.value)
    assert artifact["vehicles"][-1] == {
        "sys_id": 9,
        "request_observed": False,
        "approval_attempted": False,
        "approval_accepted": False,
        "snap_observed": True,
    }


def test_nav_before_approval_keeps_partial_evidence(tmp_path: Path) -> None:
    class Tail:
        text = "backend before safety error\n"

        def read_new(self) -> None:
            return None

    (tmp_path / "uav_7_navigation.log").write_text(
        "Heartbeat from system 7\nINIT: NAV MODE\n", encoding="utf-8"
    )
    context = SimpleNamespace(sys_ids=(7, 4, 9), api=_Api(), backend_tail=Tail())
    with pytest.raises(RegressionError, match="entered NAV before delayed operator"):
        approve_requests_until_snap(
            context, object(), tmp_path, 0.2, 0.0,
            event_stream_factory=lambda _url: _Stream(),
        )

    artifact = json.loads(
        (tmp_path / "demo_confirmation_failure.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "workflow_error"
    assert artifact["vehicles"][0] == {
        "sys_id": 7,
        "request_observed": True,
        "approval_attempted": False,
        "approval_accepted": False,
        "snap_observed": False,
    }


def test_nav_error_keeps_vehicle_attribution_when_evidence_writes_fail(
    monkeypatch, tmp_path: Path,
) -> None:
    (tmp_path / "uav_7_navigation.log").write_text(
        "Heartbeat from system 7\nINIT: NAV MODE\n", encoding="utf-8"
    )

    def write_text(path: Path, text: str, **kwargs):
        raise OSError(f"injected {path.name} failure")

    monkeypatch.setattr(Path, "write_text", write_text)
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=SimpleNamespace(text="backend\n", read_new=lambda: None),
    )
    with pytest.raises(RegressionError) as caught:
        approve_requests_until_snap(
            context, object(), tmp_path, 0.2, 0.0,
            event_stream_factory=lambda _url: _Stream(),
        )
    message = str(caught.value)
    assert "entered NAV before delayed operator" in message
    assert "request observed sysids [7]" in message
    assert "approval attempted sysids []" in message
    assert "approval accepted sysids []" in message
    assert "missing SNAP sysids [7, 4, 9]" in message
    assert isinstance(caught.value.__cause__, RegressionError)


def test_rejected_approval_is_distinct_from_no_attempt(tmp_path: Path) -> None:
    class RejectApi(_Api):
        def post_json(self, path: str, payload: dict, **kwargs):
            if path == "/api/control/task_confirm":
                return 409, {"status": "rejected"}
            return super().post_json(path, payload, **kwargs)

    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=RejectApi(),
        backend_tail=SimpleNamespace(text="backend rejection\n", read_new=lambda: None),
    )
    with pytest.raises(ValueError, match="not approved"):
        approve_requests_until_snap(
            context, object(), tmp_path, 0.2, 0.0,
            event_stream_factory=lambda _url: _Stream(),
        )
    artifact = json.loads(
        (tmp_path / "demo_confirmation_failure.json").read_text(encoding="utf-8")
    )
    assert artifact["vehicles"][0]["approval_attempted"] is True
    assert artifact["vehicles"][0]["approval_accepted"] is False
    assert artifact["vehicles"][1]["approval_attempted"] is False


def test_backend_log_write_failure_does_not_hide_timeout(monkeypatch, tmp_path: Path) -> None:
    real_write_text = Path.write_text

    def write_text(path: Path, text: str, **kwargs):
        if path.name == "gcs_backend.partial.log":
            raise OSError("injected log write failure")
        return real_write_text(path, text, **kwargs)

    monkeypatch.setattr(Path, "write_text", write_text)
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=SimpleNamespace(text="backend\n", read_new=lambda: None),
    )
    with pytest.raises(RegressionError, match="evidence incomplete.*injected log write failure"):
        approve_requests_until_snap(
            context, object(), tmp_path, 0.2, 0.0,
            event_stream_factory=lambda _url: _Stream(),
        )
    artifact = json.loads(
        (tmp_path / "demo_confirmation_failure.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "timeout"


def test_both_evidence_writes_failing_preserve_missing_snap(monkeypatch, tmp_path: Path) -> None:
    def write_text(path: Path, text: str, **kwargs):
        raise OSError(f"injected {path.name} failure")

    monkeypatch.setattr(Path, "write_text", write_text)
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=SimpleNamespace(text="backend\n", read_new=lambda: None),
    )
    with pytest.raises(RegressionError) as caught:
        approve_requests_until_snap(
            context, object(), tmp_path, 0.2, 0.0,
            event_stream_factory=lambda _url: _Stream(),
        )
    message = str(caught.value)
    assert "missing SNAP sysids [7, 4, 9]" in message
    assert "demo_confirmation_failure.json" in message
    assert "gcs_backend.partial.log" in message
    assert isinstance(caught.value.__cause__, ConfirmationTimeout)


def test_snap_log_read_failure_keeps_original_timeout(monkeypatch, tmp_path: Path) -> None:
    import scripts.eval_gcs_demo_mission as mission

    def has_snap(_log_dir: Path, sys_id: int) -> bool:
        if sys_id == 7:
            raise OSError("injected navigation read failure")
        return False

    monkeypatch.setattr(mission, "_has_snap", has_snap)
    def timed_out(**_kwargs):
        raise ConfirmationTimeout({7, 4, 9}, set(), [])

    monkeypatch.setattr(mission, "run_confirmation_workflow", timed_out)
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=SimpleNamespace(text="backend\n", read_new=lambda: None),
    )
    with pytest.raises(RegressionError) as caught:
        mission.approve_requests_until_snap(
            context, object(), tmp_path, 0.2, 0.0,
            event_stream_factory=lambda _url: _Stream(),
        )
    assert "injected navigation read failure" in str(caught.value)
    assert isinstance(caught.value.__cause__, ConfirmationTimeout)
    artifact = json.loads(
        (tmp_path / "demo_confirmation_failure.json").read_text(encoding="utf-8")
    )
    assert artifact["vehicles"][0]["snap_observed"] is None


def test_success_keeps_failure_artifacts_absent(monkeypatch, tmp_path: Path) -> None:
    import scripts.eval_gcs_demo_mission as mission

    def workflow(**kwargs):
        rows = []
        for sys_id in (7, 4, 9):
            request = mission.ConfirmationRequest(sys_id, 1, "10:1")
            kwargs["on_request_observed"](request)
            kwargs["on_approval_attempted"](request)
            row = {"sys_id": sys_id, "task_id": 1}
            kwargs["on_approval_accepted"](row)
            rows.append(row)
        return rows

    monkeypatch.setattr(mission, "run_confirmation_workflow", workflow)
    context = SimpleNamespace(
        sys_ids=(7, 4, 9),
        api=_Api(),
        backend_tail=SimpleNamespace(text="", read_new=lambda: None),
    )
    approvals = mission.approve_requests_until_snap(
        context, object(), tmp_path, 1.0, 0.0,
        event_stream_factory=lambda _url: _Stream(),
    )
    assert {row["sys_id"] for row in approvals} == {7, 4, 9}
    assert not (tmp_path / "demo_confirmation_failure.json").exists()
    assert not (tmp_path / "gcs_backend.partial.log").exists()
