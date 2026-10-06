"""The three-UAV harness must carry its own mission, not borrow the launcher's."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import eval_direct_pixel_pn as one
from scripts import eval_direct_pixel_pn_three_uav as three
from scripts import eval_siyi_geo_tracking_three_uav as siyi_geo
from scripts import eval_three_uav_mission as mission_setup


class _Sentinel(RuntimeError):
    """Ends run_case once everything under test has already happened."""


def _launch_recorder(monkeypatch: pytest.MonkeyPatch, order: list[str]) -> dict:
    """Stub the swarm launch, the upload and the heartbeat; record the calls."""
    seen: dict = {}

    def start_swarm(python, case_dir, speedup, **kwargs):
        seen["swarm"] = kwargs
        order.append("swarm")
        return None, SimpleNamespace(chat=41)

    def upload(chat, sys_ids, **kwargs):
        seen["upload"] = {"chat": chat, "sys_ids": sys_ids, **kwargs}
        order.append("upload")

    def heartbeat(device, timeout_s):
        order.append("heartbeat")
        return SimpleNamespace(close=lambda: None)

    def prepare(master, sys_ids, **kwargs):
        order.append("prepare")
        raise _Sentinel("stop after setup")

    monkeypatch.setattr(one, "_start_swarm", start_swarm)
    monkeypatch.setattr(three, "upload_missions", upload)
    monkeypatch.setattr(three, "wait_for_heartbeat", heartbeat)
    monkeypatch.setattr(three, "prepare_vehicles", prepare)
    monkeypatch.setattr(three, "stop_own_stack", lambda *a, **k: None)
    monkeypatch.setattr(one, "_terminate", lambda *a, **k: None)
    monkeypatch.setattr(three.ip, "sysids_for_chat", lambda chat: [124, 125, 126])
    monkeypatch.setattr(three.ip, "monitor_device", lambda chat: f"udp:127.0.0.1:{chat}")
    return seen


def test_run_case_uploads_its_own_mission_before_the_evaluator_link(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    order: list[str] = []
    seen = _launch_recorder(monkeypatch, order)
    args = three._parser().parse_args([])

    with pytest.raises(_Sentinel):
        three.run_case(
            Path("python"), tmp_path, speedup=1.0, repetition=1, args=args
        )

    # Before the fix the harness downloaded whatever mission the launcher's
    # WSL template happened to hold; a template with a cleared mission count
    # made every sysid == 1 mod 3 report zero NAV_WAYPOINT items.
    assert order == ["swarm", "upload", "heartbeat", "prepare"]
    assert seen["upload"]["sys_ids"] == [124, 125, 126]
    # Same start point the swarm was launched at, so the target is reachable.
    assert seen["upload"]["home"] == one._home_lat_lon(args.home)
    assert seen["upload"]["waypoint_offset"] == one.DEFAULT_WAYPOINT_OFFSET_M
    assert seen["upload"]["gate_offset"] == one.DEFAULT_GATE_OFFSET_M


def test_run_case_collapses_the_launch_grid(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    seen = _launch_recorder(monkeypatch, [])
    args = three._parser().parse_args([])

    with pytest.raises(_Sentinel):
        three.run_case(
            Path("python"), tmp_path, speedup=1.0, repetition=1, args=args
        )

    # The default 50 m grid would put vehicles 2 and 3 outside the uploader's
    # 25 m home check, and would make the three runs three flights instead of
    # three replicates of one.
    assert seen["swarm"]["dist"] == mission_setup.LAUNCH_SPACING_M == 0
    assert seen["swarm"]["home"] == args.home


def test_upload_missions_writes_one_line_per_vehicle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    uploaded: list[tuple[str, int, tuple[float, float]]] = []

    def upload_north_line(device, sysid, *, home, echo, **kwargs):
        echo(f"uploaded {sysid}")
        uploaded.append((device, sysid, home))
        return 5

    monkeypatch.setattr(mission_setup, "upload_north_line", upload_north_line)
    monkeypatch.setattr(
        mission_setup.ip, "monitor_device", lambda chat: f"udp:127.0.0.1:{chat}"
    )

    mission_setup.upload_missions(
        41,
        [124, 125, 126],
        home=(43.0, 34.0),
        loiter_offset=one.DEFAULT_LOITER_OFFSET_M,
        gate_offset=one.DEFAULT_GATE_OFFSET_M,
        waypoint_offset=one.DEFAULT_WAYPOINT_OFFSET_M,
        alt_m=400.0,
        case_dir=tmp_path,
    )

    assert [row[1] for row in uploaded] == [124, 125, 126]
    assert {row[2] for row in uploaded} == {(43.0, 34.0)}
    for sys_id in (124, 125, 126):
        transcript = (tmp_path / f"mission-{sys_id}.log").read_text(encoding="utf-8")
        assert transcript == f"uploaded {sys_id}\n"


@pytest.mark.parametrize("parser", [three._parser, siyi_geo._parser])
def test_three_uav_parsers_reject_cruise_speedup(parser) -> None:
    # Accepting it silently flew the whole run at the scored speed while the
    # result read like a cruise-stepped one.
    with pytest.raises(SystemExit):
        parser().parse_args(["--cruise-speedup", "10"])


def test_single_uav_parser_still_accepts_cruise_speedup() -> None:
    assert one._parser().parse_args(["--cruise-speedup", "10"]).cruise_speedup == 10.0


def test_geometry_record_stamps_what_the_run_measured() -> None:
    # summary.json used to record only pass/fail, speedup, repetition and wind,
    # so a run at one home against one mission looked identical to a run at
    # another.  That is how the 2026-08-17 --home change went unnoticed.
    args = three._parser().parse_args([])
    record = mission_setup.geometry_record(args)

    assert record["home"] == one.DEFAULT_HOME_COORDS
    assert record["launch_spacing_m"] == mission_setup.LAUNCH_SPACING_M
    assert record["target_offset_m"] == one.DEFAULT_WAYPOINT_OFFSET_M
    assert record["gate_offset_m"] == one.DEFAULT_GATE_OFFSET_M
    assert record["loiter_offset_m"] == one.DEFAULT_LOITER_OFFSET_M
    assert record["mission_alt_m"] == args.mission_alt
    assert record["target_alt_m"] == args.target_alt
    # Wind was already recorded; folding it in must not drop it.
    assert record["wind_speed_mps"] == args.wind_speed
    assert record["wind_dir_deg"] == args.wind_dir


def test_summary_artifact_carries_the_geometry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The recorded artifact, not just the helper, has to say what it measured."""
    def boom(*args, **kwargs):
        raise RuntimeError("no SITL in a unit test")

    monkeypatch.setattr(one, "WORKTREE", tmp_path)
    monkeypatch.setattr(three, "run_case", boom)
    monkeypatch.setattr(sys, "argv", ["eval", "--speedups", "1", "--repetitions", "1"])

    assert three.main() == 1

    summaries = list(tmp_path.glob(".sitl-runs/*/summary.json"))
    assert len(summaries) == 1
    row = json.loads(summaries[0].read_text(encoding="utf-8"))["results"][0]
    assert row["home"] == one.DEFAULT_HOME_COORDS
    assert row["launch_spacing_m"] == mission_setup.LAUNCH_SPACING_M
    assert row["target_offset_m"] == one.DEFAULT_WAYPOINT_OFFSET_M
    assert row["wind_speed_mps"] == 0.0


class _ArmRecorder:
    """Queue-backed master recording the exact order of vehicle commands.

    recv_match reproduces pymavlink's discard semantics (see _FakeMaster in
    test_eval_navigation_telemetry): a non-matching message is consumed, not
    left queued.
    """

    def __init__(self, messages) -> None:
        from collections import deque

        self._queue = deque(messages)
        self.target_system = 0
        self.target_component = 1
        self.order: list[str] = []
        self.mav = SimpleNamespace(command_long_send=self._command)

    def recv_match(self, type=None, blocking=False, timeout=None):
        if type is not None and not isinstance(type, (list, set)):
            type = [type]
        while self._queue:
            message = self._queue.popleft()
            if type is None or message.get_type() in type:
                return message
        return None

    def mode_mapping(self):
        return {"AUTO": 10}

    def set_mode(self, mode):
        self.order.append(f"mode:{self.target_system}")

    def _command(self, target_system, target_component, command, *_rest):
        from pymavlink import mavutil

        name = {
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "arm",
            mavutil.mavlink.MAV_CMD_MISSION_START: "start",
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL: "gate",
        }.get(command, str(command))
        self.order.append(f"{name}:{target_system}")


def _gate_messages(sysid, ready=True):
    from tests.scripts.test_eval_nav_origin import _READY, _PRE_ORIGIN, _ack_from, _ekf

    return [_ack_from(sysid), _ekf(_READY if ready else _PRE_ORIGIN, sysid=sysid)]


def test_no_vehicle_arms_before_its_ekf_owns_an_absolute_solution() -> None:
    """The 2026-09-05 regression: arming first pinned the origin mid-climb.

    An origin taken while airborne cannot be moved afterwards, so every
    altitude the vehicle reported carried a fixed +1.66 m bias.  The gate has
    to clear for all three vehicles -- through the real message chain, not a
    stub -- before any of them arms.
    """
    rows = []
    for sysid in (121, 122, 123):
        rows += _gate_messages(sysid)
    master = _ArmRecorder(rows)
    three._start_missions(master, [121, 122, 123])
    assert master.order == [
        "gate:121", "gate:122", "gate:123",
        "mode:121", "arm:121", "start:121",
        "mode:122", "arm:122", "start:122",
        "mode:123", "arm:123", "start:123",
    ]


def test_a_vehicle_without_an_absolute_solution_is_never_armed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Patch the module the gate FUNCTION lives in.  scripts/ modules import
    # each other both as `scripts.eval_nav_origin` and as top-level
    # `eval_nav_origin`; those are two module objects, and patching the wrong
    # one leaves the real 120 s timeout in force.
    nav_origin = sys.modules[three._start_missions.__globals__[
        "require_fleet_nav_solution"].__module__]
    monkeypatch.setattr(nav_origin, "NAV_SOLUTION_TIMEOUT_S", 0.05)
    rows = _gate_messages(121) + _gate_messages(122, ready=False)
    master = _ArmRecorder(rows)
    with pytest.raises(RuntimeError, match="vehicle 122 did not report"):
        three._start_missions(master, [121, 122, 123])
    assert [entry for entry in master.order if not entry.startswith("gate:")] == []
