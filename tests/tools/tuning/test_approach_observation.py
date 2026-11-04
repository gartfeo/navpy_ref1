"""Observation (hitl_final_approach.md section 9, step 16): navpy is the router's reader while the harness flies the
fixed search lane, so none of its writes to the aircraft reaches the board; the runner's audit of the router's write
log at the board boundary counts the forbidden ones and those after the drain ack."""
from __future__ import annotations

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from tests.tools.tuning.test_mav_router import (NAVPY, RUNNER, UdpClient, decode, heartbeat, set_interval,
                                                wait_until)
from tests.tools.tuning.test_mav_router import GatedBoard
from tools.tuning import approach_chain as chain
from tools.tuning.approach_quiesce import allowed, audit
from tools.tuning.mav_router import build_parser
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mav_router_policy import observation_allowed

M = mavlink2
ATTITUDE = M.MAVLINK_MSG_ID_ATTITUDE

# What navpy sends today (its vehicle plugin's sends and its camera/gimbal status) and may send in observation.
READS_AND_REPORTS = [
    heartbeat(),
    M.MAVLink_statustext_message(M.MAV_SEVERITY_INFO, b"navpy"),
    M.MAVLink_param_request_read_message(1, 1, b"AAS_TARG_WPS", -1),
    M.MAVLink_param_request_list_message(1, 1),
    M.MAVLink_mission_request_list_message(1, 1, 0),
    M.MAVLink_mission_request_int_message(1, 1, 3, 0),
    M.MAVLink_mission_ack_message(1, 1, 0, 0),
    M.MAVLink_terrain_check_message(403117412, 444552112),
    M.MAVLink_command_long_message(1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0, M.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0,
                                   0, 0),
    set_interval(ATTITUDE, 20000),
    M.MAVLink_camera_settings_message(0, 1, 1.0, 0.0),
    M.MAVLink_camera_fov_status_message(0, 0, 0, 0, 0, 0, 0, [1, 0, 0, 0], 60.0, 34.0),
    M.MAVLink_gimbal_device_attitude_status_message(1, 1, 0, 0, [1, 0, 0, 0], 0, 0, 0, 0),
]
# navpy's writes to the aircraft (its vehicle plugin's sends: modes, arming, targets, missions, parameters, RC,
# files), the runner-only stream request, and a message nobody named: all blocked in observation.
WRITES = [
    M.MAVLink_set_attitude_target_message(0, 1, 1, 0, [1, 0, 0, 0], 0, 0, 0, 0.5),
    M.MAVLink_set_mode_message(1, 1, 15),
    M.MAVLink_command_long_message(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, 15, 0, 0, 0, 0, 0),
    M.MAVLink_command_long_message(1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0),
    M.MAVLink_command_int_message(1, 1, 6, M.MAV_CMD_DO_REPOSITION, 0, 0, -1, 1, 0, 0, 403117412, 444552112, 80.0),
    M.MAVLink_command_long_message(1, 1, M.MAV_CMD_DO_SET_ROI_LOCATION, 0, 0, 0, 0, 0, 40.3, 44.4, 0),
    M.MAVLink_mission_count_message(1, 1, 3, 0),
    M.MAVLink_mission_item_int_message(1, 1, 0, 6, M.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, 1, 2, 80.0, 0),
    M.MAVLink_mission_item_message(1, 1, 0, 6, M.MAV_CMD_NAV_WAYPOINT, 2, 1, 0, 0, 0, 0, 40.3, 44.4, 80.0, 0),
    M.MAVLink_mission_clear_all_message(1, 1, 0),
    M.MAVLink_mission_set_current_message(1, 1, 2),
    M.MAVLink_param_set_message(1, 1, b"AAS_NAV_ONESHOT", 1.0, M.MAV_PARAM_TYPE_REAL32),
    M.MAVLink_rc_channels_override_message(1, 1, *([1500] * 8)),
    M.MAVLink_manual_control_message(1, 0, 0, 500, 0, 0),
    M.MAVLink_request_data_stream_message(1, 1, M.MAV_DATA_STREAM_ALL, 10, 1),
    M.MAVLink_file_transfer_protocol_message(0, 1, 1, [0] * 251),
    M.MAVLink_home_position_message(0, 0, 0, 0, 0, 0, [1, 0, 0, 0], 0, 0, 0),
]


@pytest.mark.parametrize("msg", READS_AND_REPORTS, ids=lambda m: m.get_type())
def test_reads_reports_and_stream_claims_pass(msg):
    parsed = decode(NAVPY, msg)
    assert observation_allowed(parsed)
    assert allowed(parsed.get_msgId(), getattr(parsed, "command", None))      # the audit agrees with the router


@pytest.mark.parametrize("msg", WRITES, ids=lambda m: m.get_type() + str(getattr(m, "command", "")))
def test_writes_to_the_aircraft_are_forbidden(msg):
    parsed = decode(NAVPY, msg)
    assert not observation_allowed(parsed)
    assert not allowed(parsed.get_msgId(), getattr(parsed, "command", None))


@pytest.fixture
def observing(tmp_path):
    """The router with navpy as a reader and the runner as a full client, writing its log to a file."""
    board, navpy, runner = GatedBoard(), UdpClient(NAVPY), UdpClient(RUNNER)
    log_path = tmp_path / "router_writes.csv"
    log = log_path.open("w", encoding="utf-8", buffering=1)
    router = MavRouter(board, [Client("navpy", navpy.port, False, reader=True), Client("runner", runner.port, False)],
                       announce=lambda line: None, control_port=0, write_log=log).start()
    ports = {name: c.sock.getsockname() for name, c in router.clients.items()}
    navpy.peer, runner.peer = ports["navpy"], ports["runner"]
    yield board, navpy, runner, router, log_path
    board.gate.set()
    router.close()
    log.close()
    navpy.sock.close()
    runner.sock.close()


def test_no_navpy_write_to_the_aircraft_reaches_the_board_and_the_runner_still_flies(observing):
    board, navpy, runner, router, log_path = observing
    for msg in WRITES + READS_AND_REPORTS:
        navpy.send(msg)
    runner.send(M.MAVLink_command_long_message(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, 10, 0, 0, 0, 0, 0))
    wait_until(lambda: len(board.written) == len(READS_AND_REPORTS) + 1)
    stats = router.report()["stats"]
    blocked = {key: n for key, n in stats.items() if key.startswith("blocked_navpy_")}
    assert sum(blocked.values()) == len(WRITES)
    kinds = [M.MAVLink(None).parse_buffer(buf)[0].get_type() for buf in board.written]
    assert sorted(kinds) == sorted([m.get_type() for m in READS_AND_REPORTS] + ["COMMAND_LONG"])
    ack = router.close_endpoint("navpy")
    result = audit(log_path, {"barrier": {"ack": ack}}, [])
    assert result["navpy_writes"] == len(READS_AND_REPORTS) and result["forbidden"] == 0
    assert result["after_drain"] == 0 and result["ok"]


def test_a_readers_stream_claim_still_reaches_the_board(observing):
    board, navpy, _, router, _ = observing
    navpy.send(set_interval(ATTITUDE, 20000))
    wait_until(lambda: len(board.written) == 1)
    assert router.report()["claims"] == {str(ATTITUDE): 20000}


# ---- the audit over a write log ----------------------------------------------------------------------
def _log(tmp_path, lines: list[str]):
    path = tmp_path / "router_writes.csv"
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    return path


def _ack(at: float) -> dict:
    return {"barrier": {"ok": True, "ack": {"op": "closed", "client": "navpy", "barrier_mono_s": at}}}


SET_MODE = f"{M.MAVLINK_MSG_ID_COMMAND_LONG},{M.MAV_CMD_DO_SET_MODE}"
INTERVAL = f"{M.MAVLINK_MSG_ID_COMMAND_LONG},{M.MAV_CMD_SET_MESSAGE_INTERVAL}"


def test_the_audit_counts_forbidden_navpy_writes_by_message_and_command(tmp_path):
    path = _log(tmp_path, [f"1.0,navpy,0,", f"1.1,navpy,{INTERVAL}", f"1.2,navpy,{SET_MODE}",
                           f"1.3,navpy,{M.MAVLINK_MSG_ID_SET_ATTITUDE_TARGET},", f"1.4,runner,{SET_MODE}",
                           f"1.5,router,{INTERVAL}"])
    result = audit(path, _ack(2.0), [])
    assert result["navpy_writes"] == 4 and result["forbidden"] == 2 and not result["ok"]
    assert result["forbidden_ids"] == {f"{M.MAVLINK_MSG_ID_COMMAND_LONG}/{M.MAV_CMD_DO_SET_MODE}": 1,
                                       str(M.MAVLINK_MSG_ID_SET_ATTITUDE_TARGET): 1}


def test_the_audit_counts_navpy_writes_after_the_first_drain_ack(tmp_path):
    path = _log(tmp_path, ["1.0,navpy,0,", "2.5,navpy,0,", "3.0,runner,11,"])
    navpys_own = [{"trigger": "terminal", "router_ack": {"barrier_mono_s": 2.0}}]
    result = audit(path, _ack(4.0), navpys_own)      # the runner's later, idempotent close does not move the drain
    assert result["drain_ack_mono_s"] == 2.0 and result["after_drain"] == 1 and not result["ok"]


def test_without_a_drain_ack_or_a_write_log_the_audit_fails(tmp_path):
    path = _log(tmp_path, ["1.0,navpy,0,"])
    assert audit(path, {"barrier": {"ok": False, "ack": None}}, [])["ok"] is False
    assert audit(tmp_path / "missing.csv", _ack(1.0), [])["ok"] is False


def test_a_write_log_line_of_the_old_three_column_form_has_no_command(tmp_path):
    path = _log(tmp_path, ["1.0,navpy,0", f"1.1,navpy,{M.MAVLINK_MSG_ID_COMMAND_LONG}"])
    assert audit(path, _ack(2.0), [])["forbidden"] == 1          # a command of unknown kind is not allowed


# ---- the chain ------------------------------------------------------------------------------------------
def test_the_approach_router_takes_navpy_as_a_reader(tmp_path):
    paths = chain.ChainPaths(tmp_path, tmp_path, "python", tmp_path, tmp_path / "p.json", tmp_path / "u.uproject",
                             "172.0.0.2", "172.0.0.1", "/mnt/c/py/python.exe", None)
    argv = list(chain.router(paths).argv)
    assert argv[argv.index("--reader") + 1] == f"navpy={chain.NAVPY_PORT}"
    assert f"navpy={chain.NAVPY_PORT}" not in [argv[i + 1] for i, a in enumerate(argv) if a == "--client"]
    args = build_parser().parse_args(argv[argv.index("--board"):])
    assert args.reader == [("navpy", chain.NAVPY_PORT)] and args.client == [("runner", chain.RUNNER_PORT)]
