"""EKF launch gate (docs/design/ekf_launch_gate.md).

Readiness verdicts on a real VehicleMav fed through its inbound router, the
EKF_STATUS_REPORT stream request on connect and reconnect, and parity of the
frontend mirror with the backend's masks, freshness and verdicts.
"""
from __future__ import annotations

import json
import os
import re
import threading
import types
from unittest.mock import MagicMock, patch

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAV_COMP_ID_AUTOPILOT1,
    MAV_TYPE_GCS,
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAVLINK_MSG_ID_EKF_STATUS_REPORT,
)

from gcs.backend import ekf_status_gate as gate
from gcs.backend.routes.control_readiness import (
    ReadinessGates, _check_launch_readiness, ekf_recheck_for_session,
)
from gcs.backend.vehicle_entry import VehicleEntry
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav
from tests.gcs.js_runner import run_node
from tests.gcs.test_preflight_vehicle_mav import _ekf_report, _feed, _make_vehicle

READY = 0x37
UNINIT = 0x400
CONST_POS = 0x80
GLITCH = 0x8000
VERT_ABS = 0x20


def test_mask_pair_is_named_from_pymavlink_enums():
    assert gate.EKF_REQUIRED_MASK == 0x37
    assert gate.EKF_FORBIDDEN_MASK == 0x8480
    assert not gate.EKF_REQUIRED_MASK & gate.EKF_FORBIDDEN_MASK
    assert all(gate.EKF_REQUIRED_BITS.values())
    assert all(gate.EKF_FORBIDDEN_BITS.values())


def test_stale_limit_is_a_few_stream_intervals():
    assert gate.EKF_STATUS_STALE_S == pytest.approx(4.0 / gate.EKF_STATUS_RATE_HZ)


@pytest.mark.parametrize("ekf, message", [
    (None, "waiting for EKF status"),
    ({"flags": UNINIT, "age_s": 0.0}, "not initialised"),
    ({"flags": 0x0, "age_s": 0.0}, "no attitude"),
    ({"flags": READY & ~VERT_ABS, "age_s": 0.0}, "no absolute vertical position"),
    ({"flags": READY | CONST_POS, "age_s": 0.0}, "constant-position mode"),
    ({"flags": READY | GLITCH, "age_s": 0.0}, "GPS glitch"),
    ({"flags": READY, "age_s": gate.EKF_STATUS_STALE_S + 0.1}, "EKF status stale"),
    ({"flags": READY}, "EKF status stale"),
])
def test_blocking_reports_name_their_reason(ekf, message):
    issue = gate.ekf_status_issue(ekf)
    assert issue is not None and message in issue


def test_ready_report_passes():
    assert gate.ekf_status_issue({"flags": READY, "age_s": 0.0}) is None
    # Extra informational bits (relative position, predictions) do not block.
    assert gate.ekf_status_issue({"flags": 0x33F, "age_s": 0.0}) is None


# --- a real vehicle: own-autopilot admission, receipt age, readiness --------

def _readiness(entry, gates=None):
    manager = MagicMock()
    manager.get_vehicle.return_value = entry
    manager.companion_status.return_value = "ok"
    issues = _check_launch_readiness([1], gates, vehicle_mgr=manager)
    return [i for i in issues if "EKF" in i]


@pytest.fixture
def entry(request):
    vehicle = _make_vehicle()
    request.addfinalizer(vehicle.close)
    return VehicleEntry(1, vehicle, "test")


def test_no_report_blocks(entry):
    assert _readiness(entry) == ["Vehicle 1: waiting for EKF status"]


@pytest.mark.parametrize("flags, message", [
    (UNINIT, "not initialised"),
    (0x0, "no attitude"),
    (READY & ~VERT_ABS, "no absolute vertical position"),
    (READY | CONST_POS, "constant-position mode"),
    (READY | GLITCH, "GPS glitch"),
])
def test_own_report_flags_block(entry, flags, message):
    _feed(entry.vehicle, "EKF_STATUS_REPORT", _ekf_report(flags=flags))
    [issue] = _readiness(entry)
    assert issue.startswith("Vehicle 1: EKF not ready") and message in issue


def test_ready_own_report_passes(entry):
    _feed(entry.vehicle, "EKF_STATUS_REPORT", _ekf_report(flags=READY))
    assert _readiness(entry) == []


def test_glitch_then_clear_passes(entry):
    _feed(entry.vehicle, "EKF_STATUS_REPORT", _ekf_report(flags=READY | GLITCH))
    assert _readiness(entry)
    _feed(entry.vehicle, "EKF_STATUS_REPORT", _ekf_report(flags=READY))
    assert _readiness(entry) == []


def test_stale_report_blocks(entry):
    now = 1_000.0
    with patch("navpy.core.kernel.runtime_time.time", lambda: now):
        _feed(entry.vehicle, "EKF_STATUS_REPORT", _ekf_report(flags=READY))
        assert _readiness(entry) == []
        assert entry.snapshot()["ekf"]["age_s"] == 0.0
    later = now + gate.EKF_STATUS_STALE_S + 0.5
    with patch("navpy.core.kernel.runtime_time.time", lambda: later):
        assert _readiness(entry) == ["Vehicle 1: EKF status stale"]
        assert entry.snapshot()["ekf"]["age_s"] == pytest.approx(gate.EKF_STATUS_STALE_S + 0.5)


def test_other_components_report_does_not_count(entry):
    report = _ekf_report(flags=READY)
    report.get_msgId = lambda: 0
    report.get_srcSystem = lambda: 1
    report.get_srcComponent = lambda: MAV_COMP_ID_ONBOARD_COMPUTER
    report.get_seq = lambda: 0
    report.get_type = lambda: "EKF_STATUS_REPORT"
    entry.vehicle.feed_message(report)
    assert entry.vehicle.ekf_status is None
    assert _readiness(entry) == ["Vehicle 1: waiting for EKF status"]
    # The vehicle's own autopilot then reports: that one counts.
    _feed(entry.vehicle, "EKF_STATUS_REPORT", _ekf_report(flags=READY))
    assert _readiness(entry) == []


def _gcs_vehicle(request, target_system=1):
    """A GCS-identity VehicleMav, as the GCS backend builds one per vehicle."""
    bus = MagicMock(conn=types.SimpleNamespace(mav=MagicMock()),
                    send_lock=threading.RLock(), heartbeats={})
    bus.reserve.return_value = MagicMock()
    vehicle = VehicleMav("unused", target_system=target_system, logger=MagicMock(),
                         skip_mission_download=True, wait_heartbeat=False,
                         send_heartbeat=False, mav_type=MAV_TYPE_GCS, bus=bus)
    request.addfinalizer(vehicle.close)
    return vehicle


def _ekf_from(system, component, flags=READY):
    report = _ekf_report(flags=flags)
    report.get_msgId = lambda: 0
    report.get_srcSystem = lambda: system
    report.get_srcComponent = lambda: component
    report.get_seq = lambda: 0
    report.get_type = lambda: "EKF_STATUS_REPORT"
    return report


@pytest.mark.parametrize("system, component", [
    (9, MAV_COMP_ID_AUTOPILOT1),      # an unregistered peer's autopilot
    (2, MAV_COMP_ID_AUTOPILOT1),      # another fleet vehicle's autopilot
    (1, MAV_COMP_ID_ONBOARD_COMPUTER),  # the right system, another component
])
def test_gcs_identity_admits_only_its_own_autopilots_report(request, system, component):
    """A GCS-mode router publishes foreign systems' messages; a peer's healthy
    EKF report must still never satisfy this vehicle's gate."""
    vehicle = _gcs_vehicle(request)
    vehicle.feed_message(_ekf_from(system, component))
    assert vehicle.ekf_status is None
    vehicle.feed_message(_ekf_from(1, MAV_COMP_ID_AUTOPILOT1, flags=UNINIT))
    assert vehicle.ekf_status["flags"] == UNINIT
    # A foreign healthy report afterwards does not replace the own bad one.
    vehicle.feed_message(_ekf_from(system, component))
    assert vehicle.ekf_status["flags"] == UNINIT


def test_gate_off_skips_the_ekf(entry):
    assert _readiness(entry, ReadinessGates(check_ekf=False)) == []
    recheck = ekf_recheck_for_session(
        ReadinessGates(check_ekf=False), force=False,
        vehicle_mgr=MagicMock(get_vehicle=MagicMock(return_value=entry)),
    )
    assert recheck(1) == []


def test_session_recheck_force_skips_and_default_gates_check(entry):
    manager = MagicMock(get_vehicle=MagicMock(return_value=entry))
    assert ekf_recheck_for_session(None, force=False, vehicle_mgr=manager)(1) == [
        "Vehicle 1: waiting for EKF status"]
    assert ekf_recheck_for_session(None, force=True, vehicle_mgr=manager)(1) == []


# --- stream request on connect and reconnect --------------------------------

def _interval_requests(vehicle):
    return [
        c for c in vehicle.send_command_long.call_args_list
        if c.args and c.args[0] == MAV_CMD_SET_MESSAGE_INTERVAL
        and c.kwargs.get("p1") == MAVLINK_MSG_ID_EKF_STATUS_REPORT
    ]


def test_stream_requested_per_vehicle_on_connect_and_reconnect():
    from gcs.backend.vehicle_manager import VehicleManager

    made = []

    def fake_vehicle_mav(**kwargs):
        vehicle = MagicMock(link_ok=True)
        made.append((kwargs["target_system"], vehicle))
        return vehicle

    manager = VehicleManager(open_bus=lambda *a, **k: MagicMock(is_closed=False, is_closing=False))
    with (
        patch("gcs.backend.vehicle_manager.VehicleMav", side_effect=fake_vehicle_mav),
        patch.object(VehicleManager, "_probe_mission_async"),
    ):
        manager.add_vehicle("udp:test", 1)
        manager.add_vehicle("udp:test", 2)
        manager.reconnect_vehicle(1, timeout=0.5)

    assert [sid for sid, _ in made] == [1, 2, 1]
    expected_us = int(round(1_000_000.0 / gate.EKF_STATUS_RATE_HZ))
    for _, vehicle in made:
        [request] = _interval_requests(vehicle)
        assert request.kwargs["p2"] == expected_us


def test_failed_stream_request_does_not_fail_connect():
    vehicle = MagicMock()
    vehicle.send_command_long.side_effect = OSError("link down")
    gate.request_ekf_status_stream(vehicle, 1)  # logged, not raised


# --- frontend mirror ---------------------------------------------------------

_PREARM_JS = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..",
    "src", "gcs", "frontend", "src", "utils", "prearmChecks.js",
))


def _js(expr):
    with open(_PREARM_JS, encoding="utf-8") as f:
        src = f.read().replace("export ", "")
    result = run_node(src + f"\nconsole.log(JSON.stringify({expr}));", timeout=5)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_frontend_constants_match_backend():
    js = _js("{rate: EKF_STATUS_RATE_HZ, stale: EKF_STATUS_STALE_S, "
             "req: Object.keys(EKF_REQUIRED_BITS).map(Number), "
             "forb: Object.keys(EKF_FORBIDDEN_BITS).map(Number)}")
    assert js["rate"] == gate.EKF_STATUS_RATE_HZ
    assert js["stale"] == gate.EKF_STATUS_STALE_S
    assert sorted(js["req"]) == sorted(gate.EKF_REQUIRED_BITS)
    assert sorted(js["forb"]) == sorted(gate.EKF_FORBIDDEN_BITS)


_PARITY_CASES = [
    None,
    {"flags": UNINIT, "age_s": 0.0},
    {"flags": 0x0, "age_s": 0.0},
    {"flags": READY & ~VERT_ABS, "age_s": 0.0},
    {"flags": READY | CONST_POS, "age_s": 0.0},
    {"flags": READY | GLITCH, "age_s": 0.0},
    {"flags": READY, "age_s": gate.EKF_STATUS_STALE_S},
    {"flags": READY, "age_s": gate.EKF_STATUS_STALE_S + 0.01},
    {"flags": READY},
    {"flags": READY, "age_s": 0.3},
    {"flags": 0x33F, "age_s": 0.3},
]


_PREFLIGHT_JS = os.path.join(os.path.dirname(_PREARM_JS), "preflightReadiness.js")


def _row_status(ekf):
    with open(_PREARM_JS, encoding="utf-8") as f:
        prearm = f.read().replace("export ", "")
    with open(_PREFLIGHT_JS, encoding="utf-8") as f:
        preflight = re.sub(r"^import .*\n", "", f.read().replace("export ", ""), flags=re.M)
    stub = "function computeFleetHarmonize() { return { rows: [] }; }\n"
    expr = f"computeEkfCheck({json.dumps(ekf)}).status"
    result = run_node(stub + prearm + "\n" + preflight
                      + f"\nconsole.log(JSON.stringify({expr}));", timeout=5)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


_VARIANCE_CASES = [
    {"flags": READY, "age_s": 0.1, "velocity_variance": 0.9},
    {"flags": READY, "age_s": 0.1, "pos_vert_variance": 0.6},
]


@pytest.mark.parametrize("ekf", _PARITY_CASES + _VARIANCE_CASES)
def test_frontend_ekf_row_is_nogo_exactly_when_backend_blocks(ekf):
    blocked = gate.ekf_status_issue(ekf) is not None
    assert (_row_status(ekf) == "nogo") is blocked


@pytest.mark.parametrize("ekf", _PARITY_CASES + _VARIANCE_CASES)
def test_frontend_verdict_matches_backend(ekf):
    blocked = gate.ekf_status_issue(ekf) is not None
    vehicle = {"prearm_ok": True, "gps_fix": 3, "companion_ok": True,
               "mission_uploaded": True, "ekf": ekf}
    js = _js(f"{{issue: ekfStatusIssue({json.dumps(ekf)}), "
             f"ready: computeLaunchReadiness({json.dumps(vehicle)}).ready, "
             f"off: computeLaunchReadiness({json.dumps(vehicle)}, "
             f"{{...DEFAULT_LAUNCH_GATES, checkEkf: false}}).ready}}")
    assert (js["issue"] is not None) is blocked
    assert js["ready"] is not blocked
    assert js["off"] is True


def test_frontend_settings_map_the_gate():
    assert _js("launchGatesFromSettings({launch: {check_ekf_enabled: false}}).checkEkf") is False
    assert _js("launchGatesFromSettings({launch: {}}).checkEkf") is True
