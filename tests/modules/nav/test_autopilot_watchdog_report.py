"""The autopilot's NavPy watchdog acting is a recovery NavPy journals (Codex review of step 8, finding 3), and the
in-process watchdog's reports reach the journal too."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

from pymavlink.dialects.v20 import ardupilotmega as mav2

from navpy.app.autopilot_watchdog_report import AutopilotWatchdogReport, AutopilotWatchdogSubscriptions
from tests.logger.recording_logger import RecordingLogger


def _named(value: float, *, name: bytes = b"NAVWD", sysid: int = 3, compid: int = 1):
    message = mav2.MAVLink_named_value_float_message(0, name, value)
    message._header = SimpleNamespace(srcSystem=sysid, srcComponent=compid)
    message.get_srcSystem = lambda: sysid
    message.get_srcComponent = lambda: compid
    return message


def _text(text: str, *, sysid: int = 3, compid: int = 1):
    message = mav2.MAVLink_statustext_message(2, text.encode())
    message.get_srcSystem = lambda: sysid
    message.get_srcComponent = lambda: compid
    return message


def test_the_lua_watchdog_s_hold_and_rtl_are_journaled_as_recoveries():
    logger = RecordingLogger()
    report = AutopilotWatchdogReport(logger, 3)
    for value in (0, 0, 1, 1, 2, 2, 0, 0):
        report.on_named_value(_named(value))
    assert logger.journal.records == [
        ("autopilot", "autopilot NavPy watchdog: hold"),
        ("autopilot", "autopilot NavPy watchdog: RTL"),
        ("autopilot", "autopilot NavPy watchdog: watching"),
    ]


def test_a_navpy_started_during_the_lua_hold_journals_it():
    logger = RecordingLogger()
    AutopilotWatchdogReport(logger, 3).on_named_value(_named(1))
    assert logger.journal.records == [("autopilot", "autopilot NavPy watchdog: hold")]


def test_the_lua_watchdog_s_action_texts_are_journaled():
    logger = RecordingLogger()
    report = AutopilotWatchdogReport(logger, 3)
    report.on_status_text(_text("NavPy watchdog: no NavPy heartbeat, LOITER"))
    report.on_status_text(_text("Throttle disarmed"))
    assert logger.journal.records == [("autopilot", "autopilot NavPy watchdog: no NavPy heartbeat, LOITER")]


def test_only_this_aircraft_s_autopilot_counts():
    logger = RecordingLogger()
    report = AutopilotWatchdogReport(logger, 3)
    report.on_named_value(_named(1, sysid=4))
    report.on_named_value(_named(1, compid=191))
    report.on_named_value(_named(1, name=b"OTHER"))
    report.on_status_text(_text("NavPy watchdog: no NavPy heartbeat, LOITER", sysid=4))
    assert logger.journal.records == []


def test_the_subscriptions_listen_to_both_messages_and_cancel():
    vehicle = SimpleNamespace(target_system=3, on_message=Mock(side_effect=lambda name, cb: Mock(name=name)))
    subscriptions = AutopilotWatchdogSubscriptions(vehicle, RecordingLogger())
    assert [c.args[0] for c in vehicle.on_message.call_args_list] == ["NAMED_VALUE_FLOAT", "STATUSTEXT"]
    held = list(subscriptions._subscriptions)
    subscriptions.close()
    assert all(s.cancel.called for s in held)


def test_runtime_composition_subscribes_the_report():
    import inspect

    from navpy.app import runtime_composition

    assert "AutopilotWatchdogSubscriptions(vehicle, logger)" in inspect.getsource(runtime_composition._build_foundation)


def test_the_in_process_watchdog_s_reports_reach_the_journal():
    """Codex review of step 8: the watchdog's wiring, as nav_watchdog_composition builds it."""
    from navpy.app.flow.nav_watchdog_composition import nav_watchdog
    from navpy.app.nav_recovery_link import nav_recovery_link
    from navpy.core.kernel.progress_beacon import ProgressBeacon
    from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence

    logger = RecordingLogger()
    threads = nav_watchdog(nav_recovery_link(MavCommandFence(), Mock()), ProgressBeacon(), None, Mock(), logger)
    threads._watchdog._ports.report("NavPy NAV loop stuck: commands fenced")
    assert logger.journal.records == [("watchdog", "NavPy NAV loop stuck: commands fenced")]
