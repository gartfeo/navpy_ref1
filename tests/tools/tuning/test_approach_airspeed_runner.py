"""The approach runner's eligible-airspeed side (tools/tuning/approach_airspeed.py, HITL step 20): pins and firmware
before arming, and navpy's source records failing the trial."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from navpy.app.approach.law_airspeed import NAVPY_AIRSPEED_MARKER
from tools.tuning import approach, approach_airspeed as airspeed

SHA = next(iter(airspeed.FIRMWARE_PINS))
COMMIT = airspeed.FIRMWARE_PINS[SHA]
GOOD = {"ARSPD_TYPE": 100.0, "ARSPD_USE": 1.0, "ARSPD_PRIMARY": 0.0}


def _version(commit: str = COMMIT) -> SimpleNamespace:
    return SimpleNamespace(flight_custom_version=commit.encode() + b"\x00" * (8 - len(commit)))


class _Board:
    def __init__(self, params=None, version=None, log=None) -> None:
        self._params = dict(GOOD if params is None else params)
        self._version = _version() if version is None else version
        self.log = [] if log is None else log
        self.mav = SimpleNamespace(target_system=1, target_component=1,
                                   mav=SimpleNamespace(command_long_send=lambda *a: self.log.append(("cmd", a[2:4]))))

    def get_param(self, name):
        self.log.append(("param", name))
        if name not in self._params:
            raise RuntimeError(f"no value for {name}")
        return self._params[name]

    def wait_message(self, mtype, timeout):
        return self._version if self._version is not False else None


def test_the_pinned_board_passes_and_is_recorded():
    board = _Board()
    record = airspeed.pin_board(board, "pc-sitl", {"path": "a", "sha256": SHA})
    assert record["params"] == GOOD and record["commit"] == COMMIT and record["problems"] == []
    assert ("cmd", (airspeed.mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0)) in board.log


@pytest.mark.parametrize("params, binary, version, needle", [
    ({**GOOD, "ARSPD_USE": 0.0}, SHA, None, "ARSPD_USE"),
    ({**GOOD, "ARSPD_TYPE": 1.0}, SHA, None, "ARSPD_TYPE"),
    ({"ARSPD_TYPE": 100.0, "ARSPD_USE": 1.0}, SHA, None, "ARSPD_PRIMARY"),
    (GOOD, "0" * 64, None, "not a pinned build"),
    (GOOD, None, None, "not a pinned build"),
    (GOOD, SHA, False, "no AUTOPILOT_VERSION"),
    (GOOD, SHA, _version("4c23c0a1"), "4c23c0a1"),
])
def test_a_board_that_is_not_the_pinned_source_is_refused(params, binary, version, needle):
    with pytest.raises(airspeed.AirspeedPinError, match=needle):
        airspeed.pin_board(_Board(params, version), "pc-sitl", {"path": "a", "sha256": binary})


def test_a_flight_mode_has_no_profile_to_pin():
    with pytest.raises(ValueError, match="simulated pitot profile"):
        airspeed.pin_board(_Board(), "flight", {"path": "a", "sha256": SHA})


def test_the_pins_come_before_takeoff(monkeypatch):
    log: list = []
    monkeypatch.setattr("tools.tuning.sitl_driver.takeoff", lambda board: log.append("takeoff"))
    board = _Board(params={**GOOD, "ARSPD_USE": 0.0}, log=log)
    run = {"mode": "pc-sitl", "sitl_binary": {"path": "a", "sha256": SHA},
           "scenario": {"limits": {"min_height_m": 30.0, "observe_s": 1.0}}}
    with pytest.raises(airspeed.AirspeedPinError):
        approach._fly(board, None, run, None)
    assert "takeoff" not in log and ("param", "ARSPD_TYPE") in log


def _out(tmp_path, *records: str):
    path = tmp_path / "navpy.out"
    path.write_text("\n".join(["noise"] + [f"{NAVPY_AIRSPEED_MARKER} {r}" for r in records]) + "\n", "utf-8")
    return path


def test_an_admitted_source_never_revoked_passes(tmp_path):
    result = airspeed.outcome(_out(tmp_path, '{"event":"claim"}', '{"event":"admitted"}'))
    assert result["problems"] == [] and result["events"] == ["claim", "admitted"]


def test_a_revoke_fails_the_trial(tmp_path):
    result = airspeed.outcome(_out(tmp_path, '{"event":"admitted"}', '{"event":"revoked","reason":"sensor_unhealthy"}'))
    assert result["problems"] == ["airspeed source revoked: sensor_unhealthy"]


@pytest.mark.parametrize("records", [(), ('{"event":"claim"}',), ('broken',)])
def test_a_source_never_admitted_fails_the_trial(tmp_path, records):
    assert "never admitted" in airspeed.outcome(_out(tmp_path, *records))["problems"][-1]
    assert "never admitted" in airspeed.outcome(tmp_path / "missing.out")["problems"][-1]


def test_the_reported_commit_is_the_first_eight_characters():
    assert airspeed.reported_commit(SimpleNamespace(flight_custom_version=[ord(c) for c in "28fbab23"])) == "28fbab23"
    assert airspeed.reported_commit(SimpleNamespace(flight_custom_version="28fbab23")) == "28fbab23"
