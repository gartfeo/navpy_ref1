"""Vehicle arguments (tools/tuning/hitl.py): the SITL path stays as before, the board path's
argument rules, and the parameter files a board gets."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from tools.tuning import hitl, sitl_driver
from tools.tuning.board import Board
from tests.tools.tuning.hitl_data import SYNTHETIC_AIRCRAFT_PARAMS, SYNTHETIC_LAUNCH_PARAMS, write_hitl_data
from tools.tuning.hitl import (BOARD_PARAMS, BRIDGE_PARAMS, add_vehicle_args, board_param_files, hitl_data_files,
                               open_vehicle, record_run, run_file, vehicle_arg_errors, write_run_params)
from tools.tuning.hitl_params import merge_param_files, read_param_file
from tools.tuning.sitl_driver import FTUNE_SCRIPT
from tools.tuning.private_data import PRIVATE_DIR_ENV
from tools.tuning.sitl_sim import (DEFAULT_BINARY, DEFAULT_PARAMS, HIL_SITL_PARAMS, HITL_PARAMS, HOME_CMAC,
                                   PERON_LIKE_PARAMS, SITL_DEFAULTS, Sitl, Transmitter)
from tools.tuning.vehicle import OverrideTransmitter

BRIDGE = ("wsl.exe --cd {repo_wsl} -- python3 -m tools.tuning.hitl_bridge --device tcpin:{port} --origin-lat {lat} "
          "--origin-lon {lon} --origin-alt {alt} --heading {hdg} --log-csv {link_csv_wsl}")
RELAY = "{python} -m tools.tuning.hitl_bridge.relay --device COM11 --bridge 127.0.0.1:{port}"


def _args(*argv: str) -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--workdir", default="run")
    add_vehicle_args(p)
    return p.parse_args(list(argv))


def test_sitl_path_builds_the_same_process_as_before(tmp_path):
    extra = tmp_path / "x_extra.parm"
    sitl = open_vehicle(_args("--workdir", str(tmp_path / "x"), "--speedup", "2"), [extra], [], DEFAULT_BINARY)
    assert isinstance(sitl, Sitl) and sitl.rc == "udp" and sitl.speedup == 2
    assert sitl.command() == [str(DEFAULT_BINARY), "--model", "plane-elevon", "--speedup", "2", "-I", "240",
                              "--home", HOME_CMAC, "--defaults",
                              ",".join(str(p) for p in DEFAULT_PARAMS + [PERON_LIKE_PARAMS, extra])]
    assert SITL_DEFAULTS == DEFAULT_PARAMS + [PERON_LIKE_PARAMS]


def test_sitl_rc_over_mavlink_also_loads_the_override_settings(tmp_path):
    sitl = open_vehicle(_args("--workdir", str(tmp_path), "--rc", "mavlink"), [], [], DEFAULT_BINARY)
    assert sitl.rc == "mavlink" and sitl.params[-1] == HITL_PARAMS
    assert Sitl(tmp_path, rc=None, defaults=[]).command()[-2:] == ["--home", HOME_CMAC]    # no --defaults at all
    with pytest.raises(ValueError):
        Sitl(tmp_path, rc="radio")


def test_transmitter_interface_is_the_same_for_both_rc_sinks():
    for cls in (Transmitter, OverrideTransmitter):
        assert all(callable(getattr(cls, m)) for m in ("start", "set", "get", "stop"))


@pytest.mark.parametrize("argv, flag", [(["--erase-params"], "--erase-params"), (["--bridge", BRIDGE], "--bridge"),
                                        (["--relay", RELAY], "--relay"), (["--bridge-port", "18000"], "--bridge-port"),
                                        (["--baud", "57600"], "--baud"), (["--log-fetch", "none"], "--log-fetch")])
def test_board_options_need_a_board(argv, flag):
    assert vehicle_arg_errors(_args(*argv), []) == [f"{flag} needs --board"]


def test_board_refuses_lua_scripts_speedup_udp_rc_and_a_bad_origin():
    errors = vehicle_arg_errors(_args("--board", "COM15", "--speedup", "2", "--rc", "udp", "--origin", "1,2,3"),
                                [FTUNE_SCRIPT])
    assert len(errors) == 4
    assert "Lua scripts" in errors[0] and "--speedup must be 1" in errors[1] and "UDP" in errors[2]
    assert "lat,lon,alt,hdg" in errors[3]
    with pytest.raises(SystemExit, match="Lua scripts"):
        open_vehicle(_args("--board", "COM15"), [], [FTUNE_SCRIPT], DEFAULT_BINARY)


def test_hitl_needs_both_commands_and_valid_templates():
    board = ("--board", "COM15", "--erase-params")
    assert vehicle_arg_errors(_args(*board, "--bridge", BRIDGE, "--relay", RELAY), []) == []
    # Changed: a bridge without a relay is the direct Windows bridge; it must then open the board's port itself
    (error,) = vehicle_arg_errors(_args(*board, "--bridge", BRIDGE), [])
    assert "--bridge without --relay must open the board's HIL port itself" in error
    direct = BRIDGE.replace("--device tcpin:{port}", "--device COM11 --plant package")
    assert vehicle_arg_errors(_args(*board, "--bridge", direct), []) == []
    assert vehicle_arg_errors(_args(*board, "--relay", RELAY), []) == ["--relay needs --bridge"]
    assert vehicle_arg_errors(_args(*board, "--bridge-port", "18000"), []) == ["--bridge-port needs --bridge and --relay"]
    errors = vehicle_arg_errors(_args(*board, "--bridge", "bridge --device tcpin:{port} --origin-lat {lat}",
                                      "--relay", RELAY), [])
    assert any("SIM_OPOS_" in e for e in errors) and any("{link_csv_wsl}" in e for e in errors)


def test_hitl_on_a_serial_board_needs_the_erase_but_sitl_standing_in_does_not():
    hitl_args = ("--bridge", BRIDGE, "--relay", RELAY)
    (error,) = vehicle_arg_errors(_args("--board", "COM15", *hitl_args), [])
    assert "--bridge on a serial board needs --erase-params" in error
    assert vehicle_arg_errors(_args("--board", "/dev/ttyACM0", *hitl_args), []) == [error]
    assert vehicle_arg_errors(_args("--board", "tcp:127.0.0.1:8160", *hitl_args), []) == []
    assert vehicle_arg_errors(_args("--board", "COM15"), []) == []          # SimOnHW without a bridge: no erase rule


def test_negative_latitude_is_given_with_an_equals_sign():
    assert _args("--origin=-35.36,149.16,584,353").origin == "-35.36,149.16,584,353"


def test_board_gets_sitls_files_in_sitls_order_then_the_starting_and_the_runs_own_files(tmp_path):
    extra, start = tmp_path / "run_extra.parm", tmp_path / "aircraft.parm"
    aircraft, launch = hitl_data_files(None)              # the private folder's (here the synthetic one)
    files = board_param_files([extra], [start], bridge=True, serial_board=True)
    assert [f.path for f in files] == DEFAULT_PARAMS + [PERON_LIKE_PARAMS, BRIDGE_PARAMS, aircraft, BOARD_PARAMS,
                                                        start, launch, extra, HITL_PARAMS]
    assert [f.required for f in files] == [False, False, True, True, True, True, True, True, True, True]
    sitl_standing_in = [f.path for f in board_param_files([extra], [start], bridge=True, serial_board=False)]
    assert {BRIDGE_PARAMS, aircraft, launch} <= set(sitl_standing_in) and BOARD_PARAMS not in sitl_standing_in
    no_bridge = [f.path for f in board_param_files([extra], [], bridge=False, serial_board=True)]
    assert not {BRIDGE_PARAMS, BOARD_PARAMS, aircraft, launch} & set(no_bridge)
    other = tmp_path / "elsewhere"
    given = [f.path for f in board_param_files([], [], bridge=True, serial_board=True, hitl_data=other)]
    assert hitl_data_files(other) == (other / "hitl_aircraft.parm", other / "hitl_launch.parm")
    assert set(hitl_data_files(other)) <= set(given) and not {aircraft, launch} & set(given)


def test_a_bridge_run_needs_the_aircrafts_private_files(tmp_path, monkeypatch):
    """The aircraft's airspeeds, takeoff and launch settings are private (tools/tuning/private_data.py): a
    bridge run without them is refused before anything is written to the board."""
    board = ("--board", "COM15", "--erase-params", "--bridge", BRIDGE.replace("--device tcpin:{port}",
                                                                               "--device COM11 --plant package"))
    assert vehicle_arg_errors(_args(*board), []) == []
    monkeypatch.setenv(PRIVATE_DIR_ENV, str(tmp_path / "missing"))
    errors = vehicle_arg_errors(_args(*board), [])
    assert len(errors) == 2 and all("private data" in e and "--hitl-data" in e for e in errors)
    assert "hitl_aircraft.parm" in errors[0] and "hitl_launch.parm" in errors[1]
    elsewhere = write_hitl_data(tmp_path / "elsewhere") / "hitl"
    assert vehicle_arg_errors(_args(*board, "--hitl-data", str(elsewhere)), []) == []
    assert vehicle_arg_errors(_args("--hitl-data", str(elsewhere)), []) == ["--hitl-data needs --board"]


def test_the_launch_settings_win_over_the_aircraft_file_and_a_run_can_still_change_them(tmp_path):
    """An aircraft's starting file can have a TKOFF_THR_DELAY of 1 (0.1 s, throttle while still on the
    rail); the private launch file comes after it. The private takeoff settings (hitl_aircraft.parm) come
    before it, so an aircraft file that sets them wins; the run's own --param wins over all."""
    aircraft = tmp_path / "aircraft.parm"
    aircraft.write_text("TKOFF_THR_DELAY 1\nTKOFF_THR_MINACC 2\nTKOFF_LVL_PITCH 12\n")
    run_own = tmp_path / "run_extra.parm"
    run_own.write_text("WP_LOITER_RAD 150\n")
    upstream = [p for p in DEFAULT_PARAMS if p.is_file()]     # the ArduPilot tree may be absent here
    files = [f for f in board_param_files([run_own], [aircraft], bridge=True, serial_board=True)
             if f.path not in DEFAULT_PARAMS or f.path in upstream]
    values, required = merge_param_files(files)
    assert (values["TKOFF_THR_DELAY"], values["TKOFF_THR_MINACC"]) == (4, 3.0) == (     # hitl_launch.parm
        SYNTHETIC_LAUNCH_PARAMS["TKOFF_THR_DELAY"], SYNTHETIC_LAUNCH_PARAMS["TKOFF_THR_MINACC"])
    assert SYNTHETIC_AIRCRAFT_PARAMS["TKOFF_LVL_PITCH"] != 12                               # the starting file's wins
    assert (values["TKOFF_LVL_PITCH"], values["TKOFF_THR_MAX"], values["TKOFF_THR_SLEW"]) == (12, 0, 0)
    assert {"TKOFF_THR_DELAY", "TKOFF_THR_MINACC", "TKOFF_THR_SLEW"} <= required
    run_own.write_text("TKOFF_THR_DELAY 1\n")
    assert merge_param_files(files)[0]["TKOFF_THR_DELAY"] == 1


def test_hitl_parameter_files_hold_the_pwm_range_and_board_only_settings():
    bridge, board = read_param_file(BRIDGE_PARAMS), read_param_file(BOARD_PARAMS)
    # the bridge normalises 1000-2000 us (hitl_bridge/physics.py)
    assert all(bridge[f"SERVO{i}_{end}"] == v for i in (1, 2, 3) for end, v in (("MIN", 1000), ("MAX", 2000)))
    # the aircraft's airspeeds, takeoff and launch settings are private (hitl_aircraft.parm, hitl_launch.parm;
    # their values: test_hitl_private_data.py), never in the repository's file
    assert not {n for n in bridge if n.startswith(("AIRSPEED_", "TKOFF_"))}
    assert "SERIAL8_PROTOCOL" not in bridge                          # SITL has no SERIAL8
    # Changed behaviour (HIL link contract v2): the HIL backend opens the port whose protocol is 100
    # (SIM HIL), so SERIAL8 = USB OTG2 gets 100 (v1 used a compiled-in port and set -1); the rest
    # are the fork's HIL.parm defaults, set because defaults never replace saved values
    calibration = {f"INS_ACC{i}{kind}_{axis}": value for i in ("", "2") for kind, value in (("OFFS", 0.001),
                   ("SCAL", 1.001)) for axis in "XYZ"}
    assert board == {"GPS1_TYPE": 100, "ARSPD_TYPE": 100, "BATT_MONITOR": 0, "BRD_HEAT_TARG": -1, "SIM_OH_MASK": 0,
                     "SERIAL8_PROTOCOL": 100, **calibration, "BRD_SAFETY_DEFLT": 0}
    assert "SIM_OH_MASK" in BOARD_PARAMS.read_text()                 # the safety-switch comment names its gate


def test_open_vehicle_with_a_board_gives_a_real_time_board(tmp_path):
    start = tmp_path / "start.parm"
    start.write_text("RLL_RATE_P 0.07\n")
    board = open_vehicle(_args("--workdir", str(tmp_path / "b"), "--board", "tcp:127.0.0.1:8160", "--erase-params",
                               "--bridge", BRIDGE, "--relay", RELAY, "--bridge-port", "18000", "--log-fetch", "none",
                               "--params-file", str(start)),
                         [], [], DEFAULT_BINARY)
    assert isinstance(board, Board) and board.speedup == 1 and board.erase and not board.log_fetch
    assert board.device == "tcp:127.0.0.1:8160" and board.origin == HOME_CMAC and Path(board.workdir) == tmp_path / "b"
    assert board.hitl.port == 18000 and "tcpin:18000 --origin-lat -35.363261" in board.hitl.bridge_cmd
    paths = [f.path for f in board.param_files]
    assert BRIDGE_PARAMS in paths and BOARD_PARAMS not in paths and start in paths
    plain = open_vehicle(_args("--workdir", str(tmp_path / "c"), "--board", "COM15"), [], [], DEFAULT_BINARY)
    assert plain.hitl is None and BRIDGE_PARAMS not in [f.path for f in plain.param_files]


def test_params_files_go_before_the_runs_own_file_on_sitl_and_must_exist(tmp_path):
    first, second, extra = tmp_path / "a.parm", tmp_path / "b.parm", tmp_path / "x_extra.parm"
    first.write_text("RLL_RATE_P 0.07\n")
    second.write_text("RLL_RATE_P 0.08\n")
    sitl = open_vehicle(_args("--workdir", str(tmp_path / "x"), "--params-file", str(first), "--params-file",
                              str(second)), [extra], [], DEFAULT_BINARY)
    assert sitl.params == SITL_DEFAULTS + [first, second, extra]
    (error,) = vehicle_arg_errors(_args("--params-file", str(tmp_path / "gone.parm")), [])
    assert "gone.parm: no such file" in error


def test_hitl_sitl_model_serves_its_link_on_serial5_with_protocol_100_at_the_origin(tmp_path):
    # Changed behaviour (HIL link contract v2): SITL manages SERIAL0-7 only and the backend opens
    # the port with protocol 100, so the link moved from --serial8 to SERIAL5 (plane-hil.parm's)
    origin = "40.2,44.4,1300,270"
    sitl = Sitl(tmp_path, model="hil", home=origin, rc=None, defaults=[])
    assert sitl.hil_port == sitl.mavlink_port + 8 == 8168
    assert sitl.command()[-7:-2] == ["-I", "240", "--home", origin, "--serial5=tcp:8168"]
    assert sitl.params == [HIL_SITL_PARAMS] and read_param_file(HIL_SITL_PARAMS) == {"SERIAL5_PROTOCOL": 100}
    plain = Sitl(tmp_path)
    assert plain.hil_port is None and HIL_SITL_PARAMS not in plain.params
    assert not any(a.startswith("--serial") for a in plain.command())


def test_sitl_backing_runs_the_requested_model_at_the_runs_origin(tmp_path, monkeypatch, capsys):
    built = {}

    class FakeSitl(Sitl):
        def start(self) -> "FakeSitl":
            built["command"] = self.command()
            self.mav = type("Conn", (), {"close": lambda conn: None})()
            self.proc = type("Proc", (), {"wait": lambda proc: 0})()
            return self

        def stop(self) -> None:
            built["stopped"] = True

    monkeypatch.setattr(hitl, "Sitl", FakeSitl)
    args = argparse.Namespace(workdir=str(tmp_path), instance=240, binary="arduplane", model="hil",
                              origin="40.2,44.4,1300,270", defaults=None)
    hitl.sitl_backing(args)
    assert built["stopped"] and built["command"][built["command"].index("--home") + 1] == "40.2,44.4,1300,270"
    assert "--serial5=tcp:8168" in built["command"]
    out = capsys.readouterr().out
    assert "--board tcp:127.0.0.1:8160 --origin=40.2,44.4,1300,270" in out and "--device tcp:127.0.0.1:8168" in out
    with pytest.raises(ValueError):
        hitl.sitl_backing(argparse.Namespace(**{**vars(args), "origin": "1,2,3"}))


# SITL's physics rate (fork libraries/SITL/SITL.cpp SIM_RATE_HZ_DEFAULT): SITL generates its
# simulated IMU samples and serves its serial ports once per physics frame.
SITL_PHYSICS_RATE_HZ = 1200


def test_sitl_backing_keeps_sitls_physics_rate_after_a_boards_hwdef_defaults(tmp_path, monkeypatch):
    """A board hwdef's defaults.parm sets SIM_RATE_HZ to its 200 Hz main loop. In SITL that
    gave a 200 Hz gyro (arming refused) and HIL replies a frame late (50 % stale), so the
    stand-in loads sitl_backing.parm after every --defaults file, restoring SITL's rate."""
    built = {}

    class FakeSitl(Sitl):
        def start(self) -> "FakeSitl":
            built["defaults"] = self.command()[self.command().index("--defaults") + 1].split(",")
            self.mav = type("Conn", (), {"close": lambda conn: None})()
            self.proc = type("Proc", (), {"wait": lambda proc: 0})()
            return self

        def stop(self) -> None:
            pass

    hwdef = tmp_path / "defaults.parm"
    hwdef.write_text("SIM_RATE_HZ 200\nSCHED_LOOP_RATE 200\n")
    monkeypatch.setattr(hitl, "Sitl", FakeSitl)
    for given in (None, [str(hwdef)]):
        args = argparse.Namespace(workdir=str(tmp_path / "w"), instance=240, binary="arduplane", model="hil",
                                  origin="40.2,44.4,1300,270", defaults=given)
        hitl.sitl_backing(args)
        # SITL: later files win; the hil model's link port file (SERIAL5_PROTOCOL 100) comes last
        assert built["defaults"] == (given or []) + [str(hitl.SITL_BACKING_PARAMS), str(HIL_SITL_PARAMS)]
    merged = {}
    for path in built["defaults"]:
        merged.update(read_param_file(Path(path)))
    assert merged["SIM_RATE_HZ"] == SITL_PHYSICS_RATE_HZ and merged["SCHED_LOOP_RATE"] == 200


def test_run_record_gets_the_board_report_and_fails_on_a_failed_link(tmp_path, capsys):
    out = tmp_path / "run.json"
    assert record_run({"logs": []}, {}, out) == 0 and json.loads(out.read_text()) == {"logs": []}      # SITL
    assert record_run({}, {"device": "COM15"}, out) == 0                                                # no HITL
    good = {"link_health": {"ok": True, "problems": []}}
    assert record_run({}, good, out) == 0 and json.loads(out.read_text())["vehicle"] == good
    bad = {"link_health": {"ok": False, "problems": ["00000002.BIN: 9 of 100 frames stale"]}}
    assert record_run({}, bad, out) == 1 and json.loads(out.read_text())["vehicle"] == bad
    assert "9 of 100 frames stale" in capsys.readouterr().out


class _RecordedVehicle:
    """What fly_and_record uses of a Sitl or Board: start, stop, messages, logs, report."""

    def __init__(self, report: dict) -> None:
        self.events: list[str] = []
        self.messages = ["FTUN: start roll", "PreArm: Throttle below failsafe", "Roll: Finished"]
        self._report = report

    def start(self) -> None:
        self.events.append("start")

    def stop(self) -> None:
        self.events.append("stop")

    def logs(self) -> list[Path]:
        return [Path("logs") / "00000001.BIN"]

    def report(self) -> dict:
        self.events.append("report")
        return self._report


def test_a_failed_flight_still_writes_its_record_with_the_boards_report_then_raises(tmp_path):
    # rail_sign1 and the rail launch of 2026-10-12: the takeoff raised, so the run's link health and
    # Mission Planner counts were never written although the board had stopped and its log was downloaded
    out = tmp_path / "run.json"
    vehicle = _RecordedVehicle({"link_health": {"ok": False, "problems": ["age"]}, "mp_mirror": {"port": 9640}})

    def takeoff_fails() -> None:
        raise RuntimeError("takeoff did not reach TKOFF_ALT 120 m")
    with pytest.raises(RuntimeError, match="TKOFF_ALT 120 m"):
        hitl.fly_and_record(vehicle, takeoff_fails, {"before": 1}, out, kept_texts=("FTUN",))
    written = json.loads(out.read_text())
    assert vehicle.events == ["start", "stop", "report"]
    assert written["error"] == "RuntimeError: takeoff did not reach TKOFF_ALT 120 m" and written["before"] == 1
    assert written["messages"] == ["FTUN: start roll"] and written["logs"] == [str(Path("logs") / "00000001.BIN")]
    assert written["vehicle"]["mp_mirror"] == {"port": 9640}


def test_a_completed_flight_returns_the_link_gate_and_keeps_only_the_chosen_texts(tmp_path):
    out = tmp_path / "run.json"
    good = _RecordedVehicle({"link_health": {"ok": True, "problems": []}})
    assert hitl.fly_and_record(good, lambda: None, {}, out, kept_texts=("FTUN", "Finished")) == 0
    assert json.loads(out.read_text())["messages"] == ["FTUN: start roll", "Roll: Finished"]
    bad = _RecordedVehicle({"link_health": {"ok": False, "problems": ["9 of 100 frames stale"]}})
    assert hitl.fly_and_record(bad, lambda: None, {}, out) == 1 and "messages" not in json.loads(out.read_text())


def test_port_check_writes_the_record_of_a_takeoff_that_failed(tmp_path, monkeypatch):
    vehicle = _RecordedVehicle({"link_health": {"ok": False, "problems": ["age"]}})
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: vehicle)

    def no_climb(sitl: object) -> None:
        raise RuntimeError("takeoff did not reach TKOFF_ALT 120 m")
    monkeypatch.setattr(sitl_driver, "takeoff_to_fbwa", no_climb)
    with pytest.raises(RuntimeError):
        sitl_driver.port_check(argparse.Namespace(workdir=str(tmp_path / "sign")))
    assert json.loads((tmp_path / "sign.json").read_text())["error"].endswith("TKOFF_ALT 120 m")


def test_port_check_flies_its_doublets_at_the_cruise_throttle_and_trim(tmp_path, monkeypatch):
    vehicle = _RecordedVehicle({"link_health": {"ok": True, "problems": []}})
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: vehicle)
    order: list[str] = []
    monkeypatch.setattr(sitl_driver, "takeoff_to_fbwa", lambda sitl: order.append("takeoff"))
    monkeypatch.setattr(sitl_driver, "pilot_throttle", lambda sitl: order.append("cruise") or {"cruise_pct": 57.0})

    def context(name: str) -> type:
        class Recorded:
            def __init__(self, *args: object) -> None:
                order.append(f"{name} {args[1:]}")

            def __enter__(self) -> None:
                order.append(f"{name} in")

            def __exit__(self, *exc: object) -> None:
                order.append(f"{name} out")
        return Recorded
    monkeypatch.setattr(sitl_driver, "HeightFloor", context("floor"))
    monkeypatch.setattr(sitl_driver, "CruisePitchTrim", context("trim"))
    monkeypatch.setattr(sitl_driver, "stick_sequence", lambda *args: order.append("stick"))
    monkeypatch.setattr(vehicle, "fly_for", lambda s: None, raising=False)
    assert sitl_driver.port_check(argparse.Namespace(workdir=str(tmp_path / "sign"), min_height=30.0)) == 0
    record = json.loads((tmp_path / "sign.json").read_text())
    # the trim gets the cruise pilot_throttle measured and the run's record; the doublets fly inside floor and trim
    run = "{'pilot_throttle': {'cruise_pct': 57.0}}"         # the run's record, filled as it goes
    assert order == ["takeoff", "cruise", f"floor (30.0, {run})", "floor in", f"trim ({{'cruise_pct': 57.0}}, {run})",
                     "trim in"] + ["stick"] * 7 + ["trim out", "floor out"]
    assert record["pilot_throttle"] == {"cruise_pct": 57.0}


def test_the_runs_own_files_sit_next_to_the_run_folder(tmp_path):
    workdir = str(tmp_path / "runs" / "tune1")
    assert run_file(workdir, ".json") == tmp_path / "runs" / "tune1.json"
    extra = write_run_params(workdir, {"FTUN_ENABLE": 1, "WP_LOITER_RAD": 150})
    assert extra == tmp_path / "runs" / "tune1_extra.parm"
    assert read_param_file(extra) == {"FTUN_ENABLE": 1.0, "WP_LOITER_RAD": 150.0}
