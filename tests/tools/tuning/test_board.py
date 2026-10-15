"""Board (tools/tuning/board.py) against a fake flight controller: the safety gates, backup
before erase, parameter load with reboots, the HITL processes, forced disarm and log download,
and the restore of a backup. No board, no SITL."""
from __future__ import annotations

from pathlib import Path

import pytest

from tests.tools.tuning.fake_board import FakeBoard, FakeLog, f32
from tools.tuning import board as board_module
from tools.tuning import hitl_boot
from tools.tuning import hitl_params
from tools.tuning.board import BACKUP_NAME, FORCE_DISARM_MAGIC, Board
from tools.tuning.board_backup import restore
from tests.tools.tuning.hitl_data import SYNTHETIC_AIRCRAFT_PARAMS, SYNTHETIC_LAUNCH_PARAMS
from tools.tuning.hitl import BOARD_PARAMS, BRIDGE_PARAMS, hitl_data_files
from tools.tuning.hitl_params import ParamFile, Refused, read_param_file
from tools.tuning.sitl_sim import HITL_PARAMS

ORIGIN = "40.2,44.4,1300,270"
SIM_DEFAULTS = {
    "SIM_OH_MASK": 0, "SIM_RATE_HZ": 200, "SCHED_LOOP_RATE": 50, "SIM_OPOS_LAT": -35.363261,
    "SIM_OPOS_LNG": 149.16523, "SIM_OPOS_ALT": 584, "SIM_OPOS_HDG": 353, "SYSID_MYGCS": 255,
    "LOG_FILE_DSRMROT": 0, "RLL_RATE_P": 0.08, "RC9_OPTION": 0, "FTUN_ENABLE": 0, "FTUN_AXES": 3,
    "FORMAT_VERSION": 13, "STAT_BOOTCNT": 3,
    # accelerometer calibration as a board saves it before HIL (hitl_board.parm marks it calibrated)
    **{f"INS_ACC{i}{kind}_{axis}": value for i in ("", "2") for kind, value in (("OFFS", 0.0), ("SCAL", 1.0))
       for axis in "XYZ"},
}
BRIDGE_DEFAULTS = {"SERVO1_FUNCTION": 78, "SERVO2_FUNCTION": 77, "SERVO1_REVERSED": 1, "SERVO2_REVERSED": 1,
                   "SERVO1_TRIM": 1500, "SERVO2_TRIM": 1500, "SERIAL8_PROTOCOL": 2, "AIRSPEED_MIN": 10,
                   "AIRSPEED_CRUISE": 22, "AIRSPEED_MAX": 30, "TKOFF_THR_MINACC": 0, "TKOFF_THR_DELAY": 2,
                   "TKOFF_LVL_PITCH": 10, "TKOFF_THR_MAX": 80, "TKOFF_THR_SLEW": -1,
                   **{f"SERVO{i}_{end}": v for i in (1, 2, 3) for end, v in (("MIN", 1100), ("MAX", 1900))},
                   # a stock board's saved flight settings (pixhawk6x backup): real GPS, pitot, battery, heater
                   "GPS1_TYPE": 1, "ARSPD_TYPE": 1, "BATT_MONITOR": 21, "BRD_HEAT_TARG": 45, "BRD_SAFETY_DEFLT": 1}
HIDDEN = {"FTUN_AXES": "FTUN_ENABLE"}         # shown at the next boot once FTUN_ENABLE is set


@pytest.fixture
def files(tmp_path: Path) -> list[ParamFile]:
    upstream = tmp_path / "plane.parm"
    upstream.write_text("EK2_ENABLE 1\nRLL_RATE_P 0.3\n")
    peron = tmp_path / "peron_like.parm"
    peron.write_text("SCHED_LOOP_RATE 200\n")
    extra = tmp_path / "run_extra.parm"
    extra.write_text("FTUN_ENABLE 1\nFTUN_AXES 2\nRC9_OPTION 240\n")
    return [ParamFile(upstream, False), ParamFile(peron, True), ParamFile(extra, True), ParamFile(HITL_PARAMS, True)]


def _sim_board(**overrides: float) -> FakeBoard:
    board = FakeBoard({**SIM_DEFAULTS, **BRIDGE_DEFAULTS, **overrides}, stored={"RLL_RATE_P": 0.13}, hidden=HIDDEN)
    board.logs = [FakeLog(b"before the run" * 10, time_utc=100)]
    return board


def _runner(board: FakeBoard, tmp_path: Path, files: list[ParamFile], **kwargs: object) -> Board:
    return Board(kwargs.pop("device", "COM15"), tmp_path / "run", files, origin=ORIGIN,
                 connect=board.connection, **kwargs)


def _writes(board: FakeBoard) -> list[str]:
    return [e for e in board.events if e.startswith(("set:", "erase", "reboot", "arm:"))]


# ---- a whole run ----------------------------------------------------------------------
def test_start_backs_up_then_erases_loads_and_verifies_then_stop_disarms_and_downloads(tmp_path, files):
    board = _sim_board()
    runner = _runner(board, tmp_path, files, erase=True)
    runner.start()
    try:
        backup = read_param_file(tmp_path / "run" / BACKUP_NAME)
        assert backup["RLL_RATE_P"] == f32(0.13) and len(backup) == len(board.defaults) - 1   # FTUN_AXES hidden
        assert board.events.index("erase") < min(i for i, e in enumerate(board.events) if e.startswith("set:"))
        assert board.ram["RLL_RATE_P"] == f32(0.3)                     # the aircraft's value was erased
        assert board.ram["FTUN_AXES"] == 2                             # revealed by a boot, then set
        assert board.ram["SYSID_MYGCS"] == 250 and board.ram["SIM_OPOS_LAT"] == f32(40.2)
        assert runner.record["erased"] and runner.record["absent_optional"] == ["EK2_ENABLE"]
        assert runner.record["load_rounds"] == 2
        assert board.overrides, "the RC overrides run once start() returns"
        assert runner.speedup == 1
        board.armed = True                                              # the flight
        board.logs.append(FakeLog(bytes(range(256)) * 9, time_utc=200))
    finally:
        runner.stop()
    assert f"arm:0:{FORCE_DISARM_MAGIC}" in board.events and not board.armed
    assert (tmp_path / "run" / "logs" / "00000002.BIN").read_bytes() == bytes(range(256)) * 9
    assert runner.logs() == [tmp_path / "run" / "logs" / "00000002.BIN"]
    assert board.events[-1] == "log_end"
    report = runner.report()
    assert report["log_entries"] == ["00000002.BIN"]
    assert report["timing"]["00000002.BIN"]["FTPR"] == {"records": 0}     # not a DataFlash log
    assert not runner.tx._thread.is_alive() and runner.mav is None     # RC stopped, link closed


def test_without_the_erase_flag_nothing_is_erased(tmp_path, files):
    board = _sim_board()
    runner = _runner(board, tmp_path, files)
    runner.start()
    runner.stop()
    assert "erase" not in board.events and not runner.record["erased"]
    assert board.ram["SCHED_LOOP_RATE"] == 200


def test_log_fetch_none_lists_the_new_log_for_the_sd_card(tmp_path, files):
    board = _sim_board()
    runner = _runner(board, tmp_path, files, log_fetch=False)
    runner.start()
    board.logs.append(FakeLog(b"n" * 500, time_utc=300))
    runner.stop()
    assert runner.record["logs_on_card"] == {2: {"bytes": 500, "time_utc": 300}}
    assert runner.logs() == [] and "log_end" not in board.events


# ---- gates --------------------------------------------------------------------------------
def test_refuses_stock_firmware_before_writing_anything(tmp_path, files):
    board = FakeBoard({k: v for k, v in SIM_DEFAULTS.items() if not k.startswith("SIM_")})
    runner = _runner(board, tmp_path, files, erase=True)
    with pytest.raises(Refused, match="not ArduPilot simulation firmware"):
        runner.start()
    assert _writes(board) == []
    assert (tmp_path / "run" / BACKUP_NAME).exists()        # the read-only backup is still kept


def test_refuses_real_outputs(tmp_path, files):
    board = _sim_board(SIM_OH_MASK=1)
    with pytest.raises(Refused, match="real servo outputs"):
        _runner(board, tmp_path, files, erase=True).start()
    assert _writes(board) == []


def test_refuses_an_armed_vehicle_and_never_disarms_it(tmp_path, files):
    board = _sim_board()
    board.armed = True
    with pytest.raises(Refused, match="armed"):
        _runner(board, tmp_path, files, erase=True).start()
    assert _writes(board) == [] and board.armed


def test_refuses_a_run_folder_that_holds_files(tmp_path, files):
    (tmp_path / "run").mkdir()
    (tmp_path / "run" / BACKUP_NAME).write_text("RLL_RATE_P,0.13\n")
    board = _sim_board()
    with pytest.raises(Refused, match="never wiped"):
        _runner(board, tmp_path, files, erase=True).start()
    assert board.events == []


def test_no_erase_without_a_complete_backup(tmp_path, files, monkeypatch):
    monkeypatch.setattr(hitl_params, "PARAM_QUIET_S", 0.01)
    monkeypatch.setattr(hitl_params, "PARAM_LIST_TIMEOUT_S", 0.2)
    board = _sim_board()
    board.never_param_indices = {3}
    with pytest.raises(RuntimeError, match="parameter list incomplete"):
        _runner(board, tmp_path, files, erase=True).start()
    assert _writes(board) == [] and not (tmp_path / "run" / BACKUP_NAME).exists()


def test_a_missing_parameter_file_stops_the_run_before_the_board_is_touched(tmp_path, files):
    board = _sim_board()
    gone = ParamFile(tmp_path / "ardupilot" / "plane.parm", False)     # ARDUPILOT_ROOT not set on Windows
    with pytest.raises(Refused, match=r"plane.parm not found; on Windows set ARDUPILOT_ROOT .*wsl\.localhost"):
        _runner(board, tmp_path, [gone] + files, erase=True).start()
    assert board.events == [] and not (tmp_path / "run").exists()


def test_a_malformed_parameter_file_stops_the_run_before_the_erase(tmp_path, files):
    bad = tmp_path / "bad.parm"
    bad.write_text("RLL_RATE_P 0.1\nTKOFF_ALT fifty\n")
    board = _sim_board()
    with pytest.raises(ValueError, match="bad.parm:2"):
        _runner(board, tmp_path, files + [ParamFile(bad, True)], erase=True).start()
    assert board.events == []


def test_refuses_when_a_run_parameter_is_missing(tmp_path, files):
    board = _sim_board()
    del board.defaults["FTUN_AXES"], board.defaults["FTUN_ENABLE"]
    board._boot()
    with pytest.raises(Refused, match=r"missing \['FTUN_AXES', 'FTUN_ENABLE'\]"):
        _runner(board, tmp_path, files).start()


def test_onboard_model_on_serial_needs_its_rate_equal_to_the_loop_rate(tmp_path, files):
    with pytest.raises(Refused, match="real time"):
        _runner(_sim_board(SIM_RATE_HZ=400), tmp_path, files).start()
    runner = _runner(_sim_board(SIM_RATE_HZ=400), tmp_path / "tcp", files, device="tcp:127.0.0.1:8160")
    runner.start()                     # SITL behind TCP keeps its own lockstep rate
    runner.stop()


def test_reboot_is_refused_while_armed(tmp_path, files):
    board = _sim_board()
    runner = _runner(board, tmp_path, files)
    runner.connect()
    board.armed = True
    with pytest.raises(Refused, match="not rebooting"):
        runner.reboot()
    assert "reboot" not in board.events


def test_a_reconnect_must_find_a_boot_newer_than_the_reboot_command(tmp_path, files, monkeypatch):
    monkeypatch.setattr(board_module, "CONNECT_TIMEOUT_S", 0.3)
    board = _sim_board()
    runner = _runner(board, tmp_path, files)
    runner.connect()
    board.ignore_reboot = True
    with pytest.raises(RuntimeError, match="no fresh boot"):
        runner.reboot()                # the old instance answers: it never rebooted


def test_a_usb_port_that_vanishes_at_the_reboot_is_a_reboot_not_an_error(tmp_path, files):
    board = _sim_board()
    board.vanish_on_reboot = True      # the Pixhawk's COM port goes away before the ack is read
    runner = _runner(board, tmp_path, files, erase=True)
    runner.start()
    runner.stop()
    assert runner.record["erased"] and board.boots >= 2
    assert board.ram["SCHED_LOOP_RATE"] == 200


# ---- HITL processes -----------------------------------------------------------------------
def _bridge_files(files: list[ParamFile]) -> list[ParamFile]:
    aircraft, launch = hitl_data_files(None)              # the synthetic private files (conftest.py)
    hitl_files = [ParamFile(BRIDGE_PARAMS, True), ParamFile(aircraft, True), ParamFile(BOARD_PARAMS, True),
                  ParamFile(launch, True)]
    return files[:2] + hitl_files + files[2:]


class FakeHitl:
    """Records when the board's run starts and stops the bridge and relay (hitl_procs.py)."""

    def __init__(self, board: FakeBoard, link_csv: Path | None = None) -> None:
        self.board = board
        self.link_csv = link_csv or Path("no_link.csv")
        self.started_at: tuple[int, int] | None = None     # (boots, events) when start() ran
        self.stopped_at: int | None = None
        self.ended: str | None = None

    def start(self) -> dict:
        self.started_at = (self.board.boots, len(self.board.events))
        self.on_start()
        return {"port": 18570}

    def on_start(self) -> None:
        """The board's next boot finds the bridge answering before its EKF starts."""
        self.board.boot_texts.insert(0, STATE_FIRST)

    def check(self) -> None:
        if self.ended:
            raise RuntimeError(f"the {self.ended} exited with code 1; see {self.ended}_stdout.txt")

    def stop(self) -> dict:
        self.stopped_at = len(self.board.events)
        return {"relay_exit": 1, "bridge_exit": 0}


STATE_FIRST = ["Ground start", "HIL: waiting for simulator on SERIAL8", "HIL: first state received",
               "EKF3 IMU0 initialised", "EKF3 IMU1 initialised"]
EKF_FIRST = ["Ground start", "HIL: waiting for simulator on SERIAL8", "EKF3 IMU0 initialised",
             "EKF3 IMU1 initialised", "HIL: first state received"]


def test_hitl_starts_before_a_last_reboot_so_the_ekf_starts_on_the_bridges_state(tmp_path, files):
    # Changed behaviour: the HITL processes started after the last reboot, so the EKF aligned on the backend's
    # level placeholder and then took the jump to the rail as gyro bias (sign_check2: 5 deg pitch error)
    board = _sim_board(SIM_RATE_HZ=400)         # no onboard model with the bridge: no rate gate
    hitl = FakeHitl(board)
    runner = _runner(board, tmp_path, _bridge_files(files), erase=True, hitl=hitl)
    runner.start()
    boots, events = hitl.started_at
    assert board.events[events:].count("reboot") == 1 and board.boots == boots + 1
    assert runner.record["hil_boot_attempts"] == 1
    # SERIAL8 (USB OTG2) carries the HIL link: SIM HIL protocol 100 (contract v2; v1 set -1)
    assert (board.ram["SERVO1_FUNCTION"], board.ram["SERVO2_FUNCTION"], board.ram["SERIAL8_PROTOCOL"]) == (77, 78, 100)
    assert [board.ram[f"SERVO{i}_{end}"] for i in (1, 2, 3) for end in ("MIN", "MAX")] == [1000, 2000] * 3
    assert (board.ram["GPS1_TYPE"], board.ram["ARSPD_TYPE"], board.ram["BRD_SAFETY_DEFLT"]) == (100, 100, 0)
    # the aircraft's launch and airspeeds from its private files (here the synthetic ones)
    assert {n: board.ram[n] for n in SYNTHETIC_LAUNCH_PARAMS} == SYNTHETIC_LAUNCH_PARAMS
    assert {n: board.ram[n] for n in SYNTHETIC_AIRCRAFT_PARAMS} == SYNTHETIC_AIRCRAFT_PARAMS
    board.armed = True
    runner.stop()
    assert board.events.index(f"arm:0:{FORCE_DISARM_MAGIC}") < hitl.stopped_at
    assert runner.record["hitl"] == {"port": 18570, "relay_exit": 1, "bridge_exit": 0}


def test_a_boot_whose_ekf_started_before_the_first_state_is_rebooted_and_three_such_boots_fail(tmp_path, files):
    board = _sim_board()
    hitl = FakeHitl(board)
    hitl.on_start = lambda: board.boot_texts.extend([EKF_FIRST, STATE_FIRST])
    runner = _runner(board, tmp_path, _bridge_files(files), hitl=hitl)
    runner.start()
    assert runner.record["hil_boot_attempts"] == 2
    board = _sim_board()
    hitl = FakeHitl(board)
    hitl.on_start = lambda: board.boot_texts.extend([EKF_FIRST] * hitl_boot.HIL_BOOT_ATTEMPTS)
    with pytest.raises(RuntimeError, match="EKF initialised before the first HIL state"):
        _runner(board, tmp_path / "again", _bridge_files(files), hitl=hitl).start()
    assert hitl.stopped_at is not None and not board.armed


def test_a_hitl_process_that_dies_in_flight_ends_the_run(tmp_path, files):
    board = _sim_board()
    hitl = FakeHitl(board)
    runner = _runner(board, tmp_path, _bridge_files(files), hitl=hitl)
    runner.start()
    hitl.ended = "relay"
    with pytest.raises(RuntimeError, match="relay exited"):
        runner.pump()
    runner.stop()
    assert hitl.stopped_at is not None and not board.armed


def _link_csv(path: Path, rows: list[tuple[str, int]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("frame,event,armed\n" + "".join(f"{i},{e},{a}\n" for i, (e, a) in enumerate(rows, 1)))
    return path


def test_a_hitl_report_carries_the_link_health_gate(tmp_path, files):
    board = _sim_board()
    csv_path = _link_csv(tmp_path / "link" / "link.csv", [("first", 0), ("step", 1), ("reinit", 1), ("step", 1)])
    runner = _runner(board, tmp_path, _bridge_files(files), hitl=FakeHitl(board, csv_path))
    runner.start()
    board.logs.append(FakeLog(b"no HIL records here" * 20, time_utc=200))
    runner.stop()
    health = runner.report()["link_health"]
    assert not health["ok"] and health["bridge"]["resets_after_armed"] == 1
    assert any("0 HIL records" in p for p in health["problems"])        # not a DataFlash log
    assert "link_health" not in _runner(_sim_board(), tmp_path / "plain", files).report()   # no HITL, no gate


def test_a_failed_start_still_stops_the_hitl_processes(tmp_path, files):
    board = _sim_board(SIM_OH_MASK=2)
    hitl = FakeHitl(board)
    with pytest.raises(Refused):
        _runner(board, tmp_path, _bridge_files(files), hitl=hitl).start()
    assert hitl.started_at is None and hitl.stopped_at is not None


# ---- restore ------------------------------------------------------------------------------
def test_restore_writes_the_backup_back_except_firmware_owned_values(tmp_path):
    board = FakeBoard({"RLL_RATE_P": 0.3, "FTUN_ENABLE": 0, "FTUN_AXES": 3, "FORMAT_VERSION": 13,
                       "STAT_BOOTCNT": 3}, hidden=HIDDEN)
    backup = tmp_path / "before.parm"
    backup.write_text("RLL_RATE_P,0.1297145\nFTUN_ENABLE,1\nFTUN_AXES,1\nFORMAT_VERSION,12\n"
                      "STAT_BOOTCNT,99\nGONE_PARAM,1\n")
    result = restore(Board("COM15", tmp_path, [], connect=board.connection), backup)
    assert board.ram["RLL_RATE_P"] == f32(0.1297145) and board.ram["FTUN_AXES"] == 1
    assert board.ram["FORMAT_VERSION"] == 13 and "set:FORMAT_VERSION" not in board.events
    assert result["skipped"] == ["FORMAT_VERSION", "STAT_BOOTCNT"]
    assert result["absent"] == ["GONE_PARAM"] and result["differing"] == []


def test_restore_refuses_an_armed_vehicle(tmp_path):
    board = FakeBoard({"RLL_RATE_P": 0.3}, armed=True)
    backup = tmp_path / "before.parm"
    backup.write_text("RLL_RATE_P,0.13\n")
    with pytest.raises(Refused, match="armed"):
        restore(Board("COM15", tmp_path, [], connect=board.connection), backup)
    assert _writes(board) == []
