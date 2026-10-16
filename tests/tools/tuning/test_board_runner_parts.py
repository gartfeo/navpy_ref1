"""Tests for the board runner's pieces without a board: RC overrides, parameter files and
lists, the safety gates, log download with gaps, the timing summary, the HITL link-health
gate and the catapult takeoff."""
from __future__ import annotations

import math
import struct
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tests.tools.tuning.fake_board import FakeBoard, FakeConn, FakeLog, Msg, f32
from tools.tuning import hitl_gate, hitl_logs, hitl_params, sitl_driver
from tools.tuning.hitl_bridge.frame_loop import LOG_COLUMNS as BRIDGE_LOG_COLUMNS
from tools.tuning.hitl_bridge.board_clock import CONFLICT, FIRST, GAP, REINIT, RETRANSMIT, STEP
from tools.tuning.hitl_gate import (BRIDGE_ARMED, BRIDGE_EVENT, HIL_AGE_MAX_US, HIL_FIELDS,
                                    HIL_STALE_FRACTION_MAX, bridge_resets, hil_link_from_log, link_health)
from tools.tuning.hitl_logs import LogBuffer, fetch_log, list_logs, new_log_entries, timing_from_log
from tools.tuning.hitl_params import (ParamFile, Refused, changes, check_disarmed, check_loaded, check_sim_firmware,
                                      check_sim_rate, fetch_params, merge_param_files, origin_params, parse_params,
                                      read_param_file, same_value, write_backup)
from tools.tuning.logdata import ARM_FIELDS, FlightLog, armed_windows
from tools.tuning.vehicle import OverrideTransmitter, Vehicle


def _vehicle(board: FakeBoard) -> Vehicle:
    v = Vehicle(".", speedup=1)
    v.mav = FakeConn(board)
    return v


# ---- RC overrides --------------------------------------------------------------
def test_override_packet_carries_16_channels_then_ignores_17_and_18():
    board = FakeBoard({})
    tx = OverrideTransmitter(_vehicle(board))
    tx.set(9, 2000)
    tx.send_once()
    (args,) = board.overrides
    assert args[:2] == (1, 1)                      # target system and component
    pwm = list(args[2:18])
    assert pwm[2] == 1000 and pwm[8] == 2000       # throttle low, tuning switch high
    assert [p for i, p in enumerate(pwm) if i not in (2, 8)] == [1500] * 14
    assert args[18:] == (0, 0)


def test_override_looks_up_the_connection_on_every_tick():
    first, second = FakeBoard({}), FakeBoard({})
    vehicle = _vehicle(first)
    tx = OverrideTransmitter(vehicle)
    tx.send_once()
    vehicle.mav = FakeConn(second)                 # a reconnect replaces the connection
    tx.send_once()
    assert len(first.overrides) == 1 and len(second.overrides) == 1


@pytest.mark.parametrize("pwm", [0, 65535, 65534, 500, 2500])
def test_override_refuses_values_that_are_not_stick_positions(pwm):
    # 0 releases channels 1-8 and 65535 means "ignore" (GCS_Common.cpp:4018-4030)
    tx = OverrideTransmitter(_vehicle(FakeBoard({})))
    with pytest.raises(ValueError):
        tx.set(1, pwm)
    assert tx.get(1) == 1500


def test_override_counts_send_errors_and_skips_while_reconnecting():
    class Broken:
        target_system = target_component = 1
        mav = SimpleNamespace(rc_channels_override_send=lambda *a: (_ for _ in ()).throw(OSError("gone")))

    vehicle = SimpleNamespace(mav=Broken())
    tx = OverrideTransmitter(vehicle)
    tx.send_once()
    vehicle.mav = None
    tx.send_once()
    assert tx.send_errors == 1


def test_override_thread_sends_until_stopped():
    board = FakeBoard({})
    tx = OverrideTransmitter(_vehicle(board), rate_hz=200).start()
    tx.stop()
    sent = len(board.overrides)
    assert sent >= 1
    tx.stop()
    assert len(board.overrides) == sent


# ---- parameter files -----------------------------------------------------------
def test_parse_params_reads_spaces_tabs_commas_and_comments():
    text = "# header\nSCHED_LOOP_RATE   200\nLOG_BITMASK\t1114111  # full rate\n\nSIM_OPOS_LAT,-35.363261\nX1 1e-3\n"
    assert parse_params(text) == [("SCHED_LOOP_RATE", 200.0), ("LOG_BITMASK", 1114111.0),
                                  ("SIM_OPOS_LAT", -35.363261), ("X1", 0.001)]


@pytest.mark.parametrize("line", ["SERVO1_FUNCTION", "SERVO1_FUNCTION 77 extra", "servo1_function 77",
                                  "SERVO1_FUNCTION seventy", "A_NAME_LONGER_THAN_16 1"])
def test_parse_params_rejects_malformed_lines_with_their_line_number(line):
    with pytest.raises(ValueError, match="f.parm:2"):
        parse_params("OK 1\n" + line, "f.parm")


def test_merge_later_files_win_and_only_required_files_name_required_parameters(tmp_path):
    upstream = tmp_path / "plane.parm"
    upstream.write_text("EK2_ENABLE 1\nRLL_RATE_P 0.3\nSCHED_LOOP_RATE 50\n")
    run = tmp_path / "run.parm"
    run.write_text("SCHED_LOOP_RATE 200\nFTUN_ENABLE 1\nFTUN_ENABLE 0\n")
    values, required = merge_param_files([ParamFile(upstream, False), ParamFile(run, True)])
    assert values == {"EK2_ENABLE": 1.0, "RLL_RATE_P": 0.3, "SCHED_LOOP_RATE": 200.0, "FTUN_ENABLE": 0.0}
    assert required == {"SCHED_LOOP_RATE", "FTUN_ENABLE"}


def test_origin_becomes_sim_opos_and_is_range_checked():
    assert origin_params("-35.363261,149.165230,584,353") == {
        "SIM_OPOS_LAT": -35.363261, "SIM_OPOS_LNG": 149.16523, "SIM_OPOS_ALT": 584.0, "SIM_OPOS_HDG": 353.0}
    for bad in ("1,2,3", "95,0,0,0", "0,0,0,360"):
        with pytest.raises(ValueError):
            origin_params(bad)


def test_values_match_as_float32_tighter_than_set_params_check():
    assert same_value(f32(-35.363261), -35.363261)
    assert same_value(f32(0.1), 0.1)
    assert not same_value(-35.3633, -35.363261)    # about 4 m: set_param's 1e-4 would accept it
    assert changes({"A": 1.0, "B": 2.0, "GONE": 3.0}, {"A": 1.0, "B": 2.5}) == {"B": 2.0}


def test_check_loaded_reports_absent_optional_and_refuses_absent_required_or_differing():
    desired = {"EK2_ENABLE": 1.0, "FTUN_ENABLE": 1.0, "SCHED_LOOP_RATE": 200.0}
    assert check_loaded(desired, {"FTUN_ENABLE"}, {"FTUN_ENABLE": 1.0, "SCHED_LOOP_RATE": 200.0}) == ["EK2_ENABLE"]
    with pytest.raises(Refused, match="FTUN_ENABLE"):
        check_loaded(desired, {"FTUN_ENABLE"}, {"SCHED_LOOP_RATE": 200.0})
    with pytest.raises(Refused, match="SCHED_LOOP_RATE"):
        check_loaded(desired, set(), {"FTUN_ENABLE": 1.0, "SCHED_LOOP_RATE": 50.0})


# ---- gates -----------------------------------------------------------------------
def test_gates_refuse_stock_firmware_real_outputs_and_an_armed_vehicle():
    with pytest.raises(Refused, match="not ArduPilot simulation firmware"):
        check_sim_firmware({"SCHED_LOOP_RATE": 50.0})
    with pytest.raises(Refused, match="real servo outputs"):
        check_sim_firmware({"SIM_OH_MASK": 4.0})
    check_sim_firmware({"SIM_OH_MASK": 0.0})
    with pytest.raises(Refused, match="armed"):
        check_disarmed(Msg("HEARTBEAT", base_mode=128 | 1))
    check_disarmed(Msg("HEARTBEAT", base_mode=1))


def test_rate_gate_needs_the_model_rate_to_equal_the_loop_rate():
    check_sim_rate({"SIM_RATE_HZ": 200.0, "SCHED_LOOP_RATE": 200.0})
    with pytest.raises(Refused, match="real time"):
        check_sim_rate({"SIM_RATE_HZ": 400.0, "SCHED_LOOP_RATE": 200.0})


# ---- parameter list and backup -----------------------------------------------------
def test_fetch_params_asks_again_for_dropped_indices(monkeypatch):
    monkeypatch.setattr(hitl_params, "PARAM_QUIET_S", 0.02)
    board = FakeBoard({f"P{i:02d}": float(i) for i in range(40)})
    board.drop_param_indices = {0, 7, 39}
    assert fetch_params(_vehicle(board)) == {f"P{i:02d}": float(i) for i in range(40)}


def test_fetch_params_fails_when_an_index_never_arrives(monkeypatch):
    monkeypatch.setattr(hitl_params, "PARAM_QUIET_S", 0.01)
    monkeypatch.setattr(hitl_params, "PARAM_LIST_TIMEOUT_S", 0.2)
    board = FakeBoard({"A": 1.0, "B": 2.0})
    board.never_param_indices = {1}
    with pytest.raises(RuntimeError, match="incomplete: 1 of 2"):
        fetch_params(_vehicle(board))


def test_backup_reads_back_exactly_and_never_overwrites(tmp_path):
    params = {"RLL_RATE_P": f32(0.1297145), "SIM_OPOS_LAT": f32(-35.363261), "SERIAL8_PROTOCOL": -1.0}
    path = write_backup(tmp_path / "before.parm", params, "COM15")
    assert read_param_file(path) == params
    with pytest.raises(FileExistsError):
        write_backup(path, params, "COM15")


def test_backup_that_does_not_read_back_is_not_verified(tmp_path):
    with pytest.raises(Refused, match="does not read back"):
        write_backup(tmp_path / "nan.parm", {"BAD": math.nan}, "COM15")


# ---- logs ------------------------------------------------------------------------
def test_new_log_entries_follow_the_newest_log_when_old_ones_are_deleted():
    before = {1: (100, 10), 2: (200, 20), 3: (300, 30)}
    assert new_log_entries(before, {**before, 4: (400, 40)}) == [4]
    # the oldest log was deleted: entries count from the oldest, so all shift down by one
    assert new_log_entries(before, {1: (200, 20), 2: (300, 30), 3: (400, 40)}) == [3]
    assert new_log_entries({}, {1: (5, 1), 2: (6, 2)}) == [1, 2]
    with pytest.raises(RuntimeError, match="no longer listed"):
        new_log_entries(before, {1: (400, 40)})


def test_log_buffer_merges_out_of_order_and_overlapping_chunks():
    buf = LogBuffer()
    assert buf.add(180, b"c" * 90) and buf.add(0, b"a" * 90)
    assert buf.missing(300) == [(90, 180), (270, 300)]
    assert buf.add(90, b"b" * 90)
    assert not buf.add(100, b"b" * 10)              # nothing new
    assert buf.missing(270) == [] and buf.ranges == [[0, 270]]
    assert bytes(buf.data) == b"a" * 90 + b"b" * 90 + b"c" * 90


def _board_with_log(content: bytes, **log_kwargs: object) -> tuple[FakeBoard, Vehicle]:
    board = FakeBoard({})
    board.logs = [FakeLog(b"old" * 50, time_utc=1), FakeLog(content, **log_kwargs)]
    return board, _vehicle(board)


def test_log_reassembles_with_dropped_and_out_of_order_chunks(monkeypatch):
    monkeypatch.setattr(hitl_logs, "LOG_DATA_STALL_S", 0.02)
    content = bytes(range(256)) * 40 + b"tail"          # 10244 bytes, not a multiple of 90
    board, vehicle = _board_with_log(content)
    board.drop_log_offsets = {0, 900, 4500, 10170}      # including the first and the last chunk
    board.shuffle_log_data = True
    assert fetch_log(vehicle, 2, len(content)) == content


def test_log_still_being_written_is_read_to_its_final_size(monkeypatch):
    monkeypatch.setattr(hitl_logs, "LOG_DATA_STALL_S", 0.02)
    content = b"x" * 1000
    board, vehicle = _board_with_log(content, listed_size=450)    # listed before reading closed it
    assert fetch_log(vehicle, 2, 450) == content


def test_log_download_gives_up_when_no_data_comes(monkeypatch):
    monkeypatch.setattr(hitl_logs, "LOG_DATA_STALL_S", 0.01)
    monkeypatch.setattr(hitl_logs, "LOG_MAX_STALLS", 3)
    board, vehicle = _board_with_log(b"y" * 500)
    board.answer_log_data = False
    with pytest.raises(RuntimeError, match="no data for 3 requests"):
        fetch_log(vehicle, 2, 500)


def test_list_logs_reads_every_entry_and_an_empty_card():
    board, vehicle = _board_with_log(b"z" * 10, time_utc=7)
    assert list_logs(vehicle) == {1: (150, 1), 2: (10, 7)}
    assert list_logs(_vehicle(FakeBoard({}))) == {}


# ---- timing ----------------------------------------------------------------------
def test_timing_summary_compares_the_tuner_against_its_budget_and_slot():
    def series(**cols: list[float]) -> dict:
        return {k: np.array(v, dtype=float) for k, v in cols.items()}

    log = FlightLog("x.BIN", {
        "FTPR": series(TimeUS=[1, 2], PcUs=[120, 340], CaUs=[610, 590]),
        "FTD2": series(TimeUS=[], PcUs=[], CaUs=[]),
        "PM": series(TimeUS=[1, 2, 3], NLon=[0, 3, 1], NL=[2000, 2000, 2000], MaxT=[4800, 9100, 5200]),
    }, {}, [])
    t = timing_from_log(log)
    assert t["FTPR"] == {"records": 2, "max_job_us": 340.0, "max_call_us": 610.0,
                         "call_over_budget": True, "call_over_slot": False}
    assert t["FTD2"] == {"records": 0}
    assert t["PM"] == {"records": 3, "long_loops": 4, "loops": 6000, "max_loop_us": 9100.0}
    assert (t["budget_us"], t["slot_us"]) == (600, 700)


# ---- HITL link health ---------------------------------------------------------------
# fork SIM_HIL.cpp write_log, HIL link contract v2
HIL_COLUMNS = b"TimeUS,LS,Age,AgeMx,Stale,Rx,Rej,Rpl,Tx,TxDrop,CrcE,ParE,Oth,Lp"
HIL_FORMAT = "<QBIIIIIIIIIIII"
FRAMES_PER_RECORD = 20                 # 200 Hz main loop, HIL records at 10 Hz
FRAMES_BEFORE_LOGGING = 5000           # the counters run from boot; logging starts at arming
# frames the board dropped before the bridge opened its port: TxDrop runs from boot, so only its
# changes after the connection count (and then Tx + TxDrop still grows as Lp)
DROPPED_BEFORE_CONNECT = 3000


ARM_FORMAT = "<QB"                     # ArduPilot's ARM record, the two fields the gate reads


def _hil_log(path: Path, stale: list[int], age_max: list[int], armed: tuple[int, int | None] = (0, None),
             link_state: dict[int, int] | None = None, more: dict[str, dict[int, int]] | None = None) -> Path:
    """A DataFlash log with the firmware's HIL record: one record per entry, cumulative counters,
    and ARM records: armed just before record armed[0], disarmed just after record armed[1]
    (None: armed to the end of the log). link_state: LS by record (2, ok, elsewhere); more: extra
    counts added to a counter (TxDrop, Rej, Lp, ...) from a record on."""
    head, hil_id, arm_id = b"\xa3\x95", 200, 201
    data = head + bytes([128]) + struct.pack("<BB4s16s64s", hil_id, 3 + struct.calcsize(HIL_FORMAT), b"HIL",
                                             HIL_FORMAT[1:].encode(), HIL_COLUMNS)
    data += head + bytes([128]) + struct.pack("<BB4s16s64s", arm_id, 3 + struct.calcsize(ARM_FORMAT), b"ARM",
                                              ARM_FORMAT[1:].encode(), b"TimeUS,ArmState")
    for k, (st, amx) in enumerate(zip(stale, age_max)):
        time_us, tx = 30_000_000 + k * 100_000, FRAMES_BEFORE_LOGGING + k * FRAMES_PER_RECORD
        if k == armed[0]:
            data += head + bytes([arm_id]) + struct.pack(ARM_FORMAT, time_us - 1, 1)
        extra = {name: sum(n for start, n in (more or {}).get(name, {}).items() if k >= start)
                 for name in ("TxDrop", "Rej", "CrcE", "ParE", "Lp", "Rpl", "Oth")}
        tx_drop = DROPPED_BEFORE_CONNECT + extra["TxDrop"]
        loops = tx + tx_drop + extra["Lp"]
        data += head + bytes([hil_id]) + struct.pack(
            HIL_FORMAT, time_us, (link_state or {}).get(k, hitl_gate.HIL_LS_OK), 5_200, amx, st, tx - st,
            2 + extra["Rej"], extra["Rpl"], tx, tx_drop, 1 + extra["CrcE"], extra["ParE"], extra["Oth"], loops)
        if k == armed[1]:
            data += head + bytes([arm_id]) + struct.pack(ARM_FORMAT, time_us + 1, 0)
    path.write_bytes(data)
    return path


def _bridge_csv(path: Path, rows: list[tuple[str, int]]) -> Path:
    path.write_text(f"frame,{BRIDGE_EVENT},{BRIDGE_ARMED}\n" + "".join(f"{i},{e},{a}\n" for i, (e, a) in enumerate(rows, 1)))
    return path


def test_hil_link_counts_the_armed_window_of_the_firmwares_record(tmp_path):
    # 10 intervals of 20 frames; 10 stale frames in all: exactly the limit, which passes
    log = _hil_log(tmp_path / "00000002.BIN", stale=list(range(100, 111)), age_max=[9_000] * 10 + [HIL_AGE_MAX_US])
    link = hil_link_from_log(hitl_gate.read_log(str(log), {"HIL": HIL_FIELDS, "ARM": ARM_FIELDS}))
    assert (link["records"], link["frames"], link["stale"]) == (11, 200, 10)
    assert link["stale_fraction"] == HIL_STALE_FRACTION_MAX and link["age_max_us"] == HIL_AGE_MAX_US
    assert (link["rx"], link["rejected"], link["crc_errors"], link["age_median_us"]) == (190, 0, 0, 5_200.0)
    # the frames the board dropped before the bridge connected do not count, and one frame per loop
    assert (link["tx_dropped"], link["loops"], link["frames_not_one_per_loop"]) == (0, 200, 0)
    assert link["link_states"] == [hitl_gate.HIL_LS_OK] and link["link_not_ok_records"] == 0
    health = link_health([log], _bridge_csv(tmp_path / "link.csv", [(FIRST, 0), (STEP, 1), (STEP, 1)]))
    assert health["ok"] and health["problems"] == [] and health["bridge"]["armed_seen"]


def test_the_gate_counts_only_the_armed_flight_not_the_records_around_it(tmp_path):
    """The log also holds HIL records from before the arming and from just after the force-disarm,
    when the runner has stopped the relay: a 78 ms age there failed a healthy HITL run on SITL."""
    stale = [0, 30, 60] + [60] * 10 + [60, 80]              # 30 stale frames per record outside the flight
    age_max = [40_000, 40_000, 9_000] + [9_000] * 10 + [9_000, 78_302]
    log = _hil_log(tmp_path / "00000001.BIN", stale, age_max, armed=(2, 12))
    link = hil_link_from_log(hitl_gate.read_log(str(log), {"HIL": HIL_FIELDS, "ARM": ARM_FIELDS}))
    assert (link["records"], link["frames"], link["stale"], link["age_max_us"]) == (11, 200, 0, 9_000)
    assert link["seconds"] == pytest.approx(1.0)
    health = link_health([log], _bridge_csv(tmp_path / "link.csv", [(FIRST, 0), (STEP, 1)]))
    assert health["ok"], health["problems"]
    never_armed = _hil_log(tmp_path / "00000002.BIN", [0] * 5, [9_000] * 5, armed=(99, None))
    (problem,) = link_health([never_armed], tmp_path / "link.csv")["problems"]
    assert "0 HIL records inside the armed flight" in problem and "5 in the log" in problem


def test_armed_windows_pair_each_arming_with_its_disarm():
    def log(times: list[float], states: list[int]) -> FlightLog:
        return FlightLog("x.BIN", {"ARM": {"TimeUS": np.array(times, dtype=float),
                                           "ArmState": np.array(states, dtype=float)}}, {}, [])
    assert armed_windows(log([10, 20, 30, 35, 40], [1, 0, 1, 1, 0])) == [(10.0, 20.0), (30.0, 40.0)]
    assert armed_windows(log([5, 10], [0, 1])) == [(10.0, np.inf)]                  # still armed at the end
    assert armed_windows(FlightLog("x.BIN", {}, {}, [])) == []


def test_link_gate_fails_on_too_many_stale_frames_or_an_old_state(tmp_path):
    csv_path = _bridge_csv(tmp_path / "link.csv", [(FIRST, 0), (STEP, 1)])
    stale = _hil_log(tmp_path / "stale.BIN", stale=[100] * 10 + [111], age_max=[9_000] * 11)
    old = _hil_log(tmp_path / "old.BIN", stale=[100] * 11, age_max=[9_000] * 5 + [HIL_AGE_MAX_US + 1] + [9_000] * 5)
    health = link_health([stale, old], csv_path)
    assert not health["ok"] and len(health["problems"]) == 2
    assert "stale.BIN: 11 of 200 frames stale (5.5%)" in health["problems"][0]
    assert f"old.BIN: state age reached {HIL_AGE_MAX_US + 1} us" in health["problems"][1]


def test_only_a_reset_to_the_origin_after_the_first_armed_frame_fails_the_run(tmp_path):
    before = _bridge_csv(tmp_path / "a.csv", [(FIRST, 0), (REINIT, 0), (STEP, 0), (STEP, 1), (GAP, 1), (STEP, 1)])
    assert bridge_resets(before) == {"frames": 6, "resets": 1, "resets_after_armed": 0, "gaps": 1,
                                     "retransmits": 0, "conflicts": 0, "crashes": 0,
                                     "armed_seen": True}   # a gap is stepped across: the state age gates it
    after = _bridge_csv(tmp_path / "b.csv", [(FIRST, 0), (STEP, 1), (REINIT, 1), (STEP, 0), (REINIT, 0)])
    assert bridge_resets(after)["resets_after_armed"] == 2         # disarmed again after it: still mid-run
    log = _hil_log(tmp_path / "ok.BIN", stale=[0] * 3, age_max=[9_000] * 3)
    health = link_health([log], after)
    assert not health["ok"] and "reset the aircraft to the origin 2 time(s)" in health["problems"][0]


def test_a_conflicting_frame_fails_the_run_and_a_retransmission_does_not(tmp_path):
    """HIL link contract v2: board time strictly increases, so other content at an answered board
    time (CONFLICT) is a broken board; the same frame again (RETRANSMIT, lockstep SITL after a
    reconnect) is answered and only counted"""
    log = _hil_log(tmp_path / "ok.BIN", stale=[0] * 3, age_max=[9_000] * 3)
    resent = _bridge_csv(tmp_path / "a.csv", [(FIRST, 1), (STEP, 1), (RETRANSMIT, 1), (STEP, 1)])
    health = link_health([log], resent)
    assert health["ok"] and (health["bridge"]["retransmits"], health["bridge"]["conflicts"]) == (1, 0)
    broken = _bridge_csv(tmp_path / "b.csv", [(FIRST, 1), (STEP, 1), (CONFLICT, 1), (STEP, 1)])
    (problem,) = link_health([log], broken)["problems"]
    assert "1 conflicting frame(s)" in problem


def test_a_crash_of_the_simulated_aircraft_fails_the_run(tmp_path):
    log = _hil_log(tmp_path / "ok.BIN", stale=[0] * 3, age_max=[9_000] * 3)
    crashed = _bridge_csv(tmp_path / "a.csv", [(FIRST, 0), (STEP, 1), (hitl_gate.CRASH, 1), (STEP, 1)])
    (problem,) = link_health([log], crashed)["problems"]
    assert "crashed 1 time(s)" in problem


def test_link_gate_needs_board_logs_and_a_readable_bridge_log(tmp_path):
    health = link_health([], tmp_path / "missing.csv")
    assert not health["ok"] and "no board log" in health["problems"][0] and "missing.csv" in health["problems"][1]
    (tmp_path / "cols.csv").write_text("frame,armed\n1,0\n")
    log = _hil_log(tmp_path / "ok.BIN", stale=[0] * 3, age_max=[9_000] * 3)
    (problem,) = link_health([log], tmp_path / "cols.csv")["problems"]
    assert "columns ['event'] missing" in problem


def test_link_gate_fails_on_a_lost_link_dropped_frames_rejected_states_or_skipped_frames(tmp_path):
    """HIL link contract v2: inside the armed flight LS stays ok, TxDrop, Rej, CrcE and ParE do not
    grow, and Tx + TxDrop grows exactly as the completed main loops (fork autotest
    hil_check_link_health); Rpl and Oth are reported only"""
    csv_path = _bridge_csv(tmp_path / "link.csv", [(FIRST, 0), (STEP, 1)])
    ok = dict(stale=[0] * 6, age_max=[9_000] * 6)
    lost = _hil_log(tmp_path / "lost.BIN", **ok, link_state={3: hitl_gate.HIL_LS_LOST})
    dropped = _hil_log(tmp_path / "dropped.BIN", **ok, more={"TxDrop": {2: 3}})
    rejected = _hil_log(tmp_path / "rejected.BIN", **ok, more={"Rej": {4: 1}, "CrcE": {4: 2}})
    skipped = _hil_log(tmp_path / "skipped.BIN", **ok, more={"Lp": {3: 1}})       # a loop sent no frame
    normal = _hil_log(tmp_path / "normal.BIN", **ok, more={"Rpl": {2: 5}, "Oth": {1: 9}})
    health = link_health([lost, dropped, rejected, skipped, normal], csv_path)
    problems = health["problems"]
    assert not health["ok"] and len(problems) == 5, problems
    assert "lost.BIN: link state not ok (LS [2, 3]) in 1 records" in problems[0]
    assert "dropped.BIN: 3 actuator frames dropped (TxDrop)" in problems[1]
    assert "rejected.BIN: 1 states rejected (Rej)" in problems[2] and "2 frames with a bad CRC (CrcE)" in problems[3]
    assert "skipped.BIN: 100 actuator frames for 101 main loops" in problems[4]
    assert health["logs"]["normal.BIN"]["replaced"] == 5 and health["logs"]["normal.BIN"]["other_frames"] == 9
    assert hitl_gate.HIL_FIELDS == tuple(HIL_COLUMNS.decode().split(","))       # every field of the record


def test_hil_records_without_a_field_are_not_read_as_a_healthy_link():
    series = {f: np.array([1.0, 2.0]) for f in HIL_FIELDS}
    series["TxDrop"] = np.array([np.nan, np.nan])                    # read_log gives NaN for an absent field
    link = hil_link_from_log(FlightLog("x.BIN", {"HIL": series}, {}, []))
    assert link == {"records": 2, "missing_fields": ["TxDrop"]}
    assert hitl_gate.hil_link_problems("x.BIN", link) == ["x.BIN: HIL records lack ['TxDrop']"]


def test_the_gate_reads_the_bridges_own_log_columns_and_reset_event():
    assert BRIDGE_EVENT in BRIDGE_LOG_COLUMNS and BRIDGE_ARMED in BRIDGE_LOG_COLUMNS
    assert (hitl_gate.REINIT, hitl_gate.GAP) == (REINIT, GAP)


# ---- catapult takeoff ------------------------------------------------------------------
class ClimbingVehicle:
    """Just enough of a Vehicle for takeoff_to_fbwa: modes, arming, a climb and STATUSTEXT."""

    def __init__(self, tkoff_alt: float, climb: list[float], texts_at_arming: list[str]) -> None:
        self.speedup = 1
        self.tkoff_alt = tkoff_alt
        self.climb = list(climb)
        self.texts_at_arming = texts_at_arming
        self.messages: list[str] = ["Ready to fly"]
        self.events: list[str] = []
        self.tx = SimpleNamespace(set=lambda ch, pwm: self.events.append(f"rc{ch}:{pwm}"))

    def get_param(self, name: str) -> float:
        self.events.append(f"get:{name}")
        return {"TKOFF_ALT": self.tkoff_alt}[name]

    def wait_ready(self) -> None:
        self.events.append("ready")

    def set_mode(self, mode: str) -> None:
        self.events.append(f"mode:{mode}")

    def arm(self) -> None:
        self.events.append("arm")
        self.messages += self.texts_at_arming

    def state(self) -> dict:
        return {"alt": self.climb.pop(0) if len(self.climb) > 1 else self.climb[0]}

    def fly_for(self, seconds: float) -> None:
        self.events.append(f"fly:{seconds:g}")


def test_takeoff_on_sitls_runway_is_unchanged():
    vehicle = ClimbingVehicle(120, [0, 30, 80, 111], ["Armed AUTO, xaccel = 0.0 m/s/s, waiting 0.2 sec",
                                                      "Triggered AUTO. GPS speed = 0.0"])
    sitl_driver.takeoff_to_fbwa(vehicle)
    assert vehicle.events == ["get:TKOFF_ALT", "ready", "mode:TAKEOFF", "arm", "rc3:1650", "mode:FBWA", "fly:10"]


def test_catapult_takeoff_climbs_to_the_aircrafts_tkoff_alt_not_a_fixed_height(monkeypatch):
    monkeypatch.setattr(sitl_driver, "TAKEOFF_TIMEOUT_S", 0.2)
    launched = ["Armed AUTO, xaccel = 41.0 m/s/s, waiting 0.1 sec", "Triggered AUTO. GPS speed = 15.0"]
    vehicle = ClimbingVehicle(50, [0, 0, 12, 33, 41], launched)       # an aircraft file's lower TKOFF_ALT 50
    monkeypatch.setattr(sitl_driver, "climb_to", lambda v, h: v.events.append(f"climb:{h:g}"))
    sitl_driver.takeoff_to_fbwa(vehicle)
    # Changed: the takeoff still ends at the aircraft's TKOFF_ALT (41 m counts), then the pilot's steps start
    # from the test height (sign_check2/fix1: FBWA from 48 m sank through the 30 m floor)
    assert vehicle.events[-4:] == ["climb:120", "rc3:1650", "mode:FBWA", "fly:10"]


def test_a_takeoff_whose_motor_never_started_says_so(monkeypatch):
    monkeypatch.setattr(sitl_driver, "TAKEOFF_TIMEOUT_S", 0.05)
    with pytest.raises(RuntimeError, match="motor never started .*TKOFF_THR_MINACC"):
        sitl_driver.takeoff_to_fbwa(ClimbingVehicle(50, [0], []))       # no catapult: MINACC never seen
    with pytest.raises(RuntimeError, match=r"did not reach TKOFF_ALT 50 m$"):
        sitl_driver.takeoff_to_fbwa(ClimbingVehicle(50, [0, 20], ["Triggered AUTO. GPS speed = 15.0"]))
