"""dock-pass (tools/tuning/dock_pass.py, sitl_driver dock-pass): the pass geometry over the board, the mission it
uploads, the upload protocol and the flight of the passes, on a fake autopilot."""
from __future__ import annotations

import argparse
import json
import math
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from tools.tuning import dock_pass, sitl_driver
from tools.tuning.dock_pass import (APPROACH_M, DOCK_BOARD_YEGHVARD, DOCK_PASS_MIN_HEIGHT_M, END_MODE, ENTRY_M, EXIT_M,
                                    FLOOR_MARGIN_M, PASS_AIRSPEED_M_S, PASS_CARD, Point, build_card, distance_m,
                                    fly_passes, mission_items, upload_mission)
from tools.tuning.flight_steps import FlightCheckFailed
from tools.tuning.vehicle import MODE

M = mavutil.mavlink
YEGHVARD = "40.3117412,44.4552112,1292.8,0"
BOARD_ABOVE_HOME_M = 1296.865 - 1292.8
OURS, MISSION_PLANNER = 250, 255          # the runner's and a mirrored Mission Planner's system ids
RAIL_M = 1.0                              # home is set on the rail, above the origin


class Msg(SimpleNamespace):
    def get_type(self) -> str:
        return self.mtype


class FakeAutopilot:
    """The mission protocol and AUTO of an ArduPlane, enough for dock_pass.

    Upload: after MISSION_COUNT it requests every item in order and then acknowledges with ack_type, all addressed to
    the runner; before_upload messages (e.g. another station's traffic) come first. Flight: in AUTO each wait for a
    mission message reports the next item reached (skip_at: that item is skipped; leave_auto_at: AUTO is left there);
    ATTITUDE carries a board clock (none when no_attitude); HOME_POSITION is the origin plus RAIL_M."""

    def __init__(self, ack_type: int = M.MAV_MISSION_ACCEPTED, request_seq: int | None = None,
                 leave_auto_at: int | None = None, skip_at: int | None = None, before_upload: list | None = None,
                 no_attitude: bool = False) -> None:
        self.speedup = 1
        self.ack_type, self.request_seq, self.leave_auto_at, self.skip_at = ack_type, request_seq, leave_auto_at, skip_at
        self.no_attitude = no_attitude
        self.items: dict[int, tuple] = {}
        self.count = 0
        self.pending: list[Msg] = list(before_upload or [])
        self.mode = "TAKEOFF"
        self.modes: list[str] = []
        self.current = 0
        self.boot_ms = 0
        self.flown_s = 0.0
        self.mav = SimpleNamespace(target_system=1, target_component=1, message_hooks=[],
                                   mav=SimpleNamespace(srcSystem=OURS, mission_count_send=self._count,
                                                       mission_item_int_send=self._item,
                                                       command_long_send=self._command))

    def _count(self, system: int, component: int, count: int, mission_type: int) -> None:
        self.count = count
        first = self.request_seq if self.request_seq is not None else 0
        self.pending.append(Msg(mtype="MISSION_REQUEST_INT", seq=first, mission_type=mission_type, target_system=OURS))

    def _item(self, system, component, seq, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z,
              mission_type) -> None:
        self.items[seq] = (frame, command, p1, p2, p3, p4, x, y, z)
        if seq + 1 < self.count:
            self.pending.append(Msg(mtype="MISSION_REQUEST_INT", seq=seq + 1, mission_type=mission_type,
                                    target_system=OURS))
        else:
            self.pending.append(Msg(mtype="MISSION_ACK", type=self.ack_type, mission_type=mission_type,
                                    target_system=OURS))

    def _command(self, system, component, command, confirmation, *params) -> None:
        if command == M.MAV_CMD_REQUEST_MESSAGE and params[0] == dock_pass.HOME_POSITION_ID:
            lat, lon, alt = (float(v) for v in YEGHVARD.split(",")[:3])
            self.pending.append(Msg(mtype="HOME_POSITION", latitude=round(lat * 1e7), longitude=round(lon * 1e7),
                                    altitude=round((alt + RAIL_M) * 1e3)))

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)
        self.mode = mode
        if mode == "AUTO":
            self.current = 1

    def fly_for(self, seconds: float) -> None:
        self.flown_s += seconds

    def wait_message(self, mtype, timeout: float = 10.0, cond=None):
        types = (mtype,) if isinstance(mtype, str) else tuple(mtype)
        while self.pending and self.pending[0].mtype in types:
            msg = self.pending.pop(0)
            if cond is None or cond(msg):
                return msg
        if "ATTITUDE" in types:
            self.boot_ms += 100
            return None if self.no_attitude else Msg(mtype="ATTITUDE", time_boot_ms=self.boot_ms)
        if "MISSION_ITEM_REACHED" in types and self.mode == "AUTO":
            if self.leave_auto_at is not None and self.current >= self.leave_auto_at:
                self.mode = "RTL"
                return Msg(mtype="HEARTBEAT", custom_mode=MODE["RTL"], autopilot=M.MAV_AUTOPILOT_ARDUPILOTMEGA)
            if self.current == self.skip_at:
                self.current += 1
            reached, self.current = self.current, self.current + 1
            self.boot_ms += 20_000
            return Msg(mtype="MISSION_ITEM_REACHED", seq=reached)
        return None


def _card():
    return build_card(DOCK_BOARD_YEGHVARD, YEGHVARD, DOCK_PASS_MIN_HEIGHT_M)


def _bearing(a: Point, b: Point) -> float:
    north = b.lat - a.lat
    east = (b.lon - a.lon) * math.cos(math.radians(a.lat))
    return math.degrees(math.atan2(east, north)) % 360.0


def test_each_pass_is_a_line_through_the_board_along_its_heading_at_its_height_above_the_board():
    card = _card()
    assert [(p.height_m, p.heading_deg) for p in card.passes] == list(PASS_CARD)
    for p in card.passes:
        assert distance_m(p.approach, card.board) == pytest.approx(APPROACH_M, rel=1e-3)
        assert distance_m(p.entry, card.board) == pytest.approx(ENTRY_M, rel=1e-3)
        assert distance_m(p.exit, card.board) == pytest.approx(EXIT_M, rel=1e-3)
        # approach -> entry -> board -> exit all on the heading
        for a, b in ((p.approach, p.entry), (p.entry, card.board), (card.board, p.exit)):
            assert (_bearing(a, b) - p.heading_deg + 180.0) % 360.0 - 180.0 == pytest.approx(0.0, abs=0.05)
        assert p.alt_rel_home_m == pytest.approx(BOARD_ABOVE_HOME_M + p.height_m, abs=0.01)
    assert [p.first_seq for p in card.passes] == list(range(2, 2 + 3 * len(PASS_CARD), 3))


def test_the_yeghvard_board_is_where_the_unreal_log_puts_it_relative_to_the_origin():
    card = _card()
    assert distance_m(card.home, card.board) == pytest.approx(math.hypot(250.0, 35.0), abs=2.0)


def test_a_floor_the_lowest_pass_would_trip_a_non_number_and_a_far_board_are_refused_before_the_flight():
    lowest = BOARD_ABOVE_HOME_M + min(h for h, _ in PASS_CARD)
    build_card(DOCK_BOARD_YEGHVARD, YEGHVARD, lowest - FLOOR_MARGIN_M)
    assert DOCK_PASS_MIN_HEIGHT_M <= lowest - FLOOR_MARGIN_M
    with pytest.raises(ValueError, match="lowest pass"):
        build_card(DOCK_BOARD_YEGHVARD, YEGHVARD, lowest - FLOOR_MARGIN_M + 0.1)
    with pytest.raises(ValueError, match="a number"):
        build_card(DOCK_BOARD_YEGHVARD, YEGHVARD, float("nan"))
    with pytest.raises(ValueError, match="from the origin"):
        build_card(DOCK_BOARD_YEGHVARD, "-35.363261,149.165230,584,353", DOCK_PASS_MIN_HEIGHT_M)


def test_the_mission_is_home_the_cruise_speed_three_waypoints_per_pass_above_home_then_a_loiter():
    card = _card()
    items = mission_items(card)
    assert [i[0] for i in items] == list(range(len(items))) and len(items) == 2 + 3 * len(PASS_CARD) + 1
    assert items[1][2] == M.MAV_CMD_DO_CHANGE_SPEED and items[1][4] == PASS_AIRSPEED_M_S
    for p in card.passes:
        for k, point in enumerate((p.approach, p.entry, p.exit)):
            seq, frame, command, *_, x, y, z = items[p.first_seq + k]
            assert (frame, command) == (M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, M.MAV_CMD_NAV_WAYPOINT)
            assert (x, y, z) == (round(point.lat * 1e7), round(point.lon * 1e7), p.alt_rel_home_m)
    # the last exit is not the mission's end and the item after it keeps AUTO, so the exit is reported reached
    # before any mode change
    assert items[-1][2] == M.MAV_CMD_NAV_LOITER_UNLIM and items[-2][0] == card.passes[-1].first_seq + 2
    assert items[-1][-3:] == items[-2][-3:]


def test_upload_answers_every_request_and_needs_an_accepted_ack_after_the_last_item():
    items = mission_items(_card())
    autopilot = FakeAutopilot()
    upload_mission(autopilot, items)
    assert sorted(autopilot.items) == list(range(len(items)))
    assert autopilot.items[2][-3:] == items[2][-3:]
    with pytest.raises(FlightCheckFailed, match="refused"):
        upload_mission(FakeAutopilot(ack_type=M.MAV_MISSION_NO_SPACE), items)
    with pytest.raises(FlightCheckFailed, match="asked for item"):
        upload_mission(FakeAutopilot(request_seq=len(items)), items)


def test_upload_ignores_answers_to_another_station_and_refuses_an_early_accept():
    items = mission_items(_card())
    other = [Msg(mtype="MISSION_ACK", type=M.MAV_MISSION_DENIED, mission_type=0, target_system=MISSION_PLANNER),
             Msg(mtype="MISSION_REQUEST_INT", seq=5, mission_type=0, target_system=MISSION_PLANNER)]
    autopilot = FakeAutopilot(before_upload=other)
    upload_mission(autopilot, items)
    assert sorted(autopilot.items) == list(range(len(items)))
    stale = [Msg(mtype="MISSION_ACK", type=M.MAV_MISSION_ACCEPTED, mission_type=0, target_system=OURS)]
    with pytest.raises(FlightCheckFailed, match="accepted after 0 of"):
        upload_mission(FakeAutopilot(before_upload=stale), items)


def test_the_passes_are_flown_in_auto_with_their_board_time_windows_then_rtl():
    card = _card()
    autopilot = FakeAutopilot()
    flown: list[dict] = []
    fly_passes(autopilot, card, flown)
    assert autopilot.modes == ["AUTO", END_MODE] and autopilot.flown_s == dock_pass.END_RTL_S
    assert [(f["height_m"], f["heading_deg"]) for f in flown] == list(PASS_CARD)
    windows = [f["window_board_ms"] for f in flown]
    assert all(lo < hi for lo, hi in windows)
    assert all(a[1] < b[0] for a, b in zip(windows, windows[1:]))      # in flight order, not overlapping


def test_leaving_auto_fails_the_run_and_keeps_the_passes_flown():
    card = _card()
    flown: list[dict] = []
    with pytest.raises(FlightCheckFailed, match="AUTO left"):
        fly_passes(FakeAutopilot(leave_auto_at=card.passes[1].first_seq), card, flown)
    assert len(flown) == 1


def test_a_skipped_waypoint_fails_instead_of_inventing_a_window():
    card = _card()
    with pytest.raises(FlightCheckFailed, match="reached before item"):
        fly_passes(FakeAutopilot(skip_at=card.passes[0].first_seq + 1), card, [])


def test_a_ground_stations_heartbeat_does_not_end_the_card():
    card = _card()
    autopilot = FakeAutopilot()
    gcs = Msg(mtype="HEARTBEAT", custom_mode=0, autopilot=M.MAV_AUTOPILOT_INVALID)
    original = autopilot.wait_message
    sent = []

    def with_gcs(mtype, timeout=10.0, cond=None):
        if "HEARTBEAT" in mtype and not sent:         # Mission Planner's heartbeat on the mirrored link, once
            sent.append(gcs)
            autopilot.pending.insert(0, gcs)
        return original(mtype, timeout, cond)
    autopilot.wait_message = with_gcs
    flown: list[dict] = []
    fly_passes(autopilot, card, flown)
    assert sent and len(flown) == len(PASS_CARD)


def test_a_waypoint_reported_after_the_deadline_fails(monkeypatch):
    clock = iter([0.0, 1.0, 20.0])          # deadline at 10 s; the receive returns at 20 s
    monkeypatch.setattr(dock_pass.time, "time", lambda: next(clock))
    autopilot = FakeAutopilot()
    autopilot.set_mode("AUTO")
    with pytest.raises(FlightCheckFailed, match="not reached within"):
        dock_pass.wait_reached(autopilot, 1, 10.0)


def test_the_lowest_pass_is_flown_from_four_directions():
    lowest = min(h for h, _ in PASS_CARD)
    assert sorted(hd for h, hd in PASS_CARD if h == lowest) == [0.0, 90.0, 180.0, 270.0]


def test_a_window_without_a_board_time_fails():
    with pytest.raises(FlightCheckFailed, match="ATTITUDE"):
        fly_passes(FakeAutopilot(no_attitude=True), _card(), [])


def test_a_wrong_origin_is_refused_before_any_vehicle_is_opened(monkeypatch):
    def no_vehicle(*_args, **_kwargs):
        raise AssertionError("opened a vehicle")
    monkeypatch.setattr(sitl_driver, "open_vehicle", no_vehicle)
    args = argparse.Namespace(dock=DOCK_BOARD_YEGHVARD, origin="-35.363261,149.165230,584,353",
                              min_height=DOCK_PASS_MIN_HEIGHT_M, workdir="unused")
    with pytest.raises(SystemExit, match="from the origin"):
        sitl_driver.dock_pass(args)


def test_dock_pass_flies_the_card_from_the_autopilots_home_under_its_floor_and_records_it(monkeypatch, tmp_path):
    autopilot = FakeAutopilot()
    autopilot.start = autopilot.stop = lambda: None
    autopilot.logs = lambda: []
    autopilot.report = lambda: {}
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *a, **k: autopilot)
    monkeypatch.setattr(sitl_driver, "takeoff", lambda v: 50.0)
    floors, guarded = [], []

    class Floor:
        active = False

        def __init__(self, vehicle, floor_m, record):
            floors.append(floor_m)
            self.record = record

        def __enter__(self):
            Floor.active = True
            return self

        def __exit__(self, *exc):
            Floor.active = False
            self.record["height"] = {"floor_m": floors[-1]}

    monkeypatch.setattr(sitl_driver, "HeightFloor", Floor)
    home_of = sitl_driver.home_position
    monkeypatch.setattr(sitl_driver, "home_position", lambda v: (guarded.append(Floor.active), home_of(v))[1])
    args = argparse.Namespace(dock=DOCK_BOARD_YEGHVARD, origin=YEGHVARD, min_height=DOCK_PASS_MIN_HEIGHT_M,
                              workdir=str(tmp_path / "run"))
    assert sitl_driver.dock_pass(args) == 0
    record = json.loads((tmp_path / "run.json").read_text())
    assert floors == [DOCK_PASS_MIN_HEIGHT_M] and guarded == [True]      # the floor watches the home lookup too
    assert len(record["passes"]) == len(PASS_CARD) and record["card"]["floor_m"] == DOCK_PASS_MIN_HEIGHT_M
    assert record["card"]["passes"][0]["seq"] == {"approach": 2, "entry": 3, "exit": 4}
    # heights above home are flown from the home the autopilot set on the rail, not the origin
    assert record["card"]["home"]["msl_m"] == pytest.approx(1292.8 + RAIL_M)
    first = record["card"]["passes"][0]
    assert first["alt_rel_home_m"] == pytest.approx(BOARD_ABOVE_HOME_M - RAIL_M + PASS_CARD[0][0], abs=0.01)


def test_a_home_that_brings_the_lowest_pass_near_the_floor_fails_the_run_cleanly(monkeypatch, tmp_path):
    autopilot = FakeAutopilot()
    lat, lon, alt = (float(v) for v in YEGHVARD.split(",")[:3])
    high_home = Msg(mtype="HOME_POSITION", latitude=round(lat * 1e7), longitude=round(lon * 1e7),
                    altitude=round((alt + 3.0) * 1e3))
    autopilot._command = lambda *args: autopilot.pending.append(high_home)
    autopilot.mav.mav.command_long_send = autopilot._command
    autopilot.start = autopilot.stop = lambda: None
    autopilot.logs = lambda: []
    autopilot.report = lambda: {}
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *a, **k: autopilot)
    monkeypatch.setattr(sitl_driver, "takeoff", lambda v: 50.0)
    args = argparse.Namespace(dock=DOCK_BOARD_YEGHVARD, origin=YEGHVARD, min_height=DOCK_PASS_MIN_HEIGHT_M,
                              workdir=str(tmp_path / "run"))
    with pytest.raises(FlightCheckFailed, match="autopilot's home"):
        sitl_driver.dock_pass(args)
    assert "lowest pass" in json.loads((tmp_path / "run.json").read_text())["error"]
