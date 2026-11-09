"""The runner's recovery floor is released at navpy's confirm (HITL step 23b, option A: the dock is on the ground)."""
from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from tools.tuning import approach_quiesce as stop
from tools.tuning.flight_floor import HeightFloor
from tools.tuning.flight_steps import FlightCheckFailed
from tests.tools.tuning.test_approach_quiesce import DOCKED, RECORD_LINE, ObservedBoard


class _Mav:
    def __init__(self) -> None:
        self.message_hooks = []

    def send(self, height_m: float) -> None:
        msg = SimpleNamespace(get_type=lambda: "GLOBAL_POSITION_INT", relative_alt=int(height_m * 1000))
        for hook in list(self.message_hooks):
            hook(self, msg)


def _floor(record: dict) -> tuple[HeightFloor, _Mav]:
    mav = _Mav()
    return HeightFloor(SimpleNamespace(mav=mav), 30.0, record), mav


def test_before_the_release_the_floor_trips():
    record = {}
    floor, mav = _floor(record)
    with pytest.raises(FlightCheckFailed):
        with floor:
            mav.send(29.0)
    assert record["height"]["breach"] and record["height"]["released"] is None


def test_after_the_release_the_descent_to_the_ground_does_not_trip():
    record = {}
    floor, mav = _floor(record)
    with floor:
        mav.send(80.0)
        floor.release("navpy confirm, task 1")
        floor.release("a second confirm")              # the first reason is kept
        mav.send(0.5)
    assert record["height"] == {"floor_m": 30.0, "lowest_m": 80.0, "breach": None,
                                "released": "navpy confirm, task 1"}


def test_a_breach_before_the_confirm_stands():
    record = {}
    floor, mav = _floor(record)
    floor.__enter__()
    with pytest.raises(FlightCheckFailed):
        mav.send(10.0)
    floor.release("late confirm")
    assert floor.released is None and floor.breach


def test_observe_hands_each_confirm_record_to_the_floor(tmp_path, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(stop.time, "monotonic", lambda: clock[0])
    out = tmp_path / "navpy.out"
    out.write_text("", encoding="utf-8")
    confirm = 'NAVPY_CONFIRM_V1 {"task":3}\n'
    board = ObservedBoard(clock, out, {1: confirm[:10], 2: confirm[10:] + "NAVPY_CONFIRM_V1 junk\n",
                                       3: RECORD_LINE + "\n"})
    group = type("Group", (), {"check": lambda self: None})()
    seen = []
    assert stop.observe(board, group, out, 10 * stop.TERMINAL_POLL_S, seen.append) == stop.ENDED_BY_TERMINAL
    assert seen == [{"task": 3}]
    assert json.loads(RECORD_LINE.split(" ", 1)[1]) == DOCKED
