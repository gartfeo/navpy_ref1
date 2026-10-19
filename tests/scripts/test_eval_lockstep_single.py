"""``eval_lockstep_single.py``'s sensor tap: the step-2 proof's baro and airspeed streams."""

from __future__ import annotations

from navpy.sim.lockstep.mavlink_wire import encoder
from navpy.sim.lockstep.step_wire import Frame, PortState
from scripts.eval_lockstep_single import SensorTap


class _Operator:
    def __init__(self) -> None:
        self.ticks: list[int] = []

    def on_tick(self, frame: Frame) -> tuple[bytes, bytes]:
        self.ticks.append(frame.tick)
        return b"s0", b"s1"


def _messages(press_abs: float, airspeed: float) -> bytes:
    link, sink = encoder(1, 1)
    link.scaled_pressure_send(1000, press_abs, 0.5, 2150)
    link.heartbeat_send(1, 3, 0, 0, 0)
    link.vfr_hud_send(airspeed, 18.0, 90, 50, 120.0, 0.0)
    return sink.drain()


def test_the_tap_records_serial1s_baro_and_airspeed_and_passes_the_operator_through(tmp_path):
    operator = _Operator()
    tap = SensorTap(operator)
    ports = (PortState(0, 0, 0, _messages(800.0, 3.0)),
             PortState(0, 0, 0, b"Init ArduPlane V4.6.3\n" + _messages(950.25, 17.5)))
    assert tap.on_tick(Frame((1, 2), 7, 140_000, 0, True, ports)) == (b"s0", b"s1")
    assert operator.ticks == [7]
    assert tap.rows == [(7, 140_000, "SCALED_PRESSURE.press_abs", 950.25),
                        (7, 140_000, "SCALED_PRESSURE.press_diff", 0.5),
                        (7, 140_000, "SCALED_PRESSURE.temperature", 2150.0),
                        (7, 140_000, "VFR_HUD.airspeed", 17.5)]
    tap.write_csv(tmp_path / "s.csv")
    lines = (tmp_path / "s.csv").read_text(encoding="utf-8").splitlines()
    assert lines[0] == "tick,source_us,field,value" and lines[1] == "7,140000,SCALED_PRESSURE.press_abs,950.25"
