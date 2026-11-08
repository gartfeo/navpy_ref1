"""Sim mode-switch params: written only when different, confirmed by reading back."""
from __future__ import annotations

from gcs.backend.sim_mode_params import (
    PLANE_MODE_AUTO,
    SIM_MODE_SWITCH_PARAMS,
    ensure_sim_mode_params,
)


class _Vehicle:
    """Parameter store; ``sticky`` names ignore writes, ``broken`` raise."""

    def __init__(self, params, *, sticky=(), broken=()):
        self.params = dict(params)
        self.sticky = set(sticky)
        self.broken = set(broken)
        self.writes: list[tuple[str, float]] = []

    def get_parameter_fresh(self, name, timeout=2.0):
        if name in self.broken:
            raise TimeoutError(name)
        return self.params.get(name)

    def set_parameter(self, name, value):
        self.writes.append((name, value))
        if name not in self.sticky:
            self.params[name] = value
        return True


def test_values_are_plane_auto():
    assert PLANE_MODE_AUTO == 10.0
    assert dict(SIM_MODE_SWITCH_PARAMS) == {"FLTMODE6": 10.0, "INITIAL_MODE": 10.0}


def test_autotune_switch_position_is_rewritten_and_confirmed():
    vehicle = _Vehicle({"FLTMODE6": 8.0, "INITIAL_MODE": 10.0})
    assert ensure_sim_mode_params(vehicle, 1) == []
    assert vehicle.writes == [("FLTMODE6", 10.0)]
    assert vehicle.params["FLTMODE6"] == 10.0


def test_right_values_are_not_written():
    vehicle = _Vehicle(dict(SIM_MODE_SWITCH_PARAMS))
    assert ensure_sim_mode_params(vehicle, 1) == []
    assert vehicle.writes == []


def test_unconfirmed_and_failing_params_are_reported_not_raised():
    vehicle = _Vehicle({"FLTMODE6": 8.0}, sticky={"FLTMODE6"}, broken={"INITIAL_MODE"})
    assert ensure_sim_mode_params(vehicle, 2) == ["FLTMODE6", "INITIAL_MODE"]
