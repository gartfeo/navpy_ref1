"""Tests for the flight-tuner fault-injection runner (tools/tuning/sitl_faults.py), without SITL."""
from tools.tuning.sitl_driver import FTUNE_SWITCH_CH
from tools.tuning.sitl_faults import FAULTS, gain_names, same_gains


class _Tx:
    def __init__(self) -> None:
        self.channels: dict[int, int] = {}

    def set(self, channel: int, pwm: int) -> None:
        self.channels[channel] = pwm


class _FakeSitl:
    """Records what a fault does to the aircraft."""

    def __init__(self) -> None:
        self.tx = _Tx()
        self.params = {"RLL_RATE_FLTD": 10.0}
        self.modes: list[str] = []
        self.autotune: list[bool] = []

    def set_mode(self, mode: str) -> None:
        self.modes.append(mode)

    def get_param(self, name: str) -> float:
        return self.params[name]

    def set_param(self, name: str, value: float) -> None:
        self.params[name] = value

    def fly_for(self, seconds: float) -> None:
        pass


def _fault(name: str):
    return next(f for f in FAULTS if f.name == name)


def test_same_gains_uses_a_relative_tolerance():
    old = {n: 0.5 for n in gain_names("roll")}
    assert same_gains({n: 0.50001 for n in gain_names("roll")}, old)
    assert not same_gains({**old, "RLL_RATE_P": 0.51}, old)


def test_gain_names_follow_the_axis():
    assert gain_names("pitch") == ["PTCH_RATE_FF", "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D"]


def test_faults_have_distinct_names_and_cover_pitch_and_the_second_new_gain_run():
    assert len({f.name for f in FAULTS}) == len(FAULTS)
    assert any(f.axis == "pitch" for f in FAULTS)
    assert any(f.b_run == 4 for f in FAULTS)


def test_switch_and_stick_faults_move_the_right_channels_and_undo():
    sitl = _FakeSitl()
    _fault("switch off").inject(sitl)(sitl)
    assert sitl.tx.channels[FTUNE_SWITCH_CH] == 1000
    undo = _fault("stick").inject(sitl)
    assert sitl.tx.channels[1] == 1700
    undo(sitl)
    assert sitl.tx.channels[1] == 1500


def test_setting_fault_restores_the_value_it_changed():
    sitl = _FakeSitl()
    undo = _fault("roll setting").inject(sitl)
    assert sitl.params["RLL_RATE_FLTD"] != 10.0
    undo(sitl)
    assert sitl.params["RLL_RATE_FLTD"] == 10.0


def test_mode_fault_leaves_an_allowed_mode_and_returns_to_loiter():
    sitl = _FakeSitl()
    _fault("mode change").inject(sitl)(sitl)
    assert sitl.modes == ["CRUISE", "LOITER"]


def test_autotune_fault_runs_last():
    # AUTOTUNE may change IMAX and SMAX when it starts, which would disturb later faults
    assert FAULTS[-1].name == "AUTOTUNE over MAVLink"


def test_only_autotune_owns_the_gains_after_its_fault():
    # AUTOTUNE tunes while it runs; every other fault leaves the gains to the tuner's restore
    assert [f.name for f in FAULTS if not f.tuner_owns_after] == ["AUTOTUNE over MAVLink"]
