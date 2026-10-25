"""Step 2 of deterministic_dev_sim.md: wind and sensor noise on the serial-step SITLs."""

from __future__ import annotations

import math

import pytest

from navpy.sim.lockstep import sitl_models as sm
from navpy.sim.lockstep.participant_budget import MODELS as PARTICIPANT_MODELS
from navpy.sim.lockstep.radio_link import radio_loss_pct
from navpy.sim.lockstep.sim_profile import CATALOGUE, ModelDraw, ModelEntry, ParamRange, Realization, \
    load_profile, model_stream, realize

WIND = ModelDraw("wind", True, {"speed_mps": 6.5, "direction_deg": 270.0, "turbulence_mps": 1.25})
NOISE = ModelDraw("sensor_noise", True, {"gps_noise_m": 0.5})
BAUD = ModelDraw("baud_limit", True)
LOSS = ModelDraw("uart_loss", True, {"loss_pct": 0.15})


def realization(*models: ModelDraw, seed: int = 7) -> Realization:
    return Realization("t", "0" * 64, seed, tuple(sorted(models, key=lambda m: m.name)))


def test_every_parameter_of_the_built_models_has_one_sitl_parameter():
    """Every model but ``uart_loss``, the coordinator's radio loss (``radio_link``, todo 114),
    and the NavPy participant models (``participant_budget``, step 4)."""
    assert set(sm.SITL_PARAMETERS) == set(CATALOGUE) - {"uart_loss", *PARTICIPANT_MODELS}
    for name, params in sm.SITL_PARAMETERS.items():
        assert set(params) == set(CATALOGUE[name].params)
    assert set(sm.SITL_SWITCHES) <= set(CATALOGUE)
    names = [p for params in sm.SITL_PARAMETERS.values() for p in params.values()]
    names += [p for switches in sm.SITL_SWITCHES.values() for p in switches]
    assert len(names) == len(set(names)) and all(p.startswith("SIM_") for p in names)
    assert set(sm.RANDOM_MODELS) <= set(CATALOGUE)


def test_an_on_model_sets_its_sitl_parameters_and_an_off_or_unlisted_one_sets_none():
    assert sm.sitl_parameters(realization(WIND, NOISE)) == {
        "SIM_GPS_NOISE": 0.5, "SIM_WIND_DIR": 270.0, "SIM_WIND_SPD": 6.5, "SIM_WIND_TURB": 1.25}
    assert list(sm.sitl_parameters(realization(WIND, NOISE))) == sorted(sm.sitl_parameters(
        realization(WIND, NOISE)))
    off = ModelDraw("wind", False, {"speed_mps": 6.5})  # an off model sets nothing, whatever it carries
    assert sm.sitl_parameters(realization(off, NOISE)) == {"SIM_GPS_NOISE": 0.5}
    assert sm.sitl_parameters(realization(WIND)) == {
        "SIM_WIND_DIR": 270.0, "SIM_WIND_SPD": 6.5, "SIM_WIND_TURB": 1.25}
    assert sm.sitl_parameters(realization()) == {}
    assert sm.sitl_parameters(realize(load_profile("none"), 1)) == {}


def test_the_line_rate_is_sitls_own_parameter_and_byte_loss_is_the_radios():
    """``baud_limit`` turns on SITL's line rate, whatever its (empty) draw. ``uart_loss`` sets no
    SITL parameter (``SIM_UART_LOSS`` would break serial0 too): it is the radio's loss in percent.
    Off, neither sets anything, so the firmware default (0) holds."""
    assert sm.sitl_parameters(realization(BAUD)) == {"SIM_BAUDLIMIT_EN": 1.0}
    assert sm.sitl_parameters(realization(LOSS)) == {}
    assert sm.sitl_parameters(realization(BAUD, LOSS, NOISE)) == {
        "SIM_BAUDLIMIT_EN": 1.0, "SIM_GPS_NOISE": 0.5}
    assert radio_loss_pct(realization(LOSS)) == 0.15
    off = (ModelDraw("baud_limit", False), ModelDraw("uart_loss", False, {"loss_pct": 0.15}))
    assert sm.sitl_parameters(realization(*off)) == {}
    assert radio_loss_pct(realization(*off)) == 0.0 == radio_loss_pct(realization())


def test_neither_byte_loss_nor_the_line_rate_seeds_the_sitl():
    """The coordinator draws the radio's loss; the line rate draws nothing."""
    assert sm.sitl_rng_seed(realization(LOSS, seed=7), 2) is None
    assert sm.sitl_rng_seed(realization(BAUD, seed=7), 2) is None
    assert sm.sitl_rng_seed(realization(BAUD, ModelDraw("uart_loss", False), seed=7), 2) is None


def test_the_rng_seed_is_the_noise_stream_for_the_sysid_while_any_sitl_model_is_on():
    """Turbulence draws from the same glibc ``rand()`` as the noise, so wind alone seeds it too."""
    r = model_stream(7, "sensor_noise", "sitl.2").random()
    seed = 1 + math.floor(r * (2**32 - 1))
    off_noise = ModelDraw("sensor_noise", False)
    off_wind = ModelDraw("wind", False, {"speed_mps": 6.5})
    assert sm.sitl_rng_seed(realization(NOISE, seed=7), 2) == seed
    assert sm.sitl_rng_seed(realization(off_noise, WIND, seed=7), 2) == seed
    assert sm.sitl_rng_seed(realization(WIND, seed=7), 2) == seed
    assert sm.sitl_rng_seeds(realization(WIND, seed=7), (2,)) == {2: seed}
    assert sm.sitl_rng_seed(realization(off_noise, off_wind), 2) is None
    assert sm.sitl_rng_seed(realization(), 2) is None
    assert sm.sitl_rng_seeds(realization(off_noise, off_wind), (1, 2, 3)) == {}


def test_noise_seeds_are_per_sysid_and_per_run_seed_and_ignore_the_other_draws():
    seeds = sm.sitl_rng_seeds(realization(NOISE, seed=1), (3, 1, 2))
    assert list(seeds) == [1, 2, 3] and len(set(seeds.values())) == 3
    assert seeds == sm.sitl_rng_seeds(realization(NOISE, seed=1), (1, 2, 3))
    assert seeds != sm.sitl_rng_seeds(realization(NOISE, seed=2), (1, 2, 3))
    other = ModelDraw("sensor_noise", True, {"gps_noise_m": 0.9})
    assert seeds == sm.sitl_rng_seeds(realization(other, WIND, seed=1), (1, 2, 3))


@pytest.mark.parametrize(("r", "seed"), [(0.0, 1), (1.0 - 2**-53, 2**32 - 1)])
def test_the_noise_seed_spans_one_to_two_to_the_32_minus_one(monkeypatch, r, seed):
    """The firmware refuses 0 and anything past 32 bits."""
    monkeypatch.setattr(sm, "model_stream", lambda *_a: type("S", (), {"random": lambda self: r})())
    assert sm.sitl_rng_seed(realization(NOISE), 1) == seed


def test_the_shipped_profiles_draw_wind_and_noise_in_the_designs_ranges():
    """Noise always on (off, with wind off, would only replay the firmware's one fixed sequence),
    ``SIM_GPS_NOISE`` 0-1 m. Wind 0-8 m/s from any direction with turbulence 0-2 m/s,
    on half the runs, except in ``quiet``: the wall runs it is calibrated against
    fly without wind. ``none`` lists no model."""
    assert load_profile("none").models == {}
    for name in ("quiet", "hw", "stress"):
        models = {m: e for m, e in load_profile(name).models.items() if m in ("wind", "sensor_noise")}
        assert set(models) == ({"sensor_noise"} if name == "quiet" else {"wind", "sensor_noise"})
        assert models["sensor_noise"].p_on == 1.0
        assert models["sensor_noise"].params == {"gps_noise_m": ParamRange(0.0, 1.0)}
    for name in ("hw", "stress"):
        wind = load_profile(name).models["wind"]
        assert wind.p_on == 0.5
        assert wind.params == {"speed_mps": ParamRange(0.0, 8.0),
                               "direction_deg": ParamRange(0.0, 360.0),
                               "turbulence_mps": ParamRange(0.0, 2.0)}
    draws = [realize(load_profile("hw"), seed) for seed in range(1, 201)]
    assert all(d.model("sensor_noise").on for d in draws)
    assert 70 < sum(d.model("wind").on for d in draws) < 130
    for d in draws:
        params = sm.sitl_parameters(d)
        assert 0.0 <= params["SIM_GPS_NOISE"] <= 1.0
        if d.model("wind").on:
            assert 0.0 <= params["SIM_WIND_SPD"] <= 8.0 and 0.0 <= params["SIM_WIND_DIR"] <= 360.0
            assert 0.0 <= params["SIM_WIND_TURB"] <= 2.0


def test_the_shipped_profiles_draw_the_line_rate_and_byte_loss():
    """The line rate in every profile but ``none``; byte loss 0-0.2 % in ``hw`` (about 0-10 % of
    50-byte packets) and 0-0.6 % in ``stress``, none in ``quiet``. ``stress`` turns both on in
    every run, so its seed-1 matrix exercises them."""
    for name in ("quiet", "hw", "stress"):
        assert load_profile(name).models["baud_limit"] == ModelEntry(
            1.0 if name == "stress" else 0.5, {})
    assert "uart_loss" not in load_profile("quiet").models
    assert load_profile("hw").models["uart_loss"] == ModelEntry(0.5, {"loss_pct": ParamRange(0.0, 0.2)})
    assert load_profile("stress").models["uart_loss"] == ModelEntry(1.0, {"loss_pct": ParamRange(0.0, 0.6)})
    stress = realize(load_profile("stress"), 1)
    assert sm.sitl_parameters(stress)["SIM_BAUDLIMIT_EN"] == 1.0
    assert "SIM_UART_LOSS" not in sm.sitl_parameters(stress) and 0.0 <= radio_loss_pct(stress) <= 0.6
    draws = [realize(load_profile("hw"), seed) for seed in range(1, 201)]
    assert 70 < sum(d.model("baud_limit").on for d in draws) < 130
    assert 70 < sum(d.model("uart_loss").on for d in draws) < 130
