"""Step 2 of deterministic_dev_sim.md: wind and sensor noise on the serial-step SITLs."""

from __future__ import annotations

import math

import pytest

from navpy.sim.lockstep import sitl_models as sm
from navpy.sim.lockstep.link_model import RADIO_LINK, SERIAL_LINK
from navpy.sim.lockstep.sim_profile import CATALOGUE, ModelDraw, ParamRange, Realization, load_profile, \
    model_stream, realize

WIND = ModelDraw("wind", True, {"speed_mps": 6.5, "direction_deg": 270.0, "turbulence_mps": 1.25})
NOISE = ModelDraw("sensor_noise", True, {"gps_noise_m": 0.5})


def realization(*models: ModelDraw, seed: int = 7) -> Realization:
    return Realization("t", "0" * 64, seed, tuple(sorted(models, key=lambda m: m.name)))


def test_every_parameter_of_the_built_models_has_one_sitl_parameter():
    """Since step 3 the coordinator's link models apply the rest of the catalogue."""
    assert set(sm.SITL_PARAMETERS) | {SERIAL_LINK, RADIO_LINK} == set(CATALOGUE)
    assert not set(sm.SITL_PARAMETERS) & {SERIAL_LINK, RADIO_LINK}
    for name in sm.SITL_PARAMETERS:
        spec = CATALOGUE[name]
        assert set(sm.SITL_PARAMETERS[name]) == set(spec.params)
    names = [p for params in sm.SITL_PARAMETERS.values() for p in params.values()]
    assert len(names) == len(set(names)) and all(p.startswith("SIM_") for p in names)


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
        models = {m: e for m, e in load_profile(name).models.items() if m in sm.SITL_PARAMETERS}
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
