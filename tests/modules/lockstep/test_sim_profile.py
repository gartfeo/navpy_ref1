"""Sim profiles and their swarm draw (``deterministic_dev_sim.md`` step 1)."""

from __future__ import annotations

import hashlib
import random

import pytest

from navpy.sim.lockstep import sim_profile as sp
from navpy.sim.lockstep.lockstep_runtime import participant_seed
from navpy.sim.lockstep.sim_profile import ModelSpec, ParamRange, model_stream, parse_profile, realize

CATALOGUE = {
    "alpha": ModelSpec("alpha", {"ticks": "int", "rate": "float"}),
    "beta": ModelSpec("beta", {"gain": "float"}),
}
SEEDS = range(1, 401)


def profile(models, name="test", catalogue=CATALOGUE):
    return parse_profile(name, {"models": models}, catalogue)


ALPHA = {"p_on": 0.5, "params": {"ticks": [0, 4], "rate": [0.0, 0.1]}}
BETA = {"params": {"gain": [1.0, 2.0]}}


def test_a_model_stream_is_sha256_of_its_domain_seed_model_and_key():
    material = b"navpy.lockstep.model.v1:7:alpha:link.sitl1.backend"
    expected = random.Random(int.from_bytes(hashlib.sha256(material).digest(), "big")).random()
    assert model_stream(7, "alpha", "link.sitl1.backend").random() == expected
    assert sp.MODEL_DOMAIN == "navpy.lockstep.model.v1"
    assert sp.SWARM_DOMAIN == "navpy.lockstep.swarm.v1"


def test_the_domains_keep_model_swarm_and_participant_streams_apart():
    model = model_stream(1, "alpha", "on").random()
    swarm = sp._swarm_random(1, "alpha", "on")
    participant = random.Random(participant_seed(1, 1)).random()
    assert len({model, swarm, participant}) == 3


@pytest.mark.parametrize("seed, model, key", [
    (True, "alpha", "k"), (1.0, "alpha", "k"), (1, "Alpha", "k"), (1, "a:b", "k"),
    (1, "alpha", "a:b"), (1, "alpha", ""), (1, "alpha", "é"),
])
def test_a_stream_refuses_a_non_int_seed_and_names_that_could_collide(seed, model, key):
    with pytest.raises((TypeError, ValueError)):
        model_stream(seed, model, key)


def test_the_same_seed_gives_the_same_realization_and_another_seed_differs():
    both = profile({"alpha": ALPHA, "beta": BETA})
    assert realize(both, 3) == realize(both, 3)
    assert any(realize(both, 3).models != realize(both, seed).models for seed in SEEDS)


def test_a_model_draw_ignores_the_other_models():
    alone, both = profile({"alpha": ALPHA}), profile({"alpha": ALPHA, "beta": BETA})
    skewed = profile({"alpha": ALPHA, "beta": {**BETA, "p_on": 0.9}})
    beta_alone = profile({"beta": BETA})
    for seed in SEEDS:
        draw = realize(alone, seed).model("alpha")
        assert realize(both, seed).model("alpha") == draw
        assert realize(skewed, seed).model("alpha") == draw
        assert realize(both, seed).model("beta") == realize(beta_alone, seed).model("beta")


def test_a_changed_p_on_moves_the_switch_but_never_the_values():
    low = profile({"alpha": {**ALPHA, "p_on": 0.3}})
    high = profile({"alpha": {**ALPHA, "p_on": 0.9}})
    both_on = 0
    for seed in SEEDS:
        a, b = realize(low, seed).model("alpha"), realize(high, seed).model("alpha")
        assert not a.on or b.on  # on at 0.3 means on at 0.9: one switch draw per seed
        if a.on and b.on:
            both_on += 1
            assert a.params == b.params
    assert both_on > 50


def test_a_parameter_draw_ignores_the_other_parameters():
    one = {"alpha": ModelSpec("alpha", {"ticks": "int"})}
    narrow = profile({"alpha": {"p_on": 1.0, "params": {"ticks": [0, 4]}}}, catalogue=one)
    wide = profile({"alpha": {"p_on": 1.0, "params": {"ticks": [0, 4], "rate": [0.0, 0.1]}}})
    for seed in SEEDS:
        assert realize(narrow, seed).model("alpha").params["ticks"] == \
            realize(wide, seed).model("alpha").params["ticks"]


def test_two_parameters_with_one_range_draw_apart():
    twin = {"gamma": ModelSpec("gamma", {"a": "float", "b": "float"})}
    entry = profile({"gamma": {"p_on": 1.0, "params": {"a": [0.0, 1.0], "b": [0.0, 1.0]}}}, catalogue=twin)
    draws = [realize(entry, seed).model("gamma").params for seed in SEEDS]
    assert all(draw["a"] != draw["b"] for draw in draws)


def test_p_on_zero_is_never_one_is_always_and_a_half_is_about_half():
    for p_on, low, high in ((0.0, 0, 0), (1.0, 400, 400), (0.5, 160, 240)):
        entry = profile({"alpha": {**ALPHA, "p_on": p_on}})
        on = sum(realize(entry, seed).model("alpha").on for seed in SEEDS)
        assert low <= on <= high


def test_a_model_listed_without_p_on_takes_the_designs_half():
    assert profile({"beta": BETA}).models["beta"].p_on == sp.DEFAULT_P_ON == 0.5


def test_ranges_draw_within_their_bounds_reach_both_int_ends_and_fixed_values_stay():
    entry = profile({"alpha": {"p_on": 1.0, "params": {"ticks": [2, 5], "rate": 0.25}}})
    ticks = {realize(entry, seed).model("alpha").params["ticks"] for seed in SEEDS}
    assert ticks == {2, 3, 4, 5} and all(isinstance(tick, int) for tick in ticks)
    rates = {realize(entry, seed).model("alpha").params["rate"] for seed in SEEDS}
    assert rates == {0.25}
    gains = [realize(profile({"beta": {"p_on": 1.0, **BETA}}), seed).model("beta").params["gain"]
             for seed in SEEDS]
    assert all(1.0 <= gain <= 2.0 for gain in gains) and len(set(gains)) == len(gains)


def test_an_off_model_draws_no_values_and_an_unlisted_model_has_no_draw():
    entry = profile({"alpha": {**ALPHA, "p_on": 0.0}})
    draw = realize(entry, 1).model("alpha")
    assert draw.on is False and draw.params == {}
    assert realize(entry, 1).model("beta") is None


def test_the_realization_json_names_the_profile_and_every_draw():
    entry = profile({"alpha": {"p_on": 1.0, "params": {"ticks": 3, "rate": 0.5}}, "beta": {"p_on": 0.0, **BETA}})
    assert realize(entry, 1).to_json() == {
        "profile": "test", "profile_sha256": entry.sha256,
        "models": {"alpha": {"on": True, "params": {"rate": 0.5, "ticks": 3}},
                   "beta": {"on": False, "params": {}}},
    }


def test_the_profile_hash_is_its_canonical_json():
    a = parse_profile("x", {"models": {"alpha": ALPHA, "beta": BETA}}, CATALOGUE)
    b = parse_profile("x", {"models": {"beta": BETA, "alpha": dict(reversed(ALPHA.items()))}}, CATALOGUE)
    c = parse_profile("x", {"models": {"alpha": ALPHA}}, CATALOGUE)
    assert a.sha256 == b.sha256 != c.sha256


@pytest.mark.parametrize("models, message", [
    ({"gamma": BETA}, "not a built model"),
    ({"beta": {"params": {}}}, "must name exactly"),
    ({"beta": {"params": {"gain": 1.0, "extra": 1.0}}}, "must name exactly"),
    ({"beta": {"params": {"gain": [2.0, 1.0]}}}, "is above hi"),
    ({"beta": {"params": {"gain": [1.0]}}}, "a range is"),
    ({"beta": {"params": {"gain": float("nan")}}}, "finite"),
    ({"beta": {"params": {"gain": True}}}, "finite"),
    ({"alpha": {"params": {"ticks": [0, 1.5], "rate": 0.1}}}, "integers"),
    ({"alpha": {"params": {"ticks": False, "rate": 0.1}}}, "integers"),
    ({"beta": {"p_on": 1.5, **BETA}}, "within"),
    ({"beta": {"p_on": -0.1, **BETA}}, "within"),
    ({"beta": {**BETA, "seed": 3}}, "only p_on and params"),
])
def test_a_bad_profile_is_refused(models, message):
    with pytest.raises(ValueError, match=message):
        profile(models)


def test_the_none_profile_lists_no_model_and_the_file_needs_none_and_hw():
    with pytest.raises(ValueError, match="must list no model"):
        profile({"beta": BETA}, name="none")
    with pytest.raises(ValueError, match="must define"):
        sp.parse_profiles({"schema": 1, "profiles": {"none": {"models": {}}}}, CATALOGUE)
    with pytest.raises(ValueError, match="schema"):
        sp.parse_profiles({"schema": 2, "profiles": {}}, CATALOGUE)
    with pytest.raises(ValueError, match="only a models object"):
        parse_profile("x", {"models": {}, "extra": 1}, CATALOGUE)


def test_the_shipped_profiles_load_and_none_is_empty():
    for name in ("none", "quiet", "hw", "stress"):
        assert sp.load_profile(name).name == name
    assert sp.load_profile("none").models == {}
    assert sp.DEFAULT_PROFILE == "hw"
    assert sp.profile_names() == ["hw", "none", "quiet", "stress"]
    with pytest.raises(ValueError, match="unknown sim profile 'bogus'"):
        sp.load_profile("bogus")


def test_a_range_is_kept_as_a_range_and_a_fixed_value_as_a_number():
    entry = profile({"alpha": {"params": {"ticks": [1, 2], "rate": 0.1}}}).models["alpha"]
    assert entry.params == {"ticks": ParamRange(1, 2), "rate": 0.1}
