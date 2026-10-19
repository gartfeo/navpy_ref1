"""Sim profiles: which realism models a lockstep run turns on, drawn from its seed.

Steps 1-2 of ``docs/design/deterministic_dev_sim.md``. A profile names realism
models, each with a swarm probability ``p_on`` and its parameters, a fixed
number or a ``[lo, hi]`` range. Per run seed the swarm draw decides which
models are on and draws a value inside each range. The result, the
``Realization``, is recorded in the input manifest.

Every draw has its own stream, keyed by domain, seed, model and key, so turning
one model on or off, changing a ``p_on``, or adding a model or a parameter never
shifts another draw. The on/off draw is always made. Values come from
``random()`` alone, whose sequence Python keeps across versions for an int seed;
``randrange`` and ``randint`` are not kept.

Only models in the catalogue may appear in a profile. A model joins
``CATALOGUE``, and its entries join ``sim_profiles.json``, in the step that
builds it, so a run never records a model that nothing applies.
"""

from __future__ import annotations

import hashlib
import json
import math
import random
import re
from collections.abc import Mapping
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Literal, Union

PROFILES_FILE = Path(__file__).with_name("sim_profiles.json")
PROFILES_SCHEMA = 1
#: Domain tag of the swarm draws (on/off and parameter values).
SWARM_DOMAIN = "navpy.lockstep.swarm.v1"
#: Domain tag of a built model's own run-time streams (``model_stream``).
MODEL_DOMAIN = "navpy.lockstep.model.v1"
#: The design's swarm probability: a model listed without ``p_on`` is on in half the seeds
#: (Groce et al., "Swarm Testing", ISSTA 2012).
DEFAULT_P_ON = 0.5
#: The profile with no model: every model off, the run as before step 1.
NONE_PROFILE = "none"
DEFAULT_PROFILE = "hw"

_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
_KEY = re.compile(r"[A-Za-z0-9._-]+\Z")

Number = Union[int, float]
ParamKind = Literal["int", "float"]


@dataclass(frozen=True)
class ModelSpec:
    """A built model: its parameters and their kinds (``int`` for ticks)."""

    name: str
    params: Mapping[str, ParamKind]


#: The built models; each model step adds its spec here. Step 2: wind and sensor noise,
#: applied to the SITLs (``sitl_models``).
CATALOGUE: Mapping[str, ModelSpec] = {
    "sensor_noise": ModelSpec("sensor_noise", {"gps_noise_m": "float"}),
    "wind": ModelSpec("wind", {"speed_mps": "float", "direction_deg": "float", "turbulence_mps": "float"}),
}


@dataclass(frozen=True)
class ParamRange:
    lo: Number
    hi: Number


@dataclass(frozen=True)
class ModelEntry:
    p_on: float
    params: Mapping[str, Union[Number, ParamRange]]


@dataclass(frozen=True)
class SimProfile:
    name: str
    #: SHA-256 of the profile's canonical JSON: independent of line endings and of other profiles.
    sha256: str
    models: Mapping[str, ModelEntry]


@dataclass(frozen=True)
class ModelDraw:
    name: str
    on: bool
    #: The drawn values; empty for a model that is off.
    params: Mapping[str, Number] = field(default_factory=dict)


@dataclass(frozen=True)
class Realization:
    profile: str
    profile_sha256: str
    seed: int
    #: One draw per profile model, sorted by name.
    models: tuple[ModelDraw, ...]

    def model(self, name: str) -> ModelDraw | None:
        """The model's draw, or None when the profile does not list it."""
        return next((draw for draw in self.models if draw.name == name), None)

    def to_json(self) -> dict[str, object]:
        return {
            "profile": self.profile,
            "profile_sha256": self.profile_sha256,
            "models": {draw.name: {"on": draw.on, "params": dict(sorted(draw.params.items()))}
                       for draw in self.models},
        }


def _stream_seed(domain: str, seed: int, model: str, key: str) -> int:
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise TypeError(f"seed must be an int, not {type(seed).__name__}")
    if not _NAME.match(model):
        raise ValueError(f"model name {model!r} must match {_NAME.pattern}")
    if not _KEY.match(key):
        raise ValueError(f"stream key {key!r} must match {_KEY.pattern}")
    material = f"{domain}:{seed}:{model}:{key}".encode("ascii")
    return int.from_bytes(hashlib.sha256(material).digest(), "big")


def model_stream(seed: int, model: str, key: str) -> random.Random:
    """A built model's run-time stream for one link, ordinal or fault (``key``).

    SHA-256 of ``"<domain>:<seed>:<model>:<key>"`` seeds it, as ``participant_seed``
    does, under its own domain: no model stream repeats a participant's or a swarm draw's.
    """
    return random.Random(_stream_seed(MODEL_DOMAIN, seed, model, key))


def _swarm_random(seed: int, model: str, key: str) -> float:
    return random.Random(_stream_seed(SWARM_DOMAIN, seed, model, key)).random()


def _number(value: object, kind: ParamKind, where: str) -> Number:
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{where}: an int parameter needs integers, got {value!r}")
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{where}: needs a finite number, got {value!r}")
    return float(value)


def _param(value: object, kind: ParamKind, where: str) -> Union[Number, ParamRange]:
    if isinstance(value, list):
        if len(value) != 2:
            raise ValueError(f"{where}: a range is [lo, hi], got {value!r}")
        lo, hi = (_number(bound, kind, where) for bound in value)
        if lo > hi:
            raise ValueError(f"{where}: lo {lo} is above hi {hi}")
        return ParamRange(lo, hi)
    return _number(value, kind, where)


def _entry(name: str, raw: object, spec: ModelSpec, where: str) -> ModelEntry:
    if not isinstance(raw, dict) or not set(raw) <= {"p_on", "params"}:
        raise ValueError(f"{where}: a model entry has only p_on and params, got {raw!r}")
    p_on = _number(raw.get("p_on", DEFAULT_P_ON), "float", f"{where}.p_on")
    if not 0.0 <= p_on <= 1.0:
        raise ValueError(f"{where}.p_on must be within [0, 1], got {p_on}")
    params = raw.get("params", {})
    if not isinstance(params, dict) or set(params) != set(spec.params):
        raise ValueError(f"{where}.params must name exactly {sorted(spec.params)}, got {params!r}")
    return ModelEntry(p_on, {param: _param(params[param], spec.params[param], f"{where}.{param}")
                             for param in sorted(params)})


def parse_profile(name: str, raw: object, catalogue: Mapping[str, ModelSpec]) -> SimProfile:
    """A profile's JSON object, checked against the built models."""
    where = f"sim profile {name!r}"
    if not isinstance(raw, dict) or set(raw) != {"models"} or not isinstance(raw["models"], dict):
        raise ValueError(f"{where} must be an object with only a models object")
    models = {}
    for model, entry in sorted(raw["models"].items()):
        if model not in catalogue:
            raise ValueError(f"{where}: {model!r} is not a built model {sorted(catalogue)}")
        models[model] = _entry(model, entry, catalogue[model], f"{where}.{model}")
    if name == NONE_PROFILE and models:
        raise ValueError(f"sim profile {NONE_PROFILE!r} must list no model")
    text = json.dumps(raw, sort_keys=True, separators=(",", ":"))
    return SimProfile(name, hashlib.sha256(text.encode("utf-8")).hexdigest(), models)


def parse_profiles(raw: object, catalogue: Mapping[str, ModelSpec]) -> dict[str, SimProfile]:
    if not isinstance(raw, dict) or set(raw) != {"schema", "profiles"}:
        raise ValueError("sim profiles must be an object with schema and profiles")
    if raw["schema"] != PROFILES_SCHEMA or not isinstance(raw["profiles"], dict):
        raise ValueError(f"sim profiles schema must be {PROFILES_SCHEMA} with a profiles object")
    profiles = {name: parse_profile(name, entry, catalogue)
                for name, entry in sorted(raw["profiles"].items())}
    if NONE_PROFILE not in profiles or DEFAULT_PROFILE not in profiles:
        raise ValueError(f"sim profiles must define {NONE_PROFILE!r} and {DEFAULT_PROFILE!r}")
    return profiles


@lru_cache(maxsize=1)
def _built_profiles() -> dict[str, SimProfile]:
    return parse_profiles(json.loads(PROFILES_FILE.read_text(encoding="utf-8")), CATALOGUE)


def profile_names() -> list[str]:
    """The profiles in ``sim_profiles.json``, sorted: the CLIs' ``--sim-profile`` choices."""
    return sorted(_built_profiles())


def load_profile(name: str) -> SimProfile:
    """A profile from ``sim_profiles.json``; an unknown name is a ``ValueError``."""
    profiles = _built_profiles()
    if name not in profiles:
        raise ValueError(f"unknown sim profile {name!r}; known: {profile_names()}")
    return profiles[name]


def _draw_value(value: Union[Number, ParamRange], seed: int, model: str, param: str) -> Number:
    if not isinstance(value, ParamRange):
        return value
    r = _swarm_random(seed, model, f"param.{param}")
    if isinstance(value.lo, int) and isinstance(value.hi, int):
        span = value.hi - value.lo
        return value.lo + min(span, math.floor(r * (span + 1)))
    return value.lo + (value.hi - value.lo) * r


def realize(profile: SimProfile, seed: int) -> Realization:
    """The run's swarm draw: each model on with probability ``p_on``, then its values."""
    draws = []
    for name, entry in sorted(profile.models.items()):
        on = _swarm_random(seed, name, "on") < entry.p_on
        params = ({param: _draw_value(value, seed, name, param)
                   for param, value in sorted(entry.params.items())} if on else {})
        draws.append(ModelDraw(name, on, params))
    return Realization(profile.name, profile.sha256, seed, tuple(draws))


__all__ = [
    "CATALOGUE", "DEFAULT_PROFILE", "DEFAULT_P_ON", "MODEL_DOMAIN", "ModelDraw", "ModelEntry",
    "ModelSpec", "NONE_PROFILE", "PROFILES_FILE", "ParamRange", "Realization", "SWARM_DOMAIN",
    "SimProfile", "load_profile", "model_stream", "parse_profile", "parse_profiles", "profile_names",
    "realize",
]
