"""Wind and sensor noise on the serial-step SITLs (step 2 of ``deterministic_dev_sim.md``).

``wind`` is one sky for the fleet: its draw becomes SITL parameters merged into
every SITL's ``defaults.parm``. ``SIM_WIND_SPD`` holds from ``SIM_WIND_T_ALT``
(60 m) up and falls with the square root of height below it (``SIM_WIND_T``
default); ``SIM_WIND_DIR`` is where it blows from; ``SIM_WIND_TURB`` is the
turbulence (m/s), off on the ground. The firmware starts the wind 5 s after
boot and ramps it in with ``SIM_WIND_TC`` (5 s).

``sensor_noise`` seeds each SITL's noise. The firmware reads
``NAVPY_STEP_RNG_SEED`` at start, before any sensor draw, and seeds glibc
``random()``/``rand()`` and ``get_random16`` with it; SITL ``sysid``'s value
comes from the ``sensor_noise`` stream ``sitl.<sysid>``. The noise levels stay
ArduPlane's defaults. ``gps_noise_m`` is ``SIM_GPS_NOISE``: the amplitude of a
slow sine on GPS altitude (``SIM_GPS.cpp``), not random.

Turbulence draws from the same glibc ``rand()`` as the noise (``SIM_Aircraft.cpp``
``update_wind``), so either model seeds the SITL, and turning turbulence on
shifts the noise draws after it (todo 81).

A model that is off, or not in the profile, sets no parameter; with both off
the SITL runs as before step 2 (glibc's default seed 1, ``get_random16``'s
fixed seeds, no wind).
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping

from navpy.sim.lockstep.sim_profile import Realization, model_stream

WIND = "wind"
SENSOR_NOISE = "sensor_noise"
#: Each model parameter's SITL parameter.
SITL_PARAMETERS: Mapping[str, Mapping[str, str]] = {
    SENSOR_NOISE: {"gps_noise_m": "SIM_GPS_NOISE"},
    WIND: {"speed_mps": "SIM_WIND_SPD", "direction_deg": "SIM_WIND_DIR",
           "turbulence_mps": "SIM_WIND_TURB"},
}
#: ``NAVPY_STEP_RNG_SEED`` is 1..2**32-1: ``srandom`` takes 32 bits and the firmware refuses 0.
RNG_SEED_MAX = 2**32 - 1


def sitl_parameters(realization: Realization) -> dict[str, float]:
    """The SITL parameters the run's draw sets, sorted; empty while every model is off."""
    parameters: dict[str, float] = {}
    for model, names in SITL_PARAMETERS.items():
        draw = realization.model(model)
        if draw is not None and draw.on:
            parameters.update({names[param]: float(value) for param, value in draw.params.items()})
    return dict(sorted(parameters.items()))


def sitl_rng_seed(realization: Realization, sysid: int) -> int | None:
    """``NAVPY_STEP_RNG_SEED`` for SITL ``sysid``; None while every SITL model is off."""
    draws = (realization.model(model) for model in SITL_PARAMETERS)
    if not any(draw is not None and draw.on for draw in draws):
        return None
    r = model_stream(realization.seed, SENSOR_NOISE, f"sitl.{sysid}").random()
    return 1 + math.floor(r * RNG_SEED_MAX)


def sitl_rng_seeds(realization: Realization, sysids: Iterable[int]) -> dict[int, int]:
    """Each SITL's ``NAVPY_STEP_RNG_SEED`` by sysid; empty while every SITL model is off."""
    seeds = {sysid: sitl_rng_seed(realization, sysid) for sysid in sysids}
    return {sysid: seed for sysid, seed in sorted(seeds.items()) if seed is not None}


__all__ = ["RNG_SEED_MAX", "SENSOR_NOISE", "SITL_PARAMETERS", "WIND", "sitl_parameters",
           "sitl_rng_seed", "sitl_rng_seeds"]
