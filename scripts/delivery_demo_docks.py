"""The hand demo's docks d1 and d2 as owner mission waypoints (sim only).

The GCS map draws a sim dock at each owner waypoint its ``targ_wps`` marks, and
each NavPy builds its sim world from the same waypoints. The delivery eval keeps
the shared dock world (``NAVPY_SIM_DOCKS``) instead.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping

from scripts.eval_delivery_fixture import FIXTURE_PATH, delivery_scenario
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_scenario_missions import scenario_missions

#: The fixture docks the demo flies to.
DEMO_DOCKS = ("d1", "d2")


def waypoint_dock_scenario(fixture: Mapping[str, JsonValue]) -> DemoScenario:
    """The fixture scenario with ``DEMO_DOCKS`` inserted as owner NAV waypoints.

    Each sits on an owner track leg, so the path is unchanged; its 1-based NAV
    ordinal becomes a POI ordinal, which ``aas_params_for`` turns into the
    owner's ``targ_wps``. Demo only: the delivery gate profile refuses POI
    ordinals, and prep runs no delivery gate.
    """
    scenario = delivery_scenario(fixture, FIXTURE_PATH)
    missions = [dict(mission) for mission in fixture["missions"]]
    owner = missions[scenario.owner_index]
    track = list(owner["waypoints"])
    docks = {dock["name"]: dock for dock in fixture["docks"]["docks"]}
    ordinals = []
    for name in DEMO_DOCKS:
        point = {"lat": docks[name]["lat"], "lon": docks[name]["lon"]}
        index = _leg_end(track, point)
        track.insert(index, point)
        ordinals = [o + 1 if o > index else o for o in ordinals] + [index + 1]
    owner["waypoints"] = track
    names = [slot.name for slot in scenario.slots]
    return dataclasses.replace(
        scenario,
        missions=scenario_missions(missions, names, scenario.owner_slot, ordinals),
        poi_nav_waypoint_ordinals=tuple(sorted(ordinals)),
    )


def _leg_end(track: list, point: Mapping[str, float]) -> int:
    """The index of the track row that ends the leg *point* lies on."""
    for index in range(1, len(track)):
        start, end = track[index - 1], track[index]
        if start["lon"] == end["lon"] == point["lon"] and (
            min(start["lat"], end["lat"]) < point["lat"] < max(start["lat"], end["lat"])
        ):
            return index
    raise RegressionError(f"dock {point} is on no owner track leg")
