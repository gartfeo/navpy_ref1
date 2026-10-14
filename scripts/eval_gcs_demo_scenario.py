"""Strict checked-in scenario and resolved live mission-plan artifact."""

from __future__ import annotations

import json
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import cast

from scripts.eval_gcs_demo_constants import SCENARIO_MANIFEST_PATH
from scripts.eval_gcs_demo_gate_profiles import require_profile_shape
from scripts.eval_gcs_demo_models import UavIds, positive_int
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario_missions import (
    exact_keys,
    owner_nav_coordinate,
    owner_nav_waypoint_rows,
    parse_home,
    scenario_missions,
)


SCENARIO_SCHEMA_VERSION = 2
#: The CLI default: the certified three-UAV scenario.
CERTIFIED_SCENARIO_PATH = SCENARIO_MANIFEST_PATH


@dataclass(frozen=True)
class ScenarioSlot:
    name: str
    role: str


@dataclass(frozen=True)
class DemoScenario:
    """One checked scenario file: slots, POIs, missions and its gate profile."""

    schema_version: int
    gate_profile: str
    slots: tuple[ScenarioSlot, ...]
    owner_slot: str
    poi_nav_waypoint_ordinals: tuple[int, ...]
    home: str
    missions: tuple[Mapping[str, JsonValue], ...]
    #: The file it was loaded from; the input manifest hashes it.
    path: Path

    @property
    def owner_index(self) -> int:
        return next(index for index, slot in enumerate(self.slots) if slot.name == self.owner_slot)

    def uav_ids(self, values: Iterable[object]) -> UavIds:
        """One unique positive sysid per scenario slot, in slot order."""
        return UavIds.from_values(values, len(self.slots))

    def owner_sys_id(self, sys_ids: UavIds) -> int:
        """The live sysid in the owner slot's ordinal."""
        return self.uav_ids(sys_ids.values).values[self.owner_index]


@dataclass(frozen=True)
class ResolvedVehicle:
    slot: str
    role: str
    sys_id: int


@dataclass(frozen=True)
class ResolvedPoi:
    task_id: int
    nav_waypoint_ordinal: int
    lat: float
    lon: float


@dataclass(frozen=True)
class DemoMissionPlan:
    schema_version: int
    manifest_schema_version: int
    owner_slot: str
    vehicles: tuple[ResolvedVehicle, ...]
    pois: tuple[ResolvedPoi, ...]

    @property
    def sys_ids(self) -> tuple[int, ...]:
        return UavIds.from_values(
            (vehicle.sys_id for vehicle in self.vehicles), len(self.vehicles)
        ).values

    @property
    def owner(self) -> ResolvedVehicle:
        return next(vehicle for vehicle in self.vehicles if vehicle.slot == self.owner_slot)

    @property
    def peers(self) -> tuple[ResolvedVehicle, ...]:
        """Every peer-role vehicle; none in an owner-only scenario."""
        return tuple(vehicle for vehicle in self.vehicles if vehicle.role == "peer")


def _load_json_object(path: Path, label: str) -> dict[str, JsonValue]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid {label} {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object")
    return cast(dict[str, JsonValue], payload)


def load_scenario(path: Path) -> DemoScenario:
    if not isinstance(path, Path):
        raise TypeError("scenario path must be a Path")
    payload = _load_json_object(path, "demo scenario")
    exact_keys(
        payload,
        {
            "schema_version",
            "gate_profile",
            "slots",
            "owner_slot",
            "poi_nav_waypoint_ordinals",
            "home",
            "missions",
        },
        "scenario manifest",
    )
    if type(payload["schema_version"]) is not int or payload["schema_version"] != SCENARIO_SCHEMA_VERSION:
        raise ValueError(f"scenario manifest schema_version must be {SCENARIO_SCHEMA_VERSION}")
    raw_slots = payload["slots"]
    if not isinstance(raw_slots, list) or not raw_slots:
        raise ValueError("scenario manifest requires a non-empty list of slots")
    slots: list[ScenarioSlot] = []
    for index, raw in enumerate(raw_slots):
        if not isinstance(raw, dict):
            raise ValueError(f"scenario slot {index} must be an object")
        exact_keys(raw, {"name", "role"}, f"scenario slot {index}")
        name, role = raw["name"], raw["role"]
        if type(name) is not str or not name or role not in {"owner", "peer"}:
            raise ValueError(f"invalid scenario slot {raw!r}")
        slots.append(ScenarioSlot(name, role))
    names = [slot.name for slot in slots]
    if len(set(names)) != len(names) or [slot.role for slot in slots].count("owner") != 1:
        raise ValueError("scenario requires unique slots with exactly one owner")
    owner_slot = payload["owner_slot"]
    if type(owner_slot) is not str or owner_slot not in names:
        raise ValueError("scenario owner_slot must name a configured slot")
    if next(slot.role for slot in slots if slot.name == owner_slot) != "owner":
        raise ValueError("scenario owner_slot must identify the owner role")
    raw_ordinals = payload["poi_nav_waypoint_ordinals"]
    if not isinstance(raw_ordinals, list):
        raise ValueError("POI NAV waypoint ordinals must be a list")
    ordinals = tuple(positive_int("POI NAV waypoint ordinal", item) for item in raw_ordinals)
    gate_profile = require_profile_shape(
        payload["gate_profile"], [slot.role for slot in slots], ordinals
    )
    missions = scenario_missions(payload["missions"], names, owner_slot, ordinals)
    return DemoScenario(
        SCENARIO_SCHEMA_VERSION,
        gate_profile,
        tuple(slots),
        owner_slot,
        ordinals,
        parse_home(payload["home"]),
        missions,
        path,
    )


def resolve_demo_plan(
    sys_ids: UavIds,
    owner_mission: Mapping[str, JsonValue],
    scenario: DemoScenario,
) -> DemoMissionPlan:
    checked = scenario.uav_ids(sys_ids.values)
    vehicles = tuple(
        ResolvedVehicle(slot.name, slot.role, sys_id)
        for slot, sys_id in zip(scenario.slots, checked.values)
    )
    owner_sys_id = scenario.owner_sys_id(checked)
    response_sys_id = positive_int(
        "owner mission response sys_id",
        owner_mission.get("sys_id"),
    )
    if response_sys_id != owner_sys_id:
        raise ValueError(
            f"owner mission response sys_id {owner_mission.get('sys_id')!r} "
            f"!= resolved owner {owner_sys_id}"
        )
    nav_rows = owner_nav_waypoint_rows(owner_mission)
    pois: list[ResolvedPoi] = []
    for task_id, ordinal in enumerate(scenario.poi_nav_waypoint_ordinals, start=1):
        waypoint = nav_rows.get(ordinal)
        if waypoint is None:
            raise ValueError(f"owner mission has no NAV waypoint ordinal {ordinal}")
        lat, lon = owner_nav_coordinate(waypoint, ordinal)
        pois.append(ResolvedPoi(task_id, ordinal, lat, lon))
    if len({(poi.lat, poi.lon) for poi in pois}) != len(pois):
        raise ValueError("the scenario requires one unique POI coordinate per ordinal")
    return DemoMissionPlan(1, scenario.schema_version, scenario.owner_slot, vehicles, tuple(pois))


def persist_resolved_plan(plan: DemoMissionPlan, path: Path, scenario: DemoScenario) -> None:
    # Validate the JSON representation, where tuple fields become arrays.
    serialized = json.loads(json.dumps(asdict(plan)))
    validated = parse_resolved_plan(serialized, scenario)
    payload = json.dumps(asdict(validated), indent=2)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(path)


def load_resolved_plan(path: Path, scenario: DemoScenario) -> DemoMissionPlan:
    return parse_resolved_plan(
        _load_json_object(path, "resolved demo mission plan"), scenario
    )


def parse_resolved_plan(
    payload: Mapping[str, JsonValue],
    scenario: DemoScenario,
) -> DemoMissionPlan:
    exact_keys(payload, {"schema_version", "manifest_schema_version", "owner_slot", "vehicles", "pois"}, "resolved plan")
    if (
        type(payload["schema_version"]) is not int
        or type(payload["manifest_schema_version"]) is not int
        or payload["schema_version"] != 1
        or payload["manifest_schema_version"] != SCENARIO_SCHEMA_VERSION
    ):
        raise ValueError(f"resolved plan schema versions must be 1 and {SCENARIO_SCHEMA_VERSION}")
    if payload["owner_slot"] != scenario.owner_slot:
        raise ValueError("resolved plan owner slot does not match the manifest")
    raw_vehicles, raw_pois = payload["vehicles"], payload["pois"]
    if not isinstance(raw_vehicles, list) or not isinstance(raw_pois, list):
        raise ValueError("resolved plan vehicles and POIs must be lists")
    if (
        len(raw_vehicles) != len(scenario.slots)
        or len(raw_pois) != len(scenario.poi_nav_waypoint_ordinals)
    ):
        raise ValueError(
            "resolved plan requires one vehicle per scenario slot and one POI per ordinal"
        )
    vehicles: list[ResolvedVehicle] = []
    for index, raw in enumerate(raw_vehicles):
        if not isinstance(raw, dict):
            raise ValueError(f"resolved vehicle {index} must be an object")
        exact_keys(raw, {"slot", "role", "sys_id"}, f"resolved vehicle {index}")
        slot = scenario.slots[index]
        if raw["slot"] != slot.name or raw["role"] != slot.role:
            raise ValueError("resolved vehicle order/roles do not match the manifest")
        vehicles.append(ResolvedVehicle(slot.name, slot.role, positive_int("sys_id", raw["sys_id"])))
    scenario.uav_ids(vehicle.sys_id for vehicle in vehicles)
    pois: list[ResolvedPoi] = []
    for task_id, (raw, ordinal) in enumerate(zip(raw_pois, scenario.poi_nav_waypoint_ordinals), start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"resolved POI {task_id} must be an object")
        exact_keys(raw, {"task_id", "nav_waypoint_ordinal", "lat", "lon"}, f"resolved POI {task_id}")
        if (
            type(raw["task_id"]) is not int
            or type(raw["nav_waypoint_ordinal"]) is not int
            or raw["task_id"] != task_id
            or raw["nav_waypoint_ordinal"] != ordinal
        ):
            raise ValueError("resolved POI order/ordinals do not match the manifest")
        lat, lon = owner_nav_coordinate(raw, ordinal)
        pois.append(ResolvedPoi(task_id, ordinal, lat, lon))
    if len({(poi.lat, poi.lon) for poi in pois}) != len(pois):
        raise ValueError("resolved plan POI coordinates must be unique")
    return DemoMissionPlan(1, scenario.schema_version, scenario.owner_slot, tuple(vehicles), tuple(pois))


__all__ = [
    "CERTIFIED_SCENARIO_PATH",
    "DemoMissionPlan",
    "DemoScenario",
    "ResolvedPoi",
    "ResolvedVehicle",
    "ScenarioSlot",
    "load_resolved_plan",
    "load_scenario",
    "parse_resolved_plan",
    "persist_resolved_plan",
    "resolve_demo_plan",
]
