"""Strict checked-in scenario and resolved live mission-plan artifact."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import cast

from scripts.eval_gcs_demo_constants import SCENARIO_MANIFEST_PATH
from scripts.eval_gcs_demo_models import ThreeUavIds, positive_int
from scripts.eval_gcs_demo_ports import JsonValue
from navpy.args.navigation_poi_args import is_nav_poi_command


@dataclass(frozen=True)
class ScenarioSlot:
    name: str
    role: str


@dataclass(frozen=True)
class ScenarioManifest:
    schema_version: int
    slots: tuple[ScenarioSlot, ScenarioSlot, ScenarioSlot]
    owner_slot: str
    poi_nav_waypoint_ordinals: tuple[int, int, int]

    @property
    def owner_index(self) -> int:
        return next(index for index, slot in enumerate(self.slots) if slot.name == self.owner_slot)


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
    vehicles: tuple[ResolvedVehicle, ResolvedVehicle, ResolvedVehicle]
    pois: tuple[ResolvedPoi, ResolvedPoi, ResolvedPoi]

    @property
    def sys_ids(self) -> tuple[int, int, int]:
        return ThreeUavIds.from_values(vehicle.sys_id for vehicle in self.vehicles).values

    @property
    def owner(self) -> ResolvedVehicle:
        return next(vehicle for vehicle in self.vehicles if vehicle.slot == self.owner_slot)

    @property
    def peers(self) -> tuple[ResolvedVehicle, ResolvedVehicle]:
        peers = tuple(vehicle for vehicle in self.vehicles if vehicle.role == "peer")
        if len(peers) != 2:
            raise ValueError("resolved plan must contain exactly two peers")
        return peers  # type: ignore[return-value]


def _load_json_object(path: Path, label: str) -> dict[str, JsonValue]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid {label} {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object")
    return cast(dict[str, JsonValue], payload)


def _exact_keys(
    payload: Mapping[str, JsonValue],
    expected: set[str],
    label: str,
) -> None:
    actual = set(payload)
    if actual != expected:
        raise ValueError(f"{label} keys must be {sorted(expected)}, got {sorted(actual)}")


def load_scenario_manifest(path: Path = SCENARIO_MANIFEST_PATH) -> ScenarioManifest:
    payload = _load_json_object(path, "three-UAV scenario manifest")
    _exact_keys(
        payload,
        {"schema_version", "slots", "owner_slot", "poi_nav_waypoint_ordinals"},
        "scenario manifest",
    )
    if type(payload["schema_version"]) is not int or payload["schema_version"] != 1:
        raise ValueError("scenario manifest schema_version must be 1")
    raw_slots = payload["slots"]
    if not isinstance(raw_slots, list) or len(raw_slots) != 3:
        raise ValueError("scenario manifest requires exactly three slots")
    slots: list[ScenarioSlot] = []
    for index, raw in enumerate(raw_slots):
        if not isinstance(raw, dict):
            raise ValueError(f"scenario slot {index} must be an object")
        _exact_keys(raw, {"name", "role"}, f"scenario slot {index}")
        name, role = raw["name"], raw["role"]
        if type(name) is not str or not name or role not in {"owner", "peer"}:
            raise ValueError(f"invalid scenario slot {raw!r}")
        slots.append(ScenarioSlot(name, role))
    names = [slot.name for slot in slots]
    if len(set(names)) != 3 or [slot.role for slot in slots].count("owner") != 1:
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
    if ordinals != (3, 4, 7):
        raise ValueError("the certified scenario POI NAV waypoint ordinals must be [3, 4, 7]")
    return ScenarioManifest(1, tuple(slots), owner_slot, ordinals)  # type: ignore[arg-type]


def _coordinate(waypoint: JsonValue, ordinal: int) -> tuple[float, float]:
    if not isinstance(waypoint, dict):
        raise ValueError(f"owner NAV waypoint {ordinal} must be an object")
    try:
        raw_lat, raw_lon = waypoint["lat"], waypoint["lon"]
    except KeyError as error:
        raise ValueError(f"invalid owner NAV waypoint {ordinal}: {waypoint!r}") from error
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in (raw_lat, raw_lon)
    ):
        raise ValueError(f"invalid owner NAV waypoint {ordinal}: {waypoint!r}")
    lat, lon = float(raw_lat), float(raw_lon)
    if not math.isfinite(lat) or not math.isfinite(lon) or not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        raise ValueError(f"invalid owner NAV waypoint {ordinal} coordinate")
    return lat, lon


def _nav_waypoint_rows(
    owner_mission: Mapping[str, JsonValue],
) -> dict[int, Mapping[str, JsonValue]]:
    raw_waypoints = owner_mission.get("waypoints")
    if not isinstance(raw_waypoints, list):
        raise ValueError("owner mission waypoints must be a list")
    rows: dict[int, Mapping[str, JsonValue]] = {}
    previous_sequence = 0
    for index, raw in enumerate(raw_waypoints, start=1):
        if not isinstance(raw, Mapping):
            raise ValueError(f"owner mission waypoint row {index} must be an object")
        required = {
            "lat",
            "lon",
            "alt",
            "mission_sequence",
            "command",
            "nav_waypoint_ordinal",
        }
        if set(raw) != required:
            raise ValueError(
                f"owner mission waypoint row {index} keys must be {sorted(required)}"
            )
        sequence = positive_int("mission sequence", raw["mission_sequence"])
        ordinal = positive_int("NAV waypoint ordinal", raw["nav_waypoint_ordinal"])
        if sequence <= previous_sequence:
            raise ValueError("owner mission waypoint sequences must be increasing")
        if ordinal != index:
            raise ValueError("owner mission NAV waypoint ordinals must be contiguous")
        if not is_nav_poi_command(raw["command"]):
            raise ValueError("owner mission row is not MAV_CMD_NAV_WAYPOINT")
        previous_sequence = sequence
        rows[ordinal] = raw
    return rows


def resolve_demo_plan(
    sys_ids: ThreeUavIds,
    owner_mission: Mapping[str, JsonValue],
    manifest: ScenarioManifest | None = None,
) -> DemoMissionPlan:
    scenario = manifest or load_scenario_manifest()
    vehicles = tuple(
        ResolvedVehicle(slot.name, slot.role, sys_id)
        for slot, sys_id in zip(scenario.slots, sys_ids.values)
    )
    owner_sys_id = next(
        vehicle.sys_id for vehicle in vehicles if vehicle.slot == scenario.owner_slot
    )
    response_sys_id = positive_int(
        "owner mission response sys_id",
        owner_mission.get("sys_id"),
    )
    if response_sys_id != owner_sys_id:
        raise ValueError(
            f"owner mission response sys_id {owner_mission.get('sys_id')!r} "
            f"!= resolved owner {owner_sys_id}"
        )
    nav_rows = _nav_waypoint_rows(owner_mission)
    pois: list[ResolvedPoi] = []
    for task_id, ordinal in enumerate(scenario.poi_nav_waypoint_ordinals, start=1):
        waypoint = nav_rows.get(ordinal)
        if waypoint is None:
            raise ValueError(f"owner mission has no NAV waypoint ordinal {ordinal}")
        lat, lon = _coordinate(waypoint, ordinal)
        pois.append(ResolvedPoi(task_id, ordinal, lat, lon))
    if len({(poi.lat, poi.lon) for poi in pois}) != 3:
        raise ValueError("the certified scenario requires three unique POI coordinates")
    return DemoMissionPlan(1, scenario.schema_version, scenario.owner_slot, vehicles, tuple(pois))  # type: ignore[arg-type]


def owner_sys_id_for(sys_ids: ThreeUavIds) -> int:
    scenario = load_scenario_manifest()
    return sys_ids.values[scenario.owner_index]


def persist_resolved_plan(plan: DemoMissionPlan, path: Path) -> None:
    # Validate the JSON representation, where tuple fields become arrays.
    serialized = json.loads(json.dumps(asdict(plan)))
    validated = parse_resolved_plan(serialized)
    payload = json.dumps(asdict(validated), indent=2)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(path)


def load_resolved_plan(path: Path) -> DemoMissionPlan:
    return parse_resolved_plan(_load_json_object(path, "resolved demo mission plan"))


def parse_resolved_plan(payload: Mapping[str, JsonValue]) -> DemoMissionPlan:
    _exact_keys(payload, {"schema_version", "manifest_schema_version", "owner_slot", "vehicles", "pois"}, "resolved plan")
    if (
        type(payload["schema_version"]) is not int
        or type(payload["manifest_schema_version"]) is not int
        or payload["schema_version"] != 1
        or payload["manifest_schema_version"] != 1
    ):
        raise ValueError("resolved plan schema versions must be 1")
    manifest = load_scenario_manifest()
    if payload["owner_slot"] != manifest.owner_slot:
        raise ValueError("resolved plan owner slot does not match the manifest")
    raw_vehicles, raw_pois = payload["vehicles"], payload["pois"]
    if not isinstance(raw_vehicles, list) or not isinstance(raw_pois, list):
        raise ValueError("resolved plan vehicles and POIs must be lists")
    if len(raw_vehicles) != 3 or len(raw_pois) != 3:
        raise ValueError("resolved plan requires exactly three vehicles and POIs")
    vehicles: list[ResolvedVehicle] = []
    for index, raw in enumerate(raw_vehicles):
        if not isinstance(raw, dict):
            raise ValueError(f"resolved vehicle {index} must be an object")
        _exact_keys(raw, {"slot", "role", "sys_id"}, f"resolved vehicle {index}")
        slot = manifest.slots[index]
        if raw["slot"] != slot.name or raw["role"] != slot.role:
            raise ValueError("resolved vehicle order/roles do not match the manifest")
        vehicles.append(ResolvedVehicle(slot.name, slot.role, positive_int("sys_id", raw["sys_id"])))
    ThreeUavIds.from_values(vehicle.sys_id for vehicle in vehicles)
    pois: list[ResolvedPoi] = []
    for task_id, (raw, ordinal) in enumerate(zip(raw_pois, manifest.poi_nav_waypoint_ordinals), start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"resolved POI {task_id} must be an object")
        _exact_keys(raw, {"task_id", "nav_waypoint_ordinal", "lat", "lon"}, f"resolved POI {task_id}")
        if (
            type(raw["task_id"]) is not int
            or type(raw["nav_waypoint_ordinal"]) is not int
            or raw["task_id"] != task_id
            or raw["nav_waypoint_ordinal"] != ordinal
        ):
            raise ValueError("resolved POI order/ordinals do not match the manifest")
        lat, lon = _coordinate(raw, ordinal)
        pois.append(ResolvedPoi(task_id, ordinal, lat, lon))
    if len({(poi.lat, poi.lon) for poi in pois}) != 3:
        raise ValueError("resolved plan POI coordinates must be unique")
    return DemoMissionPlan(1, 1, manifest.owner_slot, tuple(vehicles), tuple(pois))  # type: ignore[arg-type]


__all__ = [
    "DemoMissionPlan",
    "ResolvedPoi",
    "ResolvedVehicle",
    "ScenarioManifest",
    "ScenarioSlot",
    "load_resolved_plan",
    "load_scenario_manifest",
    "owner_sys_id_for",
    "parse_resolved_plan",
    "persist_resolved_plan",
    "resolve_demo_plan",
]
