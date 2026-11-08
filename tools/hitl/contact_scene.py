"""The contact service's dock (hitl_guidance.md step 26): where Unreal built the delivered scene's board.

The service is a scenario consumer (scenario_public.ALLOWED_CONSUMERS): it reads its own private file, then Unreal's
readback (private/unreal_applied.json, scenario_scoring.APPLIED_SPEC) and takes the board centre Unreal reports, a
WGS84 latitude, longitude and ellipsoid height, as the net's centre, the same height datum as the bridge's
vehicle_state altitude (unreal_pose: MSL plus the ellipsoid offset). The readback counts only when it reproduces the
service's own scene hash (scoring's rebuild), so a stale or foreign readback is refused, not used.
"""
from __future__ import annotations

from pathlib import Path

from navpy.core.kernel.location import Location
from tools.hitl import json_checks as jc
from tools.hitl import scenario_public as sp
from tools.hitl import scenario_service as service
from tools.hitl.json_checks import RecordError
from tools.hitl.scenario_scoring import APPLIED_SPEC, UNREAL_APPLIED_FILE

CONSUMER = "contact"


def read_dock(scenario_dir: Path) -> Location:
    """The board centre Unreal built for the scene delivered to the contact service; RecordError otherwise."""
    scenario_dir = Path(scenario_dir)
    own_path = sp.consumer_file(scenario_dir, CONSUMER)
    if not own_path.is_file():
        raise RecordError([f"no {CONSUMER} file: the series must name {CONSUMER} as a consumer"])
    own = jc.read_json(own_path)
    scene = own.get("scene") if isinstance(own, dict) else None
    if not isinstance(scene, dict) or service.scene_hash(scene) != own.get("scene_hash"):
        raise RecordError([f"the {CONSUMER} file's scene does not match its hash"])
    applied = jc.read_json(scenario_dir / sp.PRIVATE_DIR / UNREAL_APPLIED_FILE)
    if not isinstance(applied, dict):
        raise RecordError([f"{UNREAL_APPLIED_FILE} must be an object"])
    jc.require(applied, APPLIED_SPEC, UNREAL_APPLIED_FILE, lambda _: [])
    move = {key: applied[key] for key in ("east_mm", "north_mm", "yaw_mdeg")}
    if not all(value == int(value) for value in move.values()):
        raise RecordError([f"{UNREAL_APPLIED_FILE} has a dock move that is not whole mm/mdeg"])
    rebuilt = service.scene_hash({**scene, "dock": {key: int(value) for key, value in move.items()}})
    if applied["scene_hash"] != own["scene_hash"] or rebuilt != own["scene_hash"]:
        raise RecordError([f"{UNREAL_APPLIED_FILE} is not the scene delivered to {CONSUMER}"])
    lat, lon, height = applied["board"]["centre_lat_lon_height_m"]
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise RecordError(["the board centre is not a latitude and longitude"])
    return Location(float(lat), float(lon), float(height), is_absolute=True)


__all__ = ["CONSUMER", "read_dock"]
