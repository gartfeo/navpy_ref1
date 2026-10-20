"""API route for the plan/camera compatibility check."""
from __future__ import annotations

from typing import Optional

from fastapi import APIRouter
from pydantic import BaseModel, ConfigDict, Field

from gcs.backend import navpy_sim_runtime as runtime
from gcs.backend.plan_camera_check import ZoneInput, check_plan
from gcs.backend.settings_store import settings_store
from navpy.core.perception.profile.vision_profile_loader import load_profiles

router = APIRouter()


class _Point(BaseModel):
    lat: float
    lon: float


class _Dock(_Point):
    wp: int


class _Zone(BaseModel):
    model_config = ConfigDict(extra="forbid")

    zone_index: int
    altitude_m: float
    dock_alt_m: Optional[float] = None
    use_terrain: Optional[bool] = None
    route: list[_Point] = Field(default_factory=list)
    docks: list[_Dock] = Field(default_factory=list)


class PlanCameraCheckRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    zones: list[_Zone]


@router.post("/api/plan-camera-check")
def plan_camera_check(body: PlanCameraCheckRequest) -> dict:
    settings = settings_store.get()
    profiles, default_profile, _ = load_profiles()
    name = runtime.effective_vision_profile(settings, default_profile)
    zones = [
        ZoneInput(
            zone_index=z.zone_index,
            altitude_m=z.altitude_m,
            dock_alt_m=z.dock_alt_m,
            use_terrain=z.use_terrain,
            route=[(p.lat, p.lon) for p in z.route],
            docks=[(d.lat, d.lon, d.wp) for d in z.docks],
        )
        for z in body.zones
    ]
    result = check_plan(profiles.get(name) if name else None, zones,
                        sim_mode=bool(settings.simulation.sim_mode))
    return {**result, "profile": name}
