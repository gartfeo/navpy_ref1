"""API route for vision profile catalog."""
from __future__ import annotations

import json
import logging
from typing import Literal, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict

from gcs.backend import navpy_sim_runtime as runtime
from gcs.backend.settings_store import settings_store
from navpy.modules.vision.vision_profiles import (
    DETECTOR_CLASS_DIMENSIONS,
    build_device_gimbal_metadata,
    compute_confirm_slant_range,
    compute_detect_slant_range,
    get_class_detect_size,
    load_profiles,
)

log = logging.getLogger(__name__)
router = APIRouter()


class DeviceUpdate(BaseModel):
    zoom: str = "1"
    pitch_deg: Optional[float] = None
    fx: Optional[float] = None
    fy: Optional[float] = None
    image_width: Optional[int] = None
    image_height: Optional[int] = None


class ProfileUpdate(BaseModel):
    model_config = ConfigDict(extra="forbid")

    min_pitch: Optional[float] = None
    max_pitch: Optional[float] = None
    min_altitude: Optional[float] = None
    optimized_altitude: Optional[float] = None
    # Dock scan preset (altitude_m / min_pixel_size / label). Merged into
    # detector.dock_presets so the profile JSON is the single source of truth
    # for the operator's altitude + recognition size.
    dock_presets: Optional[dict[Literal["dock"], dict]] = None


def _build_catalog() -> dict:
    """Build a flat catalog of vision profiles for the GCS frontend."""
    profiles, default_profile, _ = load_profiles()
    catalog: dict[str, dict] = {}
    sim_detect_class_size = get_class_detect_size(0)  # DetectorSim publishes detection class 0
    for name, profile in profiles.items():
        devices = []
        used_gimbal_device_ids: set[int] = set()
        for device_index, device in enumerate(profile.get("devices", [])):
            cam = device.get("camera", {})
            intrinsics = cam.get("intrinsics", {})
            zooms_raw = intrinsics.get("zooms", {})
            zooms = {
                level: {
                    "fx": z["fx"],
                    "fy": z["fy"],
                    "detect_range_m": compute_detect_slant_range(z["fy"], sim_detect_class_size),
                    "confirm_range_m": compute_confirm_slant_range(z["fy"]),
                }
                for level, z in zooms_raw.items()
            }
            gimbal = device.get("gimbal", {})
            publishes_gimbal_telemetry = isinstance(gimbal, dict)
            if not isinstance(gimbal, dict):
                gimbal = {}
            pitch_deg = gimbal.get("camera_pitch", 0)
            has_gimbal = gimbal.get("type") is not None
            device_name = device.get("name", "unknown")
            gimbal_metadata = build_device_gimbal_metadata(
                device,
                device_index,
                profile_name=name,
                used_ids=used_gimbal_device_ids,
            )
            devices.append({
                "name": device_name,
                "gimbal_device_id": gimbal_metadata["gimbal_device_id"],
                "publishes_gimbal_telemetry": publishes_gimbal_telemetry,
                "image_width": cam.get("image_width", 0),
                "image_height": cam.get("image_height", 0),
                "pitch_deg": pitch_deg,
                "has_gimbal": has_gimbal,
                "setup_att": gimbal_metadata["setup_att"],
                "setup_seq": gimbal_metadata["setup_seq"],
                "gimbal_seq": gimbal_metadata["gimbal_seq"],
                "zooms": zooms,
            })
        detector = profile.get("detector", {})
        catalog[name] = {
            "devices": devices,
            "reference_height_m": detector.get("reference_height_m", 2.0),
            "imgsz": detector.get("imgsz", 640),
            "min_pitch": detector.get("min_pitch", -60),
            "max_pitch": detector.get("max_pitch", 20),
            "min_altitude": detector.get("min_altitude", 30),
            "optimized_altitude": detector.get("optimized_altitude"),
            "dock_presets": detector.get("dock_presets"),
        }
    # The per-class physical dimensions + derived diagonal size — the single
    # source the frontend uses instead of its own hardcoded class sizes.
    detector_class_dimensions = {
        str(cid): {
            "width_m": w,
            "height_m": h,
            "size_m": get_class_detect_size(cid),  # diagonal sqrt(w^2+h^2)
        }
        for cid, (w, h) in DETECTOR_CLASS_DIMENSIONS.items()
    }
    return {
        "default_profile": default_profile,
        "detector_class_dimensions": detector_class_dimensions,
        "profiles": catalog,
    }


@router.get("/api/vision-profiles")
def get_vision_profiles():
    return _build_catalog()


@router.put("/api/vision-profiles/default/{profile_name}")
def set_default_profile(profile_name: str):
    """Set the default vision profile in vision_profiles.json."""
    _, _, path = load_profiles()
    data = json.loads(path.read_text())

    if profile_name not in data.get("profiles", {}):
        raise HTTPException(404, detail=f"Profile '{profile_name}' not found")

    data["default_profile"] = profile_name
    path.write_text(json.dumps(data, indent=2) + "\n")
    _restart_navpy_if_active_profile_changed(profile_name)
    return _build_catalog()


@router.put("/api/vision-profiles/{profile_name}/devices/{device_name}")
def update_device_params(profile_name: str, device_name: str, body: DeviceUpdate):
    """Update device camera/gimbal parameters and persist to vision_profiles.json."""
    _, _, path = load_profiles()
    data = json.loads(path.read_text())

    profile = data.get("profiles", {}).get(profile_name)
    if not profile:
        raise HTTPException(404, detail=f"Profile '{profile_name}' not found")

    device = next(
        (d for d in profile.get("devices", []) if d["name"] == device_name), None
    )
    if not device:
        raise HTTPException(404, detail=f"Device '{device_name}' not found")

    cam = device.setdefault("camera", {})
    intrinsics = cam.setdefault("intrinsics", {})
    zooms = intrinsics.setdefault("zooms", {})
    zoom_data = zooms.get(body.zoom)
    if not zoom_data:
        raise HTTPException(404, detail=f"Zoom '{body.zoom}' not found")

    gimbal = device.setdefault("gimbal", {})

    if body.pitch_deg is not None:
        gimbal["camera_pitch"] = body.pitch_deg
    if body.fx is not None:
        zoom_data["fx"] = body.fx
    if body.fy is not None:
        zoom_data["fy"] = body.fy
    if body.image_width is not None:
        cam["image_width"] = body.image_width
    if body.image_height is not None:
        cam["image_height"] = body.image_height

    path.write_text(json.dumps(data, indent=2) + "\n")
    _restart_navpy_if_active_profile_changed(profile_name)
    return _build_catalog()


@router.put("/api/vision-profiles/{profile_name}")
def update_profile_params(profile_name: str, body: ProfileUpdate):
    """Update profile-level detector parameters (min/max pitch) and persist."""
    _, _, path = load_profiles()
    data = json.loads(path.read_text())

    profile = data.get("profiles", {}).get(profile_name)
    if not profile:
        raise HTTPException(404, detail=f"Profile '{profile_name}' not found")

    detector = profile.setdefault("detector", {})

    if body.min_pitch is not None:
        detector["min_pitch"] = body.min_pitch
    if body.max_pitch is not None:
        detector["max_pitch"] = body.max_pitch
    if body.min_altitude is not None:
        detector["min_altitude"] = body.min_altitude
    if body.optimized_altitude is not None:
        detector["optimized_altitude"] = body.optimized_altitude
    if body.dock_presets is not None:
        presets = detector.setdefault("dock_presets", {})
        for cls, vals in body.dock_presets.items():
            if not isinstance(vals, dict):
                continue
            cur = presets.setdefault(cls, {})
            for field in ("altitude_m", "min_pixel_size", "label"):
                if vals.get(field) is not None:
                    cur[field] = vals[field]

    path.write_text(json.dumps(data, indent=2) + "\n")
    _restart_navpy_if_active_profile_changed(profile_name)
    return _build_catalog()


def _restart_navpy_if_active_profile_changed(profile_name: str) -> None:
    """Restart running NavPy sim when the active profile content changes."""
    settings = settings_store.get()
    simulation = getattr(settings, "simulation", None)
    if not bool(getattr(simulation, "sim_mode", False)):
        return

    _, default_profile, _ = load_profiles()
    active_profile = runtime.effective_vision_profile(settings, default_profile)
    if active_profile != profile_name:
        return

    try:
        restarted = runtime.restart_running_navpy_sim(settings)
        if restarted:
            log.info(
                "Restarted NavPy sim after active vision profile edit: %s",
                restarted,
            )
    except Exception as exc:
        log.warning("Failed to restart NavPy sim after active profile edit: %s", exc)
