"""GCS configuration constants and dock presets.

Module-level constants are the factory defaults. ``get_config()`` returns
a flat dict that reflects the persisted settings (if the settings store has
been loaded), falling back to these constants when unavailable.
"""

import json
import math

# Flight parameters
CRUISE_SPEED_MS = 22.0          # m/s
FLIGHT_BUDGET_KM = 80.0         # max total range per UAV (km)
SAFETY_RESERVE_KM = 20.0        # reserved for return/contingency (km)
MIN_LAUNCH_ZONE_BUFFER_KM = 5.0  # minimum launch zone standoff (km)
USABLE_FLIGHT_KM = FLIGHT_BUDGET_KM - SAFETY_RESERVE_KM - MIN_LAUNCH_ZONE_BUFFER_KM  # 55 km for survey
OVERLAP_FRACTION = 0.1          # 10% overlap between strips
ALTITUDE_SEPARATION_M = 20.0    # vertical gap between UAVs (m)
TAKEOFF_ALTITUDE_M = 40.0       # safe climb-out height ending NAV_TAKEOFF (m)

# Camera parameters (from novoxy_18_down vision profile, zoom 1)
CAMERA_FX = 2252.628
CAMERA_FY = 2262.151
CAMERA_IMAGE_WIDTH = 1920
CAMERA_IMAGE_HEIGHT = 1080
CAMERA_PITCH_DEG = -35.0        # body-relative pitch (negative = down from horizontal)
FOV_HORIZONTAL_DEG = 2 * math.degrees(math.atan(CAMERA_IMAGE_WIDTH / (2 * CAMERA_FX)))   # ~46.2°
FOV_VERTICAL_DEG = 2 * math.degrees(math.atan(CAMERA_IMAGE_HEIGHT / (2 * CAMERA_FY)))    # ~26.8°

# Fallback dock presets — used ONLY when no vision profile is resolvable
# (early import / tests / missing JSON). The authoritative source is the
# active vision profile's ``detector.dock_presets`` in vision_profiles.json;
# see ``_resolve_presets`` below.
DOCK_PRESETS = {
    "dock": {"altitude_m": 200.0, "min_pixel_size": 45, "label": "Dock"},
}

# UAV set size (fixed-wing group)
UAVS_PER_SET = 3

# Telemetry
TELEMETRY_RATE_HZ = 5
WS_BROADCAST_INTERVAL_S = 0.2  # 5 Hz to clients

# Vehicle connection
VEHICLE_RECONNECT_BASE_S = 1.0
VEHICLE_RECONNECT_MAX_S = 30.0

# Zone colors for map display
ZONE_COLORS = [
    "#FF4444",  # red
    "#44AA44",  # green
    "#4488FF",  # blue
    "#FFAA00",  # orange
    "#AA44FF",  # purple
    "#44DDDD",  # cyan
]


def _resolve_intrinsics(c):
    """Resolve camera fx/fy/width/height from vision profile."""
    profile_name = c.vision_profile
    device_name = c.vision_device
    zoom = c.vision_zoom or "1"
    if not profile_name:
        return CAMERA_FX, CAMERA_FY, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT
    try:
        from navpy.modules.vision.vision_profiles import load_profiles
        profiles, _, _ = load_profiles()
        p = profiles.get(profile_name)
        if not p:
            return CAMERA_FX, CAMERA_FY, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT
        for dev in p.get("devices", []):
            if dev.get("name") == device_name:
                cam = dev.get("camera", {})
                zooms = cam.get("intrinsics", {}).get("zooms", {})
                z = zooms.get(zoom, zooms.get("1", {}))
                return (
                    z.get("fx", CAMERA_FX),
                    z.get("fy", CAMERA_FY),
                    cam.get("image_width", CAMERA_IMAGE_WIDTH),
                    cam.get("image_height", CAMERA_IMAGE_HEIGHT),
                )
    except (KeyError, TypeError, FileNotFoundError, json.JSONDecodeError):
        pass
    return CAMERA_FX, CAMERA_FY, CAMERA_IMAGE_WIDTH, CAMERA_IMAGE_HEIGHT


def _resolve_pitch(c) -> float:
    """Resolve the camera/gimbal neutral pitch from the active vision profile.

    The vision profile (``devices[].gimbal.camera_pitch``) is the single
    source of truth — the same value NavPy reads for the mount. Falls back to
    the module constant only when no profile/device is resolvable.
    """
    profile_name = c.vision_profile
    device_name = c.vision_device
    if not profile_name:
        return CAMERA_PITCH_DEG
    try:
        from navpy.modules.vision.vision_profiles import load_profiles
        profiles, _, _ = load_profiles()
        p = profiles.get(profile_name)
        if not p:
            return CAMERA_PITCH_DEG
        devices = p.get("devices", [])
        dev = next((d for d in devices if d.get("name") == device_name), None)
        if dev is None and devices:
            dev = devices[0]  # selection stale/unset → first device's pitch
        if dev is not None:
            gimbal = dev.get("gimbal", {})
            if isinstance(gimbal, dict) and gimbal.get("camera_pitch") is not None:
                return float(gimbal["camera_pitch"])
    except (KeyError, TypeError, ValueError, FileNotFoundError, json.JSONDecodeError):
        pass
    return CAMERA_PITCH_DEG


def _resolve_presets(c) -> dict:
    """Resolve dock presets from the active vision profile's detector block.

    ``detector.dock_presets`` in vision_profiles.json is the single source of
    truth (altitudes feed the mission planner; min_pixel_size feeds the confirm
    gate). Falls back to the module constant when unresolvable.
    """
    profile_name = c.vision_profile
    if not profile_name:
        return dict(DOCK_PRESETS)
    try:
        from navpy.modules.vision.vision_profiles import load_profiles
        profiles, _, _ = load_profiles()
        p = profiles.get(profile_name)
        presets = (p or {}).get("detector", {}).get("dock_presets")
        if presets:
            return {
                k: {
                    "altitude_m": v.get("altitude_m"),
                    "min_pixel_size": v.get("min_pixel_size"),
                    "label": v.get("label", ""),
                }
                for k, v in presets.items()
            }
    except (KeyError, TypeError, FileNotFoundError, json.JSONDecodeError):
        pass
    return dict(DOCK_PRESETS)


def get_config() -> dict:
    """Return a flat config dict from the persisted settings store.

    Falls back to the module-level constants if the store is unavailable
    (e.g. during early import or in tests that don't initialise the store).
    """
    try:
        from gcs.backend.settings_store import settings_store
        s = settings_store.get()
        f = s.flight
        c = s.camera
        conn = s.connection
        fx, fy, img_w, img_h = _resolve_intrinsics(c)
        fov_h = 2 * math.degrees(math.atan(img_w / (2 * fx)))
        fov_v = 2 * math.degrees(math.atan(img_h / (2 * fy)))
        tp = _resolve_presets(c)
        return {
            "CRUISE_SPEED_MS": f.cruise_speed_ms,
            "FLIGHT_BUDGET_KM": f.flight_budget_km,
            "SAFETY_RESERVE_KM": f.safety_reserve_km,
            "MIN_LAUNCH_ZONE_BUFFER_KM": f.min_launch_zone_buffer_km,
            "USABLE_FLIGHT_KM": f.flight_budget_km - f.safety_reserve_km - f.min_launch_zone_buffer_km,
            "OVERLAP_FRACTION": f.overlap_fraction,
            "ALTITUDE_SEPARATION_M": f.altitude_separation_m,
            "TAKEOFF_ALTITUDE_M": f.takeoff_altitude_m,
            "UAVS_PER_SET": f.uavs_per_set,
            "CAMERA_FX": fx,
            "CAMERA_FY": fy,
            "CAMERA_IMAGE_WIDTH": img_w,
            "CAMERA_IMAGE_HEIGHT": img_h,
            "CAMERA_PITCH_DEG": _resolve_pitch(c),
            "FOV_HORIZONTAL_DEG": fov_h,
            "FOV_VERTICAL_DEG": fov_v,
            "DOCK_PRESETS": tp,
            "TELEMETRY_RATE_HZ": conn.telemetry_rate_hz,
            "WS_BROADCAST_INTERVAL_S": conn.ws_broadcast_interval_s,
            "VEHICLE_RECONNECT_BASE_S": conn.reconnect_base_s,
            "VEHICLE_RECONNECT_MAX_S": conn.reconnect_max_s,
            "ZONE_COLORS": s.map_display.zone_colors,
        }
    except Exception:
        return {
            "CRUISE_SPEED_MS": CRUISE_SPEED_MS,
            "FLIGHT_BUDGET_KM": FLIGHT_BUDGET_KM,
            "SAFETY_RESERVE_KM": SAFETY_RESERVE_KM,
            "MIN_LAUNCH_ZONE_BUFFER_KM": MIN_LAUNCH_ZONE_BUFFER_KM,
            "USABLE_FLIGHT_KM": USABLE_FLIGHT_KM,
            "OVERLAP_FRACTION": OVERLAP_FRACTION,
            "ALTITUDE_SEPARATION_M": ALTITUDE_SEPARATION_M,
            "TAKEOFF_ALTITUDE_M": TAKEOFF_ALTITUDE_M,
            "UAVS_PER_SET": UAVS_PER_SET,
            "CAMERA_FX": CAMERA_FX,
            "CAMERA_FY": CAMERA_FY,
            "CAMERA_IMAGE_WIDTH": CAMERA_IMAGE_WIDTH,
            "CAMERA_IMAGE_HEIGHT": CAMERA_IMAGE_HEIGHT,
            "CAMERA_PITCH_DEG": CAMERA_PITCH_DEG,
            "FOV_HORIZONTAL_DEG": FOV_HORIZONTAL_DEG,
            "FOV_VERTICAL_DEG": FOV_VERTICAL_DEG,
            "DOCK_PRESETS": DOCK_PRESETS,
            "TELEMETRY_RATE_HZ": TELEMETRY_RATE_HZ,
            "WS_BROADCAST_INTERVAL_S": WS_BROADCAST_INTERVAL_S,
            "VEHICLE_RECONNECT_BASE_S": VEHICLE_RECONNECT_BASE_S,
            "VEHICLE_RECONNECT_MAX_S": VEHICLE_RECONNECT_MAX_S,
            "ZONE_COLORS": ZONE_COLORS,
        }
