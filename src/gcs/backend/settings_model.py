"""Pydantic models for GCS settings with typed defaults."""
from __future__ import annotations

from pydantic import BaseModel, Field


class FlightSettings(BaseModel):
    cruise_speed_ms: float = 22.0
    flight_budget_km: float = 80.0
    safety_reserve_km: float = 20.0
    min_launch_zone_buffer_km: float = 5.0
    overlap_fraction: float = 0.1
    altitude_separation_m: float = 20.0
    uavs_per_set: int = 3
    # Climb-out target that ends the NAV_TAKEOFF phase (m, relative). The
    # aircraft continues climbing to cruise/zone altitude while flying the
    # first leg, so this is a low safe height, not the mission altitude.
    # Must be positive: a zero/negative climb target would upload an unsafe
    # takeoff waypoint, so reject it here (the source of truth for /upload)
    # rather than relying only on the UI to clamp.
    takeoff_altitude_m: float = Field(default=40.0, gt=0.0)


class CameraSettings(BaseModel):
    # Camera DATA (pitch, presets, intrinsics) is owned solely by the active
    # vision profile in vision_profiles.json — the single source of truth the
    # NavPy companion reads. This block holds only the operator's active
    # SELECTION; pitch_deg / dock_presets were removed to end the
    # settings<->profile duplication (config.py resolves them from the profile).
    vision_profile: str | None = None
    vision_device: str | None = None
    vision_zoom: str | None = None


class ConnectionSettings(BaseModel):
    default_device: str = "udp:0.0.0.0:15550"
    auto_connect: bool = True  # auto-discover+connect on startup (sim mode)
    heartbeat_timeout_s: float = 5.0
    telemetry_rate_hz: int = 5
    ws_broadcast_interval_s: float = 0.2
    reconnect_base_s: float = 1.0
    reconnect_max_s: float = 30.0


class MapDisplaySettings(BaseModel):
    zone_colors: list[str] = Field(default_factory=lambda: [
        "#FF4444", "#44AA44", "#4488FF", "#FFAA00", "#AA44FF", "#44DDDD",
    ])
    default_lat: float = 40.1792
    default_lon: float = 44.4991
    default_zoom: float = 12.0


DELIVERY_HUB_TYPES = ["building", "vehicle", "antenna", "operations_site", "bridge", "fuel", "other"]


class DefaultDeliveryHub(BaseModel):
    name: str
    type: str = "other"
    lat: float
    lon: float


class LaunchSettings(BaseModel):
    launch_type: str = "bungee"  # "bungee" or "container"
    esp32_host: str = "192.168.4.1"
    esp32_port: int = 80
    default_channel_map: dict[str, int] = Field(default_factory=dict)  # "sys_id" -> channel 1-6
    altitude_threshold_m: float = 10.0
    arm_timeout_s: float = 10.0  # container arm-confirm timeout
    altitude_timeout_s: float = 60.0

    # --- Per-path timing (DEV-tunable; defaults reproduce current behavior) ---
    container_settle_s: float = 0.3   # AUTO->ARM settle pause, container path
    bungee_settle_s: float = 0.0      # AUTO->ARM settle pause, bungee path (none today)
    bungee_arm_timeout_s: float = 5.0  # bungee arm-confirm timeout (hardcoded 5s today)
    container_stagger_s: float = 0.0  # inter-vehicle delay within a container
    container_gap_s: float = 0.0      # extra delay between containers (groups of UAVS_PER_CONTAINER)
    bungee_stagger_s: float = 0.0     # delay between consecutive bungee launches

    # --- Container airborne / climb confirmation (DEV-tunable; default-off) ---
    airborne_require_armed: bool = False  # also require armed to mark airborne
    airborne_require_throttle: bool = False  # require commanded throttle to confirm airborne
    airborne_min_throttle_pct: float = 20.0  # min commanded throttle % (VFR_HUD) when enabled
    min_climb_rate_ms: float = 0.0    # 0 = sustained-climb confirmation disabled
    climb_confirm_s: float = 0.0      # 0 = sustained-climb confirmation disabled

    # --- Launch readiness gates (DEV-tunable; /launch path only, not /restart) ---
    check_gps_enabled: bool = True    # GPS 3D-fix gate (fix-type threshold fixed at 3)
    check_gps_acc_enabled: bool = False  # GPS horizontal-accuracy gate (off = current behavior)
    max_gps_hacc_m: float = 1.0       # max allowed GPS horizontal accuracy (m) when enabled
    check_throttle_enabled: bool = True
    max_throttle_rc3: int = 1050      # throttle-low PWM threshold
    check_battery_enabled: bool = True
    min_battery_pct: float = 15.0
    check_prearm_enabled: bool = True  # vehicle prearm_ok gate
    block_on_unknown_battery: bool = False  # block launch when battery telemetry is missing

    # --- Launch sequence (operator-facing; NOT DEV-gated) ---
    launch_order: list[int] = Field(default_factory=list)  # ordered sys_ids; empty = discovery order

    # --- Auto preflight calibration (operator-facing; default off) ---
    # When on, START runs gyro + baro/ground-pressure calibration on disarmed
    # vehicles before arming. Off by default so no run changes unless opted in.
    auto_preflight_cal: bool = False
    preflight_cal_settle_s: float = 2.0  # wait after cal so the gyro cal finishes before arming


class SimulationSettings(BaseModel):
    sim_mode: bool = True
    dev_mode: bool = True
    # Opt-in cv2 HighGUI detector-debug window per companion. DEFAULTS OFF and
    # is decoupled from dev_mode: the window is pumped on the companion's main
    # thread, and under 3-UAV Windows load a stuck HighGUI window can freeze the
    # whole process. Managed companions never open it unless a dev explicitly
    # turns this on (e.g. to inspect a single companion's detections).
    detector_debug_window: bool = False
    # Companion connections for non-launcher runs: local UDP server binds that
    # SITL serial0 (udpclient) streams into — same 5760+10*(g-1) arithmetic as
    # instance_ports.companion_device.
    sitl_presets: list[str] = Field(default_factory=lambda: [
        "udp:0.0.0.0:5760",
        "udp:0.0.0.0:5770",
        "udp:0.0.0.0:5780",
    ])
    simulated_vehicle_count: int = 3


class GcsSettings(BaseModel):
    flight: FlightSettings = Field(default_factory=FlightSettings)
    camera: CameraSettings = Field(default_factory=CameraSettings)
    connection: ConnectionSettings = Field(default_factory=ConnectionSettings)
    map_display: MapDisplaySettings = Field(default_factory=MapDisplaySettings)
    launch: LaunchSettings = Field(default_factory=LaunchSettings)
    simulation: SimulationSettings = Field(default_factory=SimulationSettings)
    default_delivery_hubs: list[DefaultDeliveryHub] = Field(default_factory=list)
