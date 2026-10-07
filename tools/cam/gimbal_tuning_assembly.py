"""Profile-driven hardware and detector assembly for gimbal tuning."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector import (
    Detector,
    DetectorDependencies,
    DetectorModelConfig,
    DetectorPipelineConfig,
    RealDetectorConfig,
)
from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi
from navpy.modules.vision.vision_profiles import (
    build_camera_model,
    build_gimbal_data,
    get_detector_settings,
    get_devices,
    resolve_profile,
)

from .gimbal_tuning_geometry import (
    TrackingIntrinsics,
    format_zoom,
    resolve_tracking_intrinsics,
)


class ConsoleLogger:
    def info(self, message: str, *args: Any) -> None:
        print(f"[INFO]  {message % args}" if args else f"[INFO]  {message}")

    def warning(self, message: str, *args: Any) -> None:
        print(f"[WARN]  {message % args}" if args else f"[WARN]  {message}")

    def error(self, message: str, *args: Any) -> None:
        print(f"[ERR]   {message % args}" if args else f"[ERR]   {message}")

    def debug(self, message: str, *args: Any) -> None:
        del message, args


class DummyVehicle:
    @property
    def attitude(self) -> Attitude:
        return Attitude(0.0, 0.0, 0.0)

    def location(self, _relative: bool) -> None:
        return None


class GimbalCentering(Protocol):
    def set_att(self, attitude: Attitude) -> None: ...

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None: ...


def build_mount(
    profile_name: str,
    ip_override: str | None,
    port_override: int | None,
    logger: ConsoleLogger,
) -> tuple[CameraMount, dict[str, Any], str]:
    key, profile, _ = resolve_profile(profile_name, logger)
    devices = get_devices(profile)
    if len(devices) != 1:
        raise RuntimeError(f"Profile '{key}' must define exactly one SIYI device")
    device = devices[0]
    gimbal_config = device.get("gimbal", {})
    camera_config = device.get("camera", {})
    if gimbal_config.get("type") != "siyi":
        raise RuntimeError(f"Profile '{key}' is not configured for a SIYI gimbal")
    if camera_config.get("zoom_control") not in ("siyi", "zr10"):
        raise RuntimeError(f"Profile '{key}' is not configured for SIYI zoom")
    detector_settings = get_detector_settings(profile)
    camera = build_camera_model(device, logger, log=True)
    gimbal_data = build_gimbal_data(device, 0, detector_settings)
    if gimbal_data is None:
        raise RuntimeError(f"Profile '{key}' does not define gimbal data")
    ip = ip_override or gimbal_config.get("siyi_ip", "192.168.144.25")
    port = port_override or gimbal_config.get("siyi_port", 37260)
    mount = CameraMount(
        name=device.get("name", key),
        camera=camera,
        gimbal=GimbalSiyi(gimbal_data, ip, port, logger),
    )
    return mount, detector_settings, f"rtsp://{ip}:8554/main.264"


def apply_detector_overrides(
    settings: dict[str, Any],
    confidence: float | None,
    device: str | None,
) -> dict[str, Any]:
    resolved = dict(settings)
    if confidence is not None:
        resolved["conf"] = confidence
    if device is not None:
        resolved["device"] = device
    return resolved


def capture_calibrated_intrinsics(
    mount: CameraMount,
) -> dict[float, TrackingIntrinsics]:
    levels = mount.get_zoom_levels()
    current_zoom = mount.get_current_zoom()
    calibrated: dict[float, TrackingIntrinsics] = {}
    if not levels:
        calibrated[1.0] = _capture_intrinsics(mount, 1.0)
        return calibrated
    for level in levels:
        if mount.camera.set_zoom(level) is False:
            continue
        zoom = float(level)
        calibrated[zoom] = _capture_intrinsics(mount, zoom)
    if current_zoom is not None:
        mount.camera.set_zoom(current_zoom)
    elif levels:
        mount.camera.set_zoom(levels[0])
    return calibrated


def _capture_intrinsics(
    mount: CameraMount,
    zoom: float,
) -> TrackingIntrinsics:
    k = mount.get_k()
    distortion = mount.get_dist()
    return TrackingIntrinsics(
        zoom=zoom,
        fx=float(k[0, 0]),
        fy=float(k[1, 1]),
        cx=float(k[0, 2]),
        cy=float(k[1, 2]),
        k=k.copy(),
        dist_coeffs=distortion.copy(),
        source="calibrated",
        reference_zoom=zoom,
    )


def build_detector(
    mount: CameraMount,
    frame_source: str | int,
    model_path: str,
    detector_settings: dict[str, Any],
    logger: ConsoleLogger,
) -> Detector:
    return Detector(
        DetectorDependencies(DummyVehicle(), mount, logger),
        RealDetectorConfig(
            model=DetectorModelConfig(
                model_path=model_path,
                imgsz=detector_settings.get("imgsz", 640),
                conf=detector_settings.get("conf", 0.35),
                device=detector_settings.get("device", "cpu"),
            ),
            pipeline=DetectorPipelineConfig(
                detect_hz=detector_settings.get("detect_hz", 20.0),
                track_hz=detector_settings.get("track_hz", 60.0),
                reference_height_m=detector_settings.get("reference_height_m", 2.0),
                use_poi_lock=True,
                output_mode="locked",
                frame_source=frame_source,
            ),
        ),
    )


def center_gimbal(gimbal: GimbalCentering, logger: ConsoleLogger) -> None:
    gimbal.set_att(Attitude(0.0, 0.0, 0.0))
    gimbal.set_rate(0.0, 0.0)
    logger.info("Centering gimbal")


def apply_zoom(
    mount: CameraMount,
    calibrated: dict[float, TrackingIntrinsics],
    zoom_level: float,
    logger: ConsoleLogger,
) -> TrackingIntrinsics | None:
    zoom = float(zoom_level)
    zoom_key = format_zoom(zoom)
    if zoom_key in mount.get_zoom_levels():
        if not mount.set_zoom(zoom_key):
            return None
        intrinsics = resolve_tracking_intrinsics(calibrated, zoom)
        logger.info("Zoom -> %sx (calibrated)", zoom_key)
        return intrinsics
    if not mount.gimbal.set_zoom(zoom_key):
        return None
    intrinsics = resolve_tracking_intrinsics(calibrated, zoom)
    logger.info(
        "Zoom -> %sx (estimated from %sx)",
        zoom_key,
        format_zoom(intrinsics.reference_zoom),
    )
    return intrinsics


def parse_source(raw: str) -> str | int:
    try:
        return int(raw)
    except ValueError:
        return raw


def default_model_path() -> str:
    repo_root = Path(__file__).resolve().parents[2]
    return str((repo_root / ".models" / "yolov8n-face-lindevs.pt").resolve())


__all__ = [
    "ConsoleLogger",
    "DummyVehicle",
    "GimbalCentering",
    "apply_detector_overrides",
    "apply_zoom",
    "build_detector",
    "build_mount",
    "capture_calibrated_intrinsics",
    "center_gimbal",
    "default_model_path",
    "parse_source",
]
