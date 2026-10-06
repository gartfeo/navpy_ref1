"""Profile-driven hardware, navigation, and detector assembly for the runner."""

from __future__ import annotations

import argparse
import time
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector import (
    Detector,
    DetectorDependencies,
    DetectorModelConfig,
    DetectorPipelineConfig,
    RealDetectorConfig,
)
from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi
from navpy.modules.vision.target_zoom_tracker import TargetZoomTrackerConfig

from scripts.python.run_gimbal_tracker_args import (
    SIYI_PROFILE_NAME,
)
from scripts.python.run_gimbal_tracker_profile import (
    SiyiRunnerProfile,
    build_deep_search_config,
    build_tracker_config,
    load_siyi_profile,
    pick_setting,
)


class ConsoleLogger:
    def info(self, message: str, *args: object) -> None:
        print(f"[INFO]  {message % args}" if args else f"[INFO]  {message}")

    def warning(self, message: str, *args: object) -> None:
        print(f"[WARN]  {message % args}" if args else f"[WARN]  {message}")

    def error(self, message: str, *args: object) -> None:
        print(f"[ERR]   {message % args}" if args else f"[ERR]   {message}")

    def debug(self, message: str, *args: object) -> None:
        del message, args


class DummyVehicle:
    @property
    def attitude(self) -> Attitude:
        return Attitude(0.0, 0.0, 0.0)

    def location(self, _relative: bool) -> None:
        return None


@dataclass(frozen=True)
class RunnerAssembly:
    mount: CameraMount
    detector_settings: dict[str, object]
    tracking_setup: GimbalTrackingSetup | None
    zoom_config: TargetZoomTrackerConfig | None
    default_stream: str


def assemble_runner(
    args: argparse.Namespace,
    logger: ConsoleLogger,
) -> RunnerAssembly:
    profile = load_siyi_profile(logger)
    mount = _mount_from_profile(profile, args.ip, args.port, logger)
    gimbal_config = profile.device.get("gimbal", {})
    ip = args.ip or gimbal_config.get("siyi_ip", "192.168.144.25")
    return RunnerAssembly(
        mount,
        profile.detector,
        profile.tracking_setup,
        profile.zoom_config,
        f"rtsp://{ip}:8554/main.264",
    )


def _mount_from_profile(
    profile: SiyiRunnerProfile,
    ip_override: str | None,
    port_override: int | None,
    logger: ConsoleLogger,
) -> CameraMount:
    gimbal_config = profile.device.get("gimbal", {})
    ip = ip_override or gimbal_config.get("siyi_ip", "192.168.144.25")
    port = port_override or gimbal_config.get("siyi_port", 37260)
    gimbal = GimbalSiyi(profile.gimbal_data, ip, port, logger)
    return CameraMount(
        name=profile.device.get("name", SIYI_PROFILE_NAME),
        camera=profile.camera,
        gimbal=gimbal,
        zoom_calibration=profile.zoom_calibration,
    )


def initialize_gimbal(
    args: argparse.Namespace,
    mount: CameraMount,
    logger: ConsoleLogger,
) -> None:
    gimbal = mount.gimbal
    gimbal.start()
    if not gimbal.is_connected():
        raise RuntimeError("Failed to connect to SIYI")
    deadline_s = time.time() + 5.0
    while time.time() < deadline_s:
        attitude = gimbal.get_data().att
        if abs(attitude.yaw) < 1.0 and abs(attitude.pitch) < 1.0:
            break
        time.sleep(0.05)
    gimbal.set_att(Attitude(args.pitch, 0.0, args.yaw))
    mount.set_zoom(str(args.zoom))
    time.sleep(0.5)
    gimbal.request_autofocus()
    logger.info("Initial gimbal: pitch=%.1f zoom=%.1f", args.pitch, args.zoom)


def build_navigation(
    args: argparse.Namespace,
    mount: CameraMount,
    logger: ConsoleLogger,
    tracking_setup: GimbalTrackingSetup | None,
    zoom_config: TargetZoomTrackerConfig | None,
) -> GimbalNavigation:
    setup = tracking_setup
    if setup is not None and args.max_rate is not None:
        setup = replace(
            setup,
            rate=replace(setup.rate, max_rate=float(args.max_rate)),
        )
    navigation = GimbalNavigation(
        mount,
        logger,
        tracking=setup,
        zoom_config=zoom_config,
        neutral_pitch_deg=float(args.pitch),
    )
    navigation.arm()
    return navigation


def build_detector(
    args: argparse.Namespace,
    mount: CameraMount,
    logger: ConsoleLogger,
    settings: Mapping[str, object],
    model_path: str,
    classes: list[int] | None,
    frame_source: str,
) -> Detector:
    return Detector(
        DetectorDependencies(DummyVehicle(), mount, logger),
        RealDetectorConfig(
            model=DetectorModelConfig(
                model_path=model_path,
                imgsz=settings.get("imgsz", 640),
                conf=pick_setting(args.conf, settings, "conf", 0.35),
                device=pick_setting(args.device, settings, "device", "cpu"),
                classes=classes,
                deep_search=build_deep_search_config(
                    args, settings, model_path, classes
                ),
                tracker=build_tracker_config(args, settings),
                appearance=settings.get("appearance"),
            ),
            pipeline=DetectorPipelineConfig(
                detect_hz=settings.get("detect_hz", 20.0),
                track_hz=settings.get("track_hz", 60.0),
                reference_height_m=settings.get("reference_height_m", 2.0),
                use_target_lock=True,
                output_mode="locked",
                frame_source=frame_source,
                auto_target_lock=False,
            ),
        ),
    )


def resolve_model_path(model_override: str | None, preset_model: str) -> str:
    if model_override is not None:
        return model_override
    repo_root = Path(__file__).resolve().parents[3]
    return str((repo_root / ".models" / preset_model).resolve())


__all__ = [
    "ConsoleLogger",
    "DummyVehicle",
    "RunnerAssembly",
    "assemble_runner",
    "build_detector",
    "build_navigation",
    "initialize_gimbal",
    "resolve_model_path",
]
