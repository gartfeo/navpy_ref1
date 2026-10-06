"""Explicit model-path resolution and construction for real detectors."""

from __future__ import annotations

from pathlib import Path
from typing import cast

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.vision_detector_factory_ports import VisionDetectorNode
from navpy.modules.vision.vision_detector_profile import validate_detector_settings_keys
from navpy.modules.vision.vision_profile_types import CameraMountSpec, VisionProfile
from navpy.modules.vision.vision_tracking_composition import build_tracking_config
from navpy.modules.vision.vision_zoom_profile import build_zoom_config


def resolve_real_model_path(model_path: str, logger: ILogger) -> Path:
    if not model_path:
        logger.error(
            "--detector-model-path is required when --detector-type=real"
        )
        raise ValueError("Model path is required for real detector")
    resolved = Path(__file__).resolve().parents[4] / model_path
    if not resolved.exists():
        logger.error(f"Model path does not exist: {resolved}")
        raise ValueError(f"Model path does not exist: {resolved}")
    return resolved


class RealDetectorFactory:
    """Compose one real detector from resolved profile settings."""

    def __init__(
        self,
        vehicle: IVehicle,
        logger: ILogger,
        profile: VisionProfile,
        model_path: str,
        debug_show: bool,
    ) -> None:
        self._vehicle = vehicle
        self._logger = logger
        self._profile = profile
        self._model_path = resolve_real_model_path(model_path, logger)
        self._debug_show = bool(debug_show)

    def create(
        self,
        mount_spec: CameraMountSpec,
        detector_settings: VisionProfile,
        mount_index: int,
    ) -> VisionDetectorNode:
        validate_detector_settings_keys(detector_settings)
        try:
            from navpy.modules.vision.detector import Detector
            from navpy.modules.vision.real_detector_composition import (
                DetectorDependencies,
                DetectorGimbalConfig,
                DetectorModelConfig,
                DetectorPipelineConfig,
                RealDetectorConfig,
            )
            from navpy.modules.vision.real_detector_diagnostics import (
                DetectorDebugConfig,
            )
            from navpy.modules.vision.yolo_detector import DeviceT
        except ImportError as import_error:
            self._logger.error(
                "Failed to import Detector: "
                f"{import_error}. Install opencv-python and ultralytics."
            )
            raise

        tracking_config = build_tracking_config(
            mount_spec.device,
            sim=False,
        )
        zoom_config = build_zoom_config(
            mount_spec.device,
            self._profile,
        )
        detector = Detector(
            DetectorDependencies(
                self._vehicle,
                mount_spec.mount,
                self._logger,
            ),
            RealDetectorConfig(
                model=DetectorModelConfig(
                    model_path=str(self._model_path),
                    imgsz=int(detector_settings.get("imgsz", 640)),
                    conf=float(detector_settings.get("conf", 0.35)),
                    device=cast(
                        DeviceT,
                        detector_settings.get("device", "auto"),
                    ),
                    deep_search=cast(
                        VisionProfile | None,
                        detector_settings.get("deep_search"),
                    ),
                    tracker=cast(
                        VisionProfile | None,
                        detector_settings.get("tracker"),
                    ),
                    appearance=cast(
                        VisionProfile | None,
                        detector_settings.get("appearance"),
                    ),
                ),
                pipeline=DetectorPipelineConfig(
                    detect_hz=float(detector_settings.get("detect_hz", 15.0)),
                    track_hz=float(detector_settings.get("track_hz", 60.0)),
                    reference_height_m=float(
                        detector_settings.get("reference_height_m", 2.0)
                    ),
                    frame_source=cast(
                        int | str | None,
                        detector_settings.get("camera_index", -1),
                    ),
                    output_mode="all",
                    use_poi_lock=True,
                ),
                debug=DetectorDebugConfig(
                    show=self._debug_show,
                    window_name=f"navpy-detector-{mount_index}",
                ),
                gimbal=DetectorGimbalConfig(tracking_config, zoom_config),
            ),
        )
        self._logger.info(
            f"Created real detector for mount '{mount_spec.mount.name}' "
            f"with model: {self._model_path}"
        )
        return VisionDetectorNode(
            detector=detector,
            cleanup=detector,
            real_ui=detector,
            sim_debug=None,
        )


__all__ = ["RealDetectorFactory", "resolve_real_model_path"]
