"""Backend selection, model-path resolution and construction for real detectors."""

from __future__ import annotations

from pathlib import Path
from typing import cast

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.charuco_board import charuco_board_spec_from_settings
from navpy.args.detector_backend import (
    DetectorBackend,
    backend_requires_model_path,
    validate_detector_backend,
)
from navpy.modules.vision.vision_detector_factory_ports import VisionDetectorNode
from navpy.modules.vision.vision_detector_profile import validate_detector_settings_keys
from navpy.modules.vision.vision_profile_types import CameraMountSpec, VisionProfile
from navpy.modules.vision.vision_tracking_composition import build_tracking_config
from navpy.modules.vision.vision_zoom_profile import build_zoom_config


def resolve_real_model_path(model_path: str, logger: ILogger) -> Path:
    if not model_path:
        logger.error(
            "--detector-model-path is required when "
            "--detector-type=real --detector-backend=yolo"
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
        *,
        backend: DetectorBackend,
    ) -> None:
        self._vehicle = vehicle
        self._logger = logger
        self._profile = profile
        self._backend = validate_detector_backend(backend)
        self._model_path: Path | None = (
            resolve_real_model_path(model_path, logger)
            if backend_requires_model_path(self._backend)
            else None
        )
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
                f"{import_error}. Install opencv-python "
                "(and ultralytics for the yolo backend)."
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
                    model_path=(
                        "" if self._model_path is None
                        else str(self._model_path)
                    ),
                    backend=self._backend,
                    charuco_board=charuco_board_spec_from_settings(
                        detector_settings,
                    ),
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
            f"with backend '{self._backend}'"
            + (
                "" if self._model_path is None
                else f" model: {self._model_path}"
            )
        )
        return VisionDetectorNode(
            detector=detector,
            cleanup=detector,
            real_ui=detector,
            sim_debug=None,
        )


__all__ = ["RealDetectorFactory", "resolve_real_model_path"]
