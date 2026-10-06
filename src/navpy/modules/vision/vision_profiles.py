"""Compatibility façade for vision profile loading and composition.

Focused implementations live in sibling modules; importers retain the exact
symbols and call signatures historically exposed here.
"""

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.vision_camera_calibration import (
    CameraZoomCalibration,
    parse_zoom_calibration,
    read_camera_zoom_calibration,
)
from navpy.modules.vision.vision_camera_factory import (
    build_camera_model,
    build_camera_mounts,
    build_zoom_calibration,
)
from navpy.modules.vision.vision_detector_profile import (
    DEFAULT_DETECTOR_SETTINGS,
    get_detector_settings,
)
from navpy.modules.vision.vision_gimbal_profile import (
    DEFAULT_GIMBAL_SEQ,
    DEFAULT_GIMBAL_SETUP_ATT,
    MAX_GIMBAL_DEVICE_ID,
    MIN_GIMBAL_DEVICE_ID,
    _parse_gimbal_device_id,
    build_device_gimbal_metadata,
    build_gimbal_data,
    resolve_gimbal_device_id,
    resolve_gimbal_seq,
    resolve_gimbal_setup_att,
    resolve_gimbal_setup_seq,
    validate_gimbal_device_id,
)
from navpy.modules.vision.vision_tracking_composition import build_tracking_config
from navpy.modules.vision.vision_profile_loader import (
    _merge_profile_dict,
    _profiles_path,
    _resolve_profile_definitions,
    get_devices,
    load_profiles,
    resolve_profile,
)
from navpy.modules.vision.vision_profile_types import CameraMountSpec
from navpy.modules.vision.vision_class_profile import (
    CLASS_DETECT_SIZES,
    DETECTOR_CLASS_DIMENSIONS,
    CONFIRM_Y_BUDGET,
    MIN_CONFIRM_PIXELS,
    MIN_DETECT_PIXELS,
    MIN_TRACK_PIXELS,
    _DEFAULT_DETECTOR_CLASS_DIMENSIONS,
    _DETECT_RANGE_MARGIN,
    _MAX_CLASS_SIZE,
    _MIN_CLASS_SIZE,
    _diagonal_m,
    _load_detector_class_dimensions,
    compute_approach_interval,
    compute_confirm_slant_range,
    compute_detect_slant_range,
    compute_max_detect_dist,
    get_class_detect_size,
    get_detector_class_dimensions,
    get_min_pixels_for_class,
)
from navpy.modules.vision.vision_tracking_profile import (
    VisionTrackingProfile,
    build_tracking_profile,
)
from navpy.modules.vision.vision_zoom_profile import build_zoom_config


__all__ = [
    "Attitude",
    "CLASS_DETECT_SIZES",
    "DETECTOR_CLASS_DIMENSIONS",
    "CONFIRM_Y_BUDGET",
    "CameraIntrinsics",
    "CameraMountSpec",
    "CameraZoomCalibration",
    "DEFAULT_DETECTOR_SETTINGS",
    "DEFAULT_GIMBAL_SEQ",
    "DEFAULT_GIMBAL_SETUP_ATT",
    "GimbalData",
    "MAX_GIMBAL_DEVICE_ID",
    "MIN_CONFIRM_PIXELS",
    "MIN_DETECT_PIXELS",
    "MIN_GIMBAL_DEVICE_ID",
    "MIN_TRACK_PIXELS",
    "VisionTrackingProfile",
    "build_camera_model",
    "build_camera_mounts",
    "build_device_gimbal_metadata",
    "build_gimbal_data",
    "build_tracking_config",
    "build_tracking_profile",
    "build_zoom_calibration",
    "build_zoom_config",
    "compute_approach_interval",
    "compute_confirm_slant_range",
    "compute_detect_slant_range",
    "compute_max_detect_dist",
    "get_class_detect_size",
    "get_detector_class_dimensions",
    "get_devices",
    "get_detector_settings",
    "get_min_pixels_for_class",
    "load_profiles",
    "parse_zoom_calibration",
    "read_camera_zoom_calibration",
    "resolve_gimbal_device_id",
    "resolve_gimbal_seq",
    "resolve_gimbal_setup_att",
    "resolve_gimbal_setup_seq",
    "resolve_profile",
    "validate_gimbal_device_id",
]
