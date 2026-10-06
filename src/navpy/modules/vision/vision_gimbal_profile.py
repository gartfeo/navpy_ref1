"""Validated gimbal metadata and model construction from profile devices."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.vision_class_profile import compute_max_detect_dist


DEFAULT_GIMBAL_SETUP_ATT = [90, 0, 90]
DEFAULT_GIMBAL_SEQ = "XYZ"
MIN_GIMBAL_DEVICE_ID = 1
MAX_GIMBAL_DEVICE_ID = 6
_SUPPORTED_ROTATION_SEQUENCES = {"XYZ", "ZYX", "YZX", "ZXY", "ZYZ"}


def _parse_gimbal_device_id(raw_id: object) -> int:
    if isinstance(raw_id, bool):
        raise ValueError("gimbal_device_id must be an integer")
    if isinstance(raw_id, int):
        return raw_id
    if isinstance(raw_id, str) and raw_id.strip().isdecimal():
        return int(raw_id)
    raise ValueError("gimbal_device_id must be an integer")


def resolve_gimbal_device_id(
    gimbal: Mapping[str, object],
    device_index: int,
) -> int:
    raw_id = gimbal.get("gimbal_device_id")
    return device_index + 1 if raw_id is None else _parse_gimbal_device_id(raw_id)


def validate_gimbal_device_id(
    gimbal_device_id: int,
    profile_name: str,
    device_name: str,
    used_ids: set[int] | None = None,
) -> None:
    if not MIN_GIMBAL_DEVICE_ID <= gimbal_device_id <= MAX_GIMBAL_DEVICE_ID:
        raise ValueError(
            f"Vision profile '{profile_name}' device '{device_name}' "
            f"gimbal_device_id must be in range "
            f"{MIN_GIMBAL_DEVICE_ID}..{MAX_GIMBAL_DEVICE_ID}"
        )
    if used_ids is None:
        return
    if gimbal_device_id in used_ids:
        raise ValueError(
            f"Vision profile '{profile_name}' has duplicate "
            f"gimbal_device_id {gimbal_device_id}"
        )
    used_ids.add(gimbal_device_id)


def _finite_vector3(raw: object, name: str) -> list[float]:
    if (
        not isinstance(raw, Sequence)
        or isinstance(raw, (str, bytes, bytearray))
        or len(raw) != 3
    ):
        raise ValueError(f"{name} must contain exactly 3 numeric elements")
    values: list[float] = []
    for item in raw:
        if isinstance(item, bool):
            raise ValueError(f"{name} must contain exactly 3 finite numbers")
        try:
            value = float(item)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{name} must contain exactly 3 finite numbers"
            ) from exc
        if not math.isfinite(value):
            raise ValueError(f"{name} must contain exactly 3 finite numbers")
        values.append(value)
    return values


def resolve_gimbal_setup_att(gimbal: Mapping[str, object]) -> list[float]:
    raw = gimbal.get("setup_att", DEFAULT_GIMBAL_SETUP_ATT)
    return _finite_vector3(raw, "gimbal.setup_att")


def _resolve_rotation_sequence(
    gimbal: Mapping[str, object],
    key: str,
) -> str:
    raw = gimbal.get(key, DEFAULT_GIMBAL_SEQ)
    if (
        not isinstance(raw, str)
        or raw.upper() not in _SUPPORTED_ROTATION_SEQUENCES
    ):
        raise ValueError(
            f"gimbal.{key} must be one of "
            f"{sorted(_SUPPORTED_ROTATION_SEQUENCES)}"
        )
    return raw


def resolve_gimbal_setup_seq(gimbal: Mapping[str, object]) -> str:
    return _resolve_rotation_sequence(gimbal, "setup_seq")


def resolve_gimbal_seq(gimbal: Mapping[str, object]) -> str:
    return _resolve_rotation_sequence(gimbal, "seq")


def build_device_gimbal_metadata(
    device: Mapping[str, object],
    device_index: int,
    *,
    profile_name: str = "",
    used_ids: set[int] | None = None,
) -> dict:
    if not isinstance(device, Mapping):
        raise ValueError("Vision profile device must be a mapping")
    raw_gimbal = device.get("gimbal", {})
    if raw_gimbal is None:
        raw_gimbal = {}
    if not isinstance(raw_gimbal, Mapping):
        raise ValueError("Vision profile device gimbal must be a mapping")
    raw_name = device.get("name", f"device_{device_index}")
    if not isinstance(raw_name, str) or not raw_name:
        raise ValueError("Vision profile device name must be a non-empty string")
    gimbal_device_id = resolve_gimbal_device_id(raw_gimbal, device_index)
    validate_gimbal_device_id(
        gimbal_device_id,
        profile_name,
        raw_name,
        used_ids,
    )
    return {
        "gimbal_device_id": gimbal_device_id,
        "setup_att": resolve_gimbal_setup_att(raw_gimbal),
        "setup_seq": resolve_gimbal_setup_seq(raw_gimbal),
        "gimbal_seq": resolve_gimbal_seq(raw_gimbal),
    }


def _first_zoom_fy(device: Mapping[str, object]) -> float | None:
    camera = device.get("camera")
    if not isinstance(camera, Mapping):
        return None
    intrinsics = camera.get("intrinsics")
    if not isinstance(intrinsics, Mapping):
        return None
    zooms = intrinsics.get("zooms")
    if not isinstance(zooms, Mapping):
        return None
    first_zoom = next(iter(zooms.values()), None)
    if not isinstance(first_zoom, Mapping) or "fy" not in first_zoom:
        return None
    return float(first_zoom["fy"])


def build_gimbal_data(
    device: Mapping[str, object],
    index: int,
    detector_settings: Mapping[str, object] | None = None,
) -> GimbalData | None:
    """Build validated gimbal geometry from a profile device."""
    raw_gimbal = device.get("gimbal")
    if not isinstance(raw_gimbal, Mapping):
        return None

    setup_dist_mm = _finite_vector3(
        raw_gimbal.get("setup_dist_mm", [0, 0, 0]),
        "gimbal.setup_dist_mm",
    )
    setup_att_values = resolve_gimbal_setup_att(raw_gimbal)
    camera_pitch = float(raw_gimbal.get("camera_pitch", 0.0))
    if not math.isfinite(camera_pitch):
        raise ValueError("gimbal.camera_pitch must be finite")

    max_distance = 3000.0
    if detector_settings is not None:
        focal_y = _first_zoom_fy(device)
        if focal_y is not None:
            max_distance = compute_max_detect_dist(focal_y)

    return GimbalData(
        att=Attitude(camera_pitch, 0, 0),
        name=f"gimbal_{index}",
        roll_stabilize=bool(raw_gimbal.get("stabilize_roll", False)),
        pitch_stabilize=bool(raw_gimbal.get("stabilize_pitch", False)),
        g_seq=resolve_gimbal_seq(raw_gimbal),
        setup=GimbalMountSetup(
            att=Attitude(*setup_att_values),
            seq=resolve_gimbal_setup_seq(raw_gimbal),
            dist=[value / 1000.0 for value in setup_dist_mm],
        ),
        max_detect_distance=max_distance,
    )


__all__ = [
    "DEFAULT_GIMBAL_SEQ",
    "DEFAULT_GIMBAL_SETUP_ATT",
    "MAX_GIMBAL_DEVICE_ID",
    "MIN_GIMBAL_DEVICE_ID",
    "build_device_gimbal_metadata",
    "build_gimbal_data",
    "resolve_gimbal_device_id",
    "resolve_gimbal_seq",
    "resolve_gimbal_setup_att",
    "resolve_gimbal_setup_seq",
    "validate_gimbal_device_id",
]
