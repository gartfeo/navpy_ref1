"""Shared immutable and mutable values for resolved vision profiles."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import TYPE_CHECKING, Union, cast

if TYPE_CHECKING:
    from navpy.modules.vision.camera_mount import CameraMount


JsonScalar = Union[str, int, float, bool, None]
JsonValue = Union[JsonScalar, list["JsonValue"], dict[str, "JsonValue"]]
VisionProfile = dict[str, JsonValue]
FrozenJsonValue = Union[
    JsonScalar,
    tuple["FrozenJsonValue", ...],
    Mapping[str, "FrozenJsonValue"],
]
FrozenVisionProfile = Mapping[str, FrozenJsonValue]


def freeze_profile_value(value: JsonValue) -> FrozenJsonValue:
    """Return a recursively immutable snapshot of JSON-shaped profile data."""
    if isinstance(value, Mapping):
        frozen = {
            str(key): freeze_profile_value(cast(JsonValue, child))
            for key, child in value.items()
        }
        return MappingProxyType(frozen)
    if isinstance(value, (list, tuple)):
        return tuple(freeze_profile_value(cast(JsonValue, child)) for child in value)
    return value


@dataclass(frozen=True)
class CameraMountSpec:
    """Constructed mount paired with its immutable source configuration."""

    mount: "CameraMount"
    device: FrozenVisionProfile
    gimbal_device_id: int | None = None
    profile_device_index: int = 0

    def __post_init__(self) -> None:
        frozen = freeze_profile_value(cast(JsonValue, self.device))
        if not isinstance(frozen, Mapping):  # pragma: no cover - type guard
            raise TypeError("camera mount device configuration must be a mapping")
        object.__setattr__(self, "device", frozen)


__all__ = [
    "CameraMountSpec",
    "FrozenJsonValue",
    "FrozenVisionProfile",
    "JsonScalar",
    "JsonValue",
    "VisionProfile",
    "freeze_profile_value",
]
