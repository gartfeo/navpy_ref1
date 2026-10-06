"""Deterministic inheritance and device-override resolution for profiles."""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy


def merge_profile_dict(base: dict, override: Mapping[str, object]) -> dict:
    merged = deepcopy(base)
    for key, value in override.items():
        current = merged.get(key)
        if isinstance(current, dict) and isinstance(value, Mapping):
            merged[key] = merge_profile_dict(current, value)
        else:
            merged[key] = deepcopy(value)
    return merged


class ProfileInheritanceResolver:
    """Resolve profile ancestry while owning cycle and memoization state."""

    def __init__(self, raw_profiles: object) -> None:
        if not isinstance(raw_profiles, Mapping):
            raise ValueError("Vision profiles must be a JSON object")
        self._raw_profiles = raw_profiles
        self._resolved: dict[str, dict] = {}
        self._resolving: list[str] = []

    def resolve_all(self) -> dict[str, dict]:
        for profile_name in self._raw_profiles:
            if not isinstance(profile_name, str) or not profile_name:
                raise ValueError("Vision profile names must be non-empty strings")
            self._resolve(profile_name)
        return self._resolved

    def _resolve(self, name: str) -> dict:
        if name in self._resolved:
            return self._resolved[name]
        if name in self._resolving:
            cycle = " -> ".join([*self._resolving, name])
            raise ValueError(f"Vision profile inheritance cycle: {cycle}")

        definition = self._definition(name)
        self._resolving.append(name)
        try:
            profile = self._base_profile(name, definition)
            profile = merge_profile_dict(
                profile,
                {
                    key: value
                    for key, value in definition.items()
                    if key not in {"extends", "device_overrides"}
                },
            )
            self._apply_device_overrides(name, profile, definition)
            self._resolved[name] = profile
            return profile
        finally:
            self._resolving.pop()

    def _definition(self, name: str) -> Mapping[str, object]:
        definition = self._raw_profiles.get(name)
        if definition is None:
            raise ValueError(f"Vision profile '{name}' not found")
        if not isinstance(definition, Mapping):
            raise ValueError(f"Vision profile '{name}' must be a JSON object")
        return definition

    def _base_profile(
        self,
        name: str,
        definition: Mapping[str, object],
    ) -> dict:
        base_name = definition.get("extends")
        if base_name is None:
            return {}
        if not isinstance(base_name, str) or not base_name.strip():
            raise ValueError(
                f"Vision profile '{name}' extends must be a non-empty string"
            )
        normalized_base = base_name.strip()
        if normalized_base not in self._raw_profiles:
            raise ValueError(
                f"Vision profile '{name}' extends unknown profile "
                f"'{normalized_base}'"
            )
        return deepcopy(self._resolve(normalized_base))

    def _apply_device_overrides(
        self,
        name: str,
        profile: dict,
        definition: Mapping[str, object],
    ) -> None:
        raw_overrides = definition.get("device_overrides", {})
        if not isinstance(raw_overrides, Mapping):
            raise ValueError(
                f"Vision profile '{name}' device_overrides must be a JSON object"
            )
        devices = profile.get("devices", [])
        if not isinstance(devices, list):
            raise ValueError(f"Vision profile '{name}' devices must be a list")
        if not all(isinstance(device, Mapping) for device in devices):
            raise ValueError(
                f"Vision profile '{name}' devices entries must be JSON objects"
            )
        indices = self._device_indices(name, devices)
        for device_name, override in raw_overrides.items():
            if device_name not in indices:
                raise ValueError(
                    f"Vision profile '{name}' overrides unknown device "
                    f"'{device_name}'"
                )
            if not isinstance(override, Mapping):
                raise ValueError(
                    f"Vision profile '{name}' override for device "
                    f"'{device_name}' must be a JSON object"
                )
            index = indices[device_name]
            devices[index] = merge_profile_dict(devices[index], override)

    @staticmethod
    def _device_indices(
        name: str,
        devices: list[Mapping[str, object]],
    ) -> dict[object, int]:
        indices: dict[object, int] = {}
        for index, device in enumerate(devices):
            device_name = device.get("name")
            if device_name in indices:
                raise ValueError(
                    f"Vision profile '{name}' has duplicate device name "
                    f"'{device_name}'"
                )
            indices[device_name] = index
        return indices


__all__ = ["ProfileInheritanceResolver", "merge_profile_dict"]
