"""Settings-to-launch policy for simulated NavPy companions."""

from __future__ import annotations

import os
import re

from gcs.backend.settings_model import GcsSettings


_ENV_LOG_LEVEL = "NAVPY_SIM_LOG_LEVEL"


def sim_log_level() -> str | None:
    value = os.environ.get(_ENV_LOG_LEVEL)
    return value.strip() if value and value.strip() else None


def detector_debug_enabled(settings: GcsSettings) -> bool:
    return bool(
        getattr(settings.simulation, "detector_debug_window", False)
    )


def selected_vision_profile(settings: GcsSettings) -> str | None:
    camera = getattr(settings, "camera", None)
    profile = getattr(camera, "vision_profile", None)
    if not isinstance(profile, str):
        return None
    profile = profile.strip()
    return profile or None


def effective_vision_profile(
    settings: GcsSettings,
    default_profile: str | None = None,
) -> str | None:
    return selected_vision_profile(settings) or (default_profile or None)


def sim_connection_from_settings(sys_id: int, settings: GcsSettings) -> str:
    from gcs.backend import instance_ports

    if instance_ports.chat_index() is not None:
        return instance_ports.companion_device(sys_id)
    presets = settings.simulation.sitl_presets
    index = sys_id - 1
    if 0 <= index < len(presets):
        return presets[index]
    last = presets[-1] if presets else "udp:0.0.0.0:14560"
    match = re.match(r"^(.+:)(\d+)$", last)
    if not match:
        return last
    port = int(match.group(2)) + (index - len(presets) + 1) * 10
    return match.group(1) + str(port)


__all__ = [
    "detector_debug_enabled",
    "effective_vision_profile",
    "selected_vision_profile",
    "sim_connection_from_settings",
    "sim_log_level",
]
