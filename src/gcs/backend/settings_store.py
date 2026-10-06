"""Singleton JSON-file persistence for GCS settings."""
from __future__ import annotations

import json
import logging
import os
import re
import threading
from pathlib import Path

from gcs.backend import instance_ports
from gcs.backend.settings_model import GcsSettings

log = logging.getLogger(__name__)

_DEFAULT_PATH = Path(__file__).resolve().parent.parent.parent.parent / "gcs_settings.json"
SETTINGS_PATH_ENV = "GCS_SETTINGS_PATH"


def _configured_path() -> Path:
    """Return the process-local settings path.

    Regression/live-test stacks must not read or rewrite a developer's shared
    ``gcs_settings.json``. An explicit environment binding keeps the global
    store isolated while preserving the historical repository path by default.
    """
    override = os.environ.get(SETTINGS_PATH_ENV, "").strip()
    return Path(override).expanduser() if override else _DEFAULT_PATH

_LEGACY_TCP_PRESET = re.compile(r"^tcp:[^:]+:(\d+)$")


def _migrate_legacy_companion_presets(data: dict) -> bool:
    """Rewrite persisted ``tcp:<host>:<companion-port>`` presets to UDP form.

    The companion transport moved from dialing SITL serial0 over TCP to a
    local UDP server bind. Persisted presets from the TCP era no longer match
    the injected per-chat values, so the settings API would keep them as
    "genuine user edits" and non-launcher runs would dial a TCP port nothing
    listens on. Only companion-band ports are rewritten; anything else in the
    list is a real user edit and is left alone. Returns True if changed.
    """
    sim = data.get("simulation") if isinstance(data, dict) else None
    presets = sim.get("sitl_presets") if isinstance(sim, dict) else None
    if not isinstance(presets, list):
        return False
    changed = False
    for i, preset in enumerate(presets):
        if not isinstance(preset, str):
            continue
        match = _LEGACY_TCP_PRESET.match(preset.strip())
        if match and instance_ports.is_companion_port(int(match.group(1))):
            presets[i] = f"udp:0.0.0.0:{match.group(1)}"
            changed = True
    return changed


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base* (mutates base)."""
    for key, val in override.items():
        if isinstance(val, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], val)
        else:
            base[key] = val
    return base


class SettingsStore:
    """Thread-safe settings store backed by a JSON file."""

    def __init__(self, path: Path | str | None = None):
        self._path = Path(path) if path is not None else _configured_path()
        self._lock = threading.Lock()
        self._settings = self._load()

    def _load(self) -> GcsSettings:
        if self._path.exists():
            try:
                # utf-8-sig also accepts a leading BOM (Windows PowerShell 5.1,
                # some editors); plain UTF-8 reads identically. A BOM must not
                # drop the file to defaults, which turn sim_mode back on.
                data = json.loads(self._path.read_text(encoding="utf-8-sig"))
            except Exception as exc:
                log.warning("Failed to read/parse settings from %s: %s", self._path, exc)
                return GcsSettings()
            # Detect-then-pop so an explicit `"aas": null` is also pruned.
            # Guard for non-dict JSON (e.g. `[]`, `"string"`, `42`) so the
            # `pop` doesn't raise AttributeError before we reach validation.
            had_legacy_aas = isinstance(data, dict) and "aas" in data
            if isinstance(data, dict):
                data.pop("aas", None)
                unknown_fields = set(data) - GcsSettings.model_fields.keys()
                if unknown_fields:
                    raise ValueError(
                        f"Unsupported settings fields in {self._path}: "
                        f"{', '.join(sorted(unknown_fields))}. "
                        "Update the file to the current settings format before starting GCS."
                    )
            migrated_presets = _migrate_legacy_companion_presets(data)
            try:
                settings = GcsSettings.model_validate(data)
            except Exception as exc:
                log.warning("Failed to validate settings from %s: %s", self._path, exc)
                return GcsSettings()
            if migrated_presets:
                # Same eager-rewrite rationale as the aas prune below: make the
                # migration deterministic and visible on disk; a write failure
                # must not discard the validated in-memory settings.
                try:
                    self._path.write_text(
                        settings.model_dump_json(indent=2),
                        encoding="utf-8",
                    )
                    log.info("Migrated legacy tcp companion presets in %s", self._path)
                except Exception as exc:
                    log.warning("Failed to rewrite %s after preset migration: %s", self._path, exc)
            if had_legacy_aas:
                # AAS parameters are autopilot-owned now; the GCS no longer
                # persists a copy. Eagerly rewrite the file so the migration
                # is deterministic and visible in the file on disk. A write
                # failure here must NOT discard the validated settings we
                # already have in memory -- log and continue.
                try:
                    self._path.write_text(
                        settings.model_dump_json(indent=2),
                        encoding="utf-8",
                    )
                    log.info("Pruned legacy `aas` key from %s", self._path)
                except Exception as exc:
                    log.warning("Failed to rewrite %s after pruning legacy aas: %s", self._path, exc)
            return settings
        return GcsSettings()

    def _save(self) -> None:
        try:
            self._path.write_text(
                self._settings.model_dump_json(indent=2),
                encoding="utf-8",
            )
        except Exception as exc:
            log.error("Failed to save settings to %s: %s", self._path, exc)

    def get(self) -> GcsSettings:
        with self._lock:
            return self._settings.model_copy(deep=True)

    def update(self, partial: dict) -> GcsSettings:
        """Deep-merge *partial* into current settings, persist, and return new state."""
        with self._lock:
            current = self._settings.model_dump()
            merged = _deep_merge(current, partial)
            self._settings = GcsSettings.model_validate(merged)
            self._save()
            return self._settings.model_copy(deep=True)

    def reset(self) -> GcsSettings:
        """Restore factory defaults, persist, and return new state."""
        with self._lock:
            self._settings = GcsSettings()
            self._save()
            return self._settings.model_copy(deep=True)


settings_store = SettingsStore()
