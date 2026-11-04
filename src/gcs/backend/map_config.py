"""Machine-level map configuration the frontend reads at runtime.

The Cesium Ion access token (satellite imagery, 3D terrain) lives outside
every checkout, so each clone or worktree on a machine gets it without a
rebuild and it can never be committed: the repository is public. Sources, in
order: the ``CESIUM_ION_TOKEN`` environment variable, then the file
``~/.gcs/cesium_ion_token`` (override the path with
``GCS_CESIUM_ION_TOKEN_FILE``). The browser receives the token, so it is never
secret; restrict it in the Ion dashboard (assets:read, assets 1 and 3, the GCS
URLs).
"""
from __future__ import annotations

import os
from pathlib import Path

CESIUM_ION_TOKEN_ENV = "CESIUM_ION_TOKEN"
CESIUM_ION_TOKEN_FILE_ENV = "GCS_CESIUM_ION_TOKEN_FILE"


def cesium_ion_token_path() -> Path:
    override = os.environ.get(CESIUM_ION_TOKEN_FILE_ENV, "").strip()
    if override:
        return Path(override).expanduser()
    return Path.home() / ".gcs" / "cesium_ion_token"


def cesium_ion_token() -> str | None:
    """The configured Ion token, or None when unset, empty or unreadable."""
    token = os.environ.get(CESIUM_ION_TOKEN_ENV, "").strip()
    if token:
        return token
    try:
        token = cesium_ion_token_path().read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return token or None
