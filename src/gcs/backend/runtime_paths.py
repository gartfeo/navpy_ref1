"""Deployment paths the backend takes from the environment.

A desktop run keeps the historical repository-relative locations. A packaged
run (e.g. Termux on the handheld ground station) points the logs at a writable
app directory and names the built frontend the backend should serve.
"""
from __future__ import annotations

import os
from pathlib import Path

LOG_DIR_ENV = "GCS_LOG_DIR"
FRONTEND_DIST_ENV = "GCS_FRONTEND_DIST"

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _env_path(name: str) -> Path | None:
    value = os.environ.get(name, "").strip()
    return Path(value).expanduser() if value else None


def gcs_log_dir() -> Path:
    """Directory for backend.log and diagnostic sessions."""
    return _env_path(LOG_DIR_ENV) or _REPO_ROOT / ".logs" / "gcs"


def frontend_dist_dir() -> Path | None:
    """Built frontend (``npm run build`` output) to serve, or None for none."""
    return _env_path(FRONTEND_DIST_ENV)
