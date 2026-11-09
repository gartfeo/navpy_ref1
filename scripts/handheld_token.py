"""The Cesium Ion token deploy_handheld.py gives the handheld: found on this PC as
the backend finds it (map_config.py), put in place on the handheld under the
deploy lock. Never printed."""
from __future__ import annotations

import os
from pathlib import Path

# Where this PC's token is found, as the backend finds it (map_config.py).
CESIUM_ION_TOKEN_ENV = "CESIUM_ION_TOKEN"
CESIUM_ION_TOKEN_FILE_ENV = "GCS_CESIUM_ION_TOKEN_FILE"


def local_cesium_token() -> str | None:
    """This PC's Cesium Ion token, found as the backend finds it (map_config.py), or None."""
    token = os.environ.get(CESIUM_ION_TOKEN_ENV, "").strip()
    if token:
        return token
    override = os.environ.get(CESIUM_ION_TOKEN_FILE_ENV, "").strip()
    path = Path(override).expanduser() if override else Path.home() / ".gcs" / "cesium_ion_token"
    try:
        token = path.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    return token or None


def token_step(staged: str | None, active: str) -> str:
    """The locked update's token step (bash): puts this deploy's staged token
    (its own unique name) in place, mode 600, only when it differs from the
    active one (`active`); the same token leaves the active file untouched.
    It touches no other deploy's staged file, which may still be waiting for
    the lock. A missing staged file or a failed install stops the update
    before anything is changed, whatever `set -e` does inside a list.
    Never prints the token."""
    if not staged:
        return ""
    stop = "exit 1"
    return (
        f"if [ ! -f '{staged}' ]; then\n"
        f"    echo 'the staged Cesium Ion token is missing; nothing was changed' >&2\n"
        f"    {stop}\n"
        f"fi\n"
        f"if [ -f '{active}' ] && [ \"$(sha256sum < '{staged}')\" = \"$(sha256sum < '{active}')\" ]; then\n"
        f"    rm -f '{staged}' || {stop}\n"
        f"else\n"
        f"    chmod 600 '{staged}' || {stop}\n"
        f"    mv -f '{staged}' '{active}' || {stop}\n"
        f"    echo \"copied this PC's Cesium Ion token to {active}\"\n"
        f"fi\n")
