"""Where the sim video test source finds MediaMTX, FFmpeg and a label font.

Operator-side tooling only (gcs_video_streams.md, section 6). Order: the
environment override, then PATH, then the HITL tools folder MediaMTX is
unpacked to on the dev PCs. Nothing is downloaded or installed here.
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path

MEDIAMTX_ENV = "NAVPY_MEDIAMTX"
FFMPEG_ENV = "NAVPY_FFMPEG"
FONT_ENV = "NAVPY_VIDEO_FONT"

_WINDOWS_FONT = Path("C:/Windows/Fonts/arial.ttf")
_LINUX_FONT = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")


def _override(name: str) -> Path | None:
    value = os.environ.get(name, "").strip()
    path = Path(value).expanduser() if value else None
    return path if path is not None and path.is_file() else None


def _hitl_tools_mediamtx() -> Path | None:
    base = os.environ.get("LOCALAPPDATA", "").strip()
    if not base:
        return None
    path = Path(base) / "navpy-hitl-tools" / "mediamtx" / "mediamtx.exe"
    return path if path.is_file() else None


def find_mediamtx() -> Path | None:
    """The MediaMTX executable, or None when it is not installed."""
    if (path := _override(MEDIAMTX_ENV)) is not None:
        return path
    on_path = shutil.which("mediamtx")
    return Path(on_path) if on_path else _hitl_tools_mediamtx()


def find_ffmpeg() -> Path | None:
    """The FFmpeg executable, or None when it is not installed."""
    if (path := _override(FFMPEG_ENV)) is not None:
        return path
    on_path = shutil.which("ffmpeg")
    return Path(on_path) if on_path else None


def find_font() -> Path | None:
    """A TrueType font for the test-pattern label, or None (FFmpeg default)."""
    if (path := _override(FONT_ENV)) is not None:
        return path
    for candidate in (_WINDOWS_FONT, _LINUX_FONT):
        if candidate.is_file():
            return candidate
    return None
