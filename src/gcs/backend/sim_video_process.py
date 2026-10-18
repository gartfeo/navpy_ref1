"""Start a slot's sim video test source (MediaMTX) without risking the stack.

Any failure -- a missing binary, an unwritable config, a spawn error -- is
logged and returns None: video stays off and the rest of the stack runs.
"""
from __future__ import annotations

import logging
import subprocess
from collections.abc import Callable
from pathlib import Path

from gcs.backend import sim_video_binaries as binaries
from gcs.backend.sim_video_config import mediamtx_config

log = logging.getLogger(__name__)

Spawn = Callable[[list[str], Path, Path], "subprocess.Popen[bytes]"]


def video_test_source_enabled() -> bool:
    """Whether the stored settings ask for the sim video test source."""
    from gcs.backend.settings_store import settings_store

    simulation = settings_store.get().simulation
    return bool(simulation.sim_mode and simulation.video_test_source)


def config_path(log_dir: Path, chat: int) -> Path:
    return log_dir / f"sim_video_{chat}.yml"


def start_sim_video(
    chat: int,
    uav_count: int,
    frontend_port: int,
    log_dir: Path,
    spawn: Spawn,
) -> "subprocess.Popen[bytes] | None":
    """Spawn the slot's MediaMTX; None (logged) when video cannot start.

    *spawn(command, cwd, log_file)* starts the process with its output in
    *log_file*.
    """
    mediamtx = binaries.find_mediamtx()
    ffmpeg = binaries.find_ffmpeg()
    if mediamtx is None or ffmpeg is None:
        missing = [
            name for name, found in (("MediaMTX", mediamtx), ("FFmpeg", ffmpeg))
            if found is None
        ]
        log.warning(
            "sim video test source is on but %s not found (set %s / %s or "
            "install it); video stays off",
            " and ".join(missing), binaries.MEDIAMTX_ENV, binaries.FFMPEG_ENV,
        )
        return None
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        config = config_path(log_dir, chat)
        config.write_text(
            mediamtx_config(chat, uav_count, ffmpeg, frontend_port, binaries.find_font()),
            encoding="utf-8",
        )
        return spawn(
            [str(mediamtx), str(config)], log_dir, log_dir / f"sim_video_{chat}.out",
        )
    except Exception:
        log.exception("sim video test source failed to start; video stays off")
        return None
