"""FFmpeg executable discovery and bounded stream probing."""

from __future__ import annotations

import glob
import os
import shutil
import subprocess
from typing import Optional

from navpy.modules.vision.frame_capture_ports import CaptureLogger


_SUBPROCESS_FLAGS = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def _find_ffmpeg_dir() -> Optional[str]:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is not None:
        return os.path.dirname(ffmpeg)
    pattern = os.path.join(
        os.environ.get("LOCALAPPDATA", ""),
        "Microsoft",
        "WinGet",
        "Packages",
        "Gyan.FFmpeg*",
        "ffmpeg-*",
        "bin",
    )
    for directory in sorted(glob.glob(pattern), reverse=True):
        if os.path.isfile(os.path.join(directory, "ffmpeg.exe")):
            return directory
    return None


_FFMPEG_DIR: Optional[str] = _find_ffmpeg_dir()


def ffmpeg_binary(name: str) -> str:
    """Return a discovered full binary path or the PATH-resolved name."""
    if _FFMPEG_DIR is not None:
        candidate = os.path.join(_FFMPEG_DIR, name)
        if os.path.isfile(candidate):
            return candidate
        candidate_exe = candidate + ".exe"
        if os.path.isfile(candidate_exe):
            return candidate_exe
    return name


def probe_resolution(
    url: str,
    logger: CaptureLogger,
) -> Optional[tuple[int, int]]:
    """Use ffprobe to get stream resolution."""
    logger.info(f"FrameProvider: probing {url} with ffprobe ...")
    try:
        result = subprocess.run(
            [
                ffmpeg_binary("ffprobe"),
                "-v", "error", "-rtsp_transport", "udp",
                "-select_streams", "v:0",
                "-show_entries", "stream=width,height",
                "-of", "csv=p=0", url,
            ],
            capture_output=True,
            text=True,
            timeout=10,
            creationflags=_SUBPROCESS_FLAGS,
        )
        if result.returncode != 0:
            logger.warning(
                "FrameProvider: ffprobe returned "
                f"{result.returncode}: {result.stderr.strip()}"
            )
            return None
        parts = result.stdout.strip().split(",")
        if len(parts) == 2:
            width, height = int(parts[0]), int(parts[1])
            logger.info(f"FrameProvider: ffprobe detected {width}x{height}")
            return width, height
        logger.warning(
            "FrameProvider: ffprobe unexpected output: "
            f"{result.stdout.strip()}"
        )
    except FileNotFoundError:
        logger.warning(
            "FrameProvider: ffprobe not found "
            f"(tried: {ffmpeg_binary('ffprobe')})"
        )
    except subprocess.TimeoutExpired:
        logger.warning("FrameProvider: ffprobe timed out (stream unreachable?)")
    except ValueError as error:
        logger.warning(f"FrameProvider: ffprobe parse error: {error}")
    return None


def open_raw_video_process(
    url: str,
) -> Optional[subprocess.Popen[bytes]]:
    """Launch one low-latency FFmpeg raw BGR stream."""
    try:
        return subprocess.Popen(
            [
                ffmpeg_binary("ffmpeg"),
                "-hide_banner",
                "-loglevel",
                "fatal",
                "-nostdin",
                "-hwaccel",
                "auto",
                "-threads",
                "1",
                "-rtsp_transport",
                "udp",
                "-fflags",
                "nobuffer+discardcorrupt",
                "-flags",
                "low_delay",
                "-avioflags",
                "direct",
                "-max_delay",
                "0",
                "-probesize",
                "32",
                "-analyzeduration",
                "0",
                "-i",
                url,
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-an",
                "-sn",
                "-vsync",
                "passthrough",
                "-",
            ],
            stdout=subprocess.PIPE,
            stdin=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            creationflags=_SUBPROCESS_FLAGS,
        )
    except FileNotFoundError:
        return None


__all__ = [
    "_SUBPROCESS_FLAGS",
    "ffmpeg_binary",
    "open_raw_video_process",
    "probe_resolution",
]
