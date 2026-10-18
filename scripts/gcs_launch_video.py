"""Launcher glue for the slot's sim video test source (MediaMTX + FFmpeg).

Only with ``simulation.video_test_source`` on in sim mode. A failure to start
prints one line and leaves video off; the rest of the stack still runs.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Any

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry as reg
from gcs.backend.runtime_paths import gcs_log_dir
from gcs.backend.sim_video_process import start_sim_video, video_test_source_enabled


def reap_sim_video_ports(chat: int, protected: set[int]) -> list[int]:
    """Orphans of an earlier run holding this slot's three video ports."""
    return (
        reg.reap_port(ip.sim_video_rtsp_port(chat), exclude=protected)
        + reg.reap_port(ip.sim_video_whep_port(chat), exclude=protected)
        + reg.reap_port(ip.sim_video_webrtc_udp_port(chat), kind="udp", exclude=protected)
    )


def _spawn_hidden(command: list[str], cwd: Path, log_file: Path) -> "subprocess.Popen[bytes]":
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    with log_file.open("ab") as out:
        return subprocess.Popen(
            command, cwd=str(cwd), creationflags=flags,
            stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
        )


def launch_sim_video(entry: dict[str, Any], uav_count: int) -> None:
    """Start and record the slot's sim MediaMTX when the setting asks for it."""
    chat = entry["chat_index"]
    try:
        if not video_test_source_enabled():
            return
        process = start_sim_video(
            chat, uav_count, entry["frontend"], gcs_log_dir(), _spawn_hidden,
        )
    except Exception as error:  # video must never take the stack down
        print(f"  video           OFF ({error})")
        return
    if process is None:
        print("  video           OFF (MediaMTX/FFmpeg not found or failed to start; see log)")
        return
    reg.record_pids(chat, video_pid=process.pid)
    print(f"  video           sim test source, RTSP tcp {ip.sim_video_rtsp_port(chat)}, "
          f"WHEP http {ip.sim_video_whep_port(chat)}")
