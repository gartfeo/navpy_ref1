"""The sim video stream a GCS-managed sim companion announces.

With ``simulation.video_test_source`` on, UAV *n* of this slot announces its
own MediaMTX path (``sim_video_config.stream_uri``). The values reach the
companion through its environment; the NavPy side only reads them
(``navpy.app.perception.sim_video_test_source``).

Each start records whether its companion got the stream, for the preflight
video row (``video_stream_expected``); lockstep starts never get it.
"""
from __future__ import annotations

from gcs.backend import instance_ports as ip
from gcs.backend.settings_model import GcsSettings
from gcs.backend.sim_video_config import stream_label, stream_uri
from navpy.app.perception.sim_video_test_source import (
    SIM_VIDEO_NAME_ENV,
    SIM_VIDEO_URI_ENV,
)

# sys_id -> whether its last companion start got the test stream.
_expected_by_sys_id: dict[int, bool] = {}


def sim_video_env(settings: GcsSettings, sys_id: int) -> dict[str, str]:
    """Environment for the companion of *sys_id*; empty when the source is off."""
    simulation = settings.simulation
    chat = ip.chat_index()
    if not (simulation.sim_mode and simulation.video_test_source) or chat is None:
        return {}
    uav = sys_id - ip.VEHICLES_PER_CHAT * chat
    if not 1 <= uav <= ip.VEHICLES_PER_CHAT:
        return {}
    return {SIM_VIDEO_URI_ENV: stream_uri(chat, uav), SIM_VIDEO_NAME_ENV: stream_label(uav)}


def sim_video_start_kwargs(
    settings: GcsSettings, sys_id: int, subprocess_manager: bool,
) -> dict[str, dict[str, str]]:
    """``start`` keyword for the companion manager: ``extra_env`` only when on.

    The lockstep launcher port (not a subprocess manager) never takes it.
    """
    env = sim_video_env(settings, sys_id) if subprocess_manager else {}
    _expected_by_sys_id[sys_id] = bool(env)
    return {"extra_env": env} if env else {}


def video_stream_expected(sys_id: int) -> bool:
    """True when the last companion start of *sys_id* got the test stream."""
    return _expected_by_sys_id.get(sys_id, False)
