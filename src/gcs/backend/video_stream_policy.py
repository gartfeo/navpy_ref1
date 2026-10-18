"""Which announced video hosts are accepted and which WHEP port is played.

From settings (docs/design/gcs_video_streams.md, sections 2 and 6):
``video.allowed_networks`` plus loopback in sim mode only, and
``video.whep_port`` unless sim mode runs this slot's test source, whose WHEP
port is slot-derived (``instance_ports.sim_video_whep_port``).
"""
from __future__ import annotations

import ipaddress
from dataclasses import dataclass
from typing import Optional

from gcs.backend import instance_ports
from gcs.backend.settings_model import GcsSettings
from gcs.backend.settings_store import settings_store
from gcs.backend.video_stream_uri import IpNetwork

# The sim MediaMTX listens on loopback; real aircraft never do.
SIM_LOOPBACK_NETWORK = ipaddress.ip_network("127.0.0.0/8")


@dataclass(frozen=True)
class VideoPolicy:
    networks: tuple[IpNetwork, ...]
    whep_port: int


def video_policy(settings: GcsSettings, chat: Optional[int]) -> VideoPolicy:
    sim = settings.simulation.sim_mode
    networks = tuple(ipaddress.ip_network(n) for n in settings.video.allowed_networks)
    if sim:
        networks += (SIM_LOOPBACK_NETWORK,)
    whep_port = settings.video.whep_port
    if sim and settings.simulation.video_test_source and chat is not None:
        whep_port = instance_ports.sim_video_whep_port(chat)
    return VideoPolicy(networks, whep_port)


def current_video_policy() -> VideoPolicy:
    """The policy from the live settings and this backend's slot."""
    return video_policy(settings_store.get(), instance_ports.chat_index())
