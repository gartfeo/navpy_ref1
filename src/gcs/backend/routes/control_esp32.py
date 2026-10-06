"""ESP32 status and simulator lifecycle operations."""
from __future__ import annotations

import asyncio
import logging
from gcs.backend.settings_store import SettingsStore

log = logging.getLogger("gcs.backend.routes.control")

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from navpy.tools.esp32_simulator import Esp32Simulator

async def esp32_status(*, _esp32_sim: Esp32Simulator | None, settings_store: SettingsStore) -> dict:
    """Check ESP32 health and return status.

    When the local simulator is running, always probe localhost on the
    simulator port so that "Test Connection" works without having to save
    settings first.
    """
    sim_running = _esp32_sim is not None
    settings = settings_store.get()
    ls = settings.launch

    if ls.launch_type != "container" and not sim_running:
        return {"reachable": False, "reason": "container_disabled", "simulator_running": False}

    # When the simulator is running, probe it on localhost regardless of
    # the persisted host/port — the user may not have saved yet.
    if sim_running:
        host = "127.0.0.1"
        port = _esp32_sim.port
    else:
        host = ls.esp32_host
        port = ls.esp32_port

    loop = asyncio.get_running_loop()
    from navpy.modules.comm.esp32_trigger import Esp32TriggerClient
    client = Esp32TriggerClient(host, port, timeout=3.0)
    healthy = await loop.run_in_executor(None, client.health_check)
    status = await loop.run_in_executor(None, client.get_status) if healthy else None
    return {
        "reachable": healthy,
        "status": status,
        "simulator_running": sim_running,
    }
