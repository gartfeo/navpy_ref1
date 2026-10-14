"""The producer fence: shutdown step 1, one stop per inventory producer.

:data:`FENCE` names how each producer of ``producer_inventory.PRODUCERS``
is stopped; ``tests/gcs/backend/test_producer_inventory.py`` requires the
two to match, so a new producer fails until it is fenced here. The fence
closes admission only: work admitted before it (a scheduled broadcast
coroutine, a running job) runs to completion or its cancellation.
"""

from __future__ import annotations

import asyncio

from gcs.backend.producer_inventory import ProducerInventory
from gcs.backend.vehicle_manager import vehicle_mgr

#: Producer -> how the fence stops it.
FENCE = {
    "http_routes": "admission closed: the request runner answers 503",
    "telemetry_loop": "stopped first, with its own stop (task cancel, then awaited)",
    "autoconnect": "task cancelled",
    "launch_tasks": "tasks cancelled",
    "manual_control": "task cancelled",
    "mission_probe": "admission closed; the cancel step cancels its scope",
    "task_confirm_listener": "subscriptions cancelled",
    "task_assign_listener": "subscriptions cancelled",
    "compass_cal_listener": "subscriptions cancelled",
    "vehicle_entry_messages": "subscriptions cancelled",
    "full_param_observer": "subscriptions cancelled",
    "route_broadcasts": "admission closed; the cancel step cancels the routes",
    "esp32_simulator": "admission closed; the cancel step cancels its trigger threads",
    "companion_processes": "never started: the launcher port replaces it under lockstep",
}
_TASKS = ("autoconnect", "launch_tasks", "manual_control")
_SUBSCRIBERS = ("task_confirm_listener", "task_assign_listener", "compass_cal_listener",
                "vehicle_entry_messages", "full_param_observer")


async def fence(inventory: ProducerInventory) -> list[asyncio.Task]:
    """Close admission and stop every producer; returns the tasks it cancelled."""
    inventory.close_admission()
    await vehicle_mgr.stop_telemetry_loop()
    cancelled = []
    for producer in _TASKS:
        for task in inventory.tasks(producer):
            if not task.done():
                task.cancel()
                cancelled.append(task)
    for producer in _SUBSCRIBERS:
        inventory.unsubscribe(producer)
    return cancelled


__all__ = ["FENCE", "fence"]
