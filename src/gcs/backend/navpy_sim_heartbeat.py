"""Diagnose repeated silent-heartbeat exits of simulated companions."""

from __future__ import annotations

import logging

from navpy.modules.vehicle.vehicle_mav import (
    NO_HEARTBEAT_EXIT_MARKER,
    STARTUP_HEARTBEAT_TIMEOUT_S,
)


HB_EXIT_WINDOW_S = STARTUP_HEARTBEAT_TIMEOUT_S + 5.0
STALL_DIAGNOSIS_AFTER = 3


def track_heartbeat_exit_streak(
    sys_id: int,
    status: dict[str, object] | None,
    streaks: dict[int, int],
    logger: logging.Logger,
) -> None:
    tail = "\n".join((status or {}).get("log") or [])
    if NO_HEARTBEAT_EXIT_MARKER not in tail:
        streaks.pop(sys_id, None)
        return
    streak = streaks.get(sys_id, 0) + 1
    streaks[sys_id] = streak
    if streak == STALL_DIAGNOSIS_AFTER:
        logger.error(
            "NavPy companion for sys_id=%d has exited %d times in a row with "
            "zero heartbeats received — its UDP link from SITL serial0 is a "
            "black hole. Likely causes: (1) the WSL fork's run_swarm.sh "
            "predates COMPANION_UDP, so SITL still serves serial0 over TCP; "
            "(2) Windows firewall blocks WSL->Windows UDP on the companion "
            "port for this python.exe; (3) run_swarm.sh detected the wrong "
            "WIN_IP (mirrored-networking WSL — override with "
            "WIN_IP=<host-ip>).",
            sys_id,
            streak,
        )


__all__ = [
    "HB_EXIT_WINDOW_S",
    "STALL_DIAGNOSIS_AFTER",
    "track_heartbeat_exit_streak",
]
