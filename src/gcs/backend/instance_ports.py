"""Per-chat (per-instance) port and sys_id arithmetic — single source of truth.

Multiple GCS stacks (one per Claude/dev "chat") can run on one machine without
colliding. Everything derives from one integer ``N`` (the chat index), allocated
by ``scripts/gcs_launch.py`` and handed to the backend via the ``GCS_CHAT_INDEX``
environment variable.

Port layout (vehicle ``i = 1..VEHICLES_PER_CHAT``, global vehicle index
``g = VEHICLES_PER_CHAT*N + i`` which is also the MAVLink sys_id)::

    Frontend (Vite)            3000 + N
    Backend  (uvicorn)         8000 + N
    Mission Planner (2nd GCS)  14550 + N       (chat 0 = 14550 -> MP auto-connects)
    Our GCS monitor            15550 + N       (all UAVs, multiplexed via router)
    NavPy companion (sys_id g) udp 5760 + 10*(g-1)  (companion binds; SITL streams in)
    Eval telemetry (sys_id g)  udp 24000 + (g-1)   (scorer binds; SITL serial1 in)
    SITL -I / dir / sys_id     (g-1) / g / g

The two GCS-facing ports are exported to the Windows host by the SITL
mavlink-router (``ROUTER_WIN_PORTS``). Mission Planner sits on the
MAVLink-standard 14550 for chat 0 so it attaches with no manual config; our GCS
monitor is the per-run "default connection" (injected into the settings API as
``default_device``). Each NavPy companion binds a local UDP server on its
per-vehicle port; the SITL instance's serial0 is launched as
``udpclient:<WIN_IP>:<port>`` (see run_swarm.sh ``COMPANION_UDP``) and streams
into it -- a dedicated per-vehicle link, not routed with the GCS traffic. The
bands (3000 / 8000 / 14550 / 15550) are disjoint across the whole supported chat
range. The binding cap is MAVLink's 8-bit sys_id (1..255): with 3 vehicles per
chat that is ~84 chats.
"""
from __future__ import annotations

import os

# GCS-facing UDP bands — kept disjoint so no two families ever meet.
FRONTEND_BASE = 3000
BACKEND_BASE = 8000
# Mission Planner on the MAVLink-standard 14550 for chat 0 -> auto-connects.
MISSION_PLANNER_BASE = 14550
# Our GCS monitor (also the per-run default connection injected into settings).
GCS_MONITOR_BASE = 15550
# A dedicated UDP port the SITL router also mirrors telemetry to, used ONLY by
# swarm_run.py at launch time to verify every instance is actually streaming
# heartbeats (not just that a serial0 socket opened). Its own band so it never
# contends with the backend's monitor port or Mission Planner.
GCS_VERIFY_BASE = 16550
# Per-vehicle companion UDP ports. The numbers keep ArduPilot's historical
# serial0 arithmetic (5760 + 10*instance_idx) but are now Windows-side bind
# ports: the companion listens here and SITL serial0 (udpclient) streams in.
SITL_SERIAL0_BASE = 5760
SITL_SERIAL0_STEP = 10

VEHICLES_PER_CHAT = 3
MAX_SYSID = 255  # MAVLink system id is a uint8 (1..255)

#: Highest chat index whose top sys_id still fits in a uint8.
MAX_CHAT_INDEX = (MAX_SYSID - VEHICLES_PER_CHAT) // VEHICLES_PER_CHAT  # == 84

#: Split between the two reserved chat bands:
#: interactive GCS launches (with a frontend) auto-allocate in ``[0, EVAL_CHAT_MIN)``;
#: eval / SITL-only launches (no frontend) auto-allocate in ``[EVAL_CHAT_MIN, MAX]``.
#: Keeping them disjoint stops an interactive session and a running eval swarm from
#: ever grabbing the same slot. ``--chat N`` is an explicit override of either band.
EVAL_CHAT_MIN = 40

_ENV_CHAT_INDEX = "GCS_CHAT_INDEX"


def interactive_chat_hi() -> int:
    """Highest chat index the interactive band may auto-allocate (``EVAL_CHAT_MIN-1``)."""
    return EVAL_CHAT_MIN - 1


#: Labels whose slots live in the eval band (SITL-only eval runs, no frontend).
#: Everything else (``"gcs"``, ``"sitl"``, ...) belongs to the interactive band.
#: Lets registry lookups scope to a band so an interactive stack and an eval
#: stack launched from the SAME directory are treated as independent slots
#: (see ``instance_registry.find_for_owner``).
EVAL_LABELS = frozenset({"sitl-eval"})


def interactive_band() -> tuple[int, int]:
    """Inclusive ``(lo, hi)`` chat-index band for interactive GCS / paired-SITL launches."""
    return (0, interactive_chat_hi())


def eval_band() -> tuple[int, int]:
    """Inclusive ``(lo, hi)`` chat-index band for eval / SITL-only launches."""
    return (EVAL_CHAT_MIN, MAX_CHAT_INDEX)


def band_for_label(label: str | None) -> tuple[int, int]:
    """The chat-index band a launch with *label* auto-allocates in.

    Eval labels (:data:`EVAL_LABELS`) map to the eval band, everything else to
    the interactive band. Registry lookups pass the label they mean so they only
    ever resolve a slot in that band — an eval cleanup never touches this
    directory's interactive slot, and an interactive one never touches its eval
    slot.
    """
    return eval_band() if label in EVAL_LABELS else interactive_band()


def in_band(n: int, band: tuple[int, int]) -> bool:
    """True if chat index *n* falls within the inclusive ``(lo, hi)`` *band*."""
    lo, hi = band
    return lo <= n <= hi


def chat_index() -> int | None:
    """Return the chat index ``N`` from the environment, or ``None`` if unset/invalid."""
    raw = os.environ.get(_ENV_CHAT_INDEX)
    if raw is None or raw == "":
        return None
    try:
        n = int(raw)
    except (TypeError, ValueError):
        return None
    return n if n >= 0 else None


def frontend_port(n: int) -> int:
    return FRONTEND_BASE + n


def backend_port(n: int) -> int:
    return BACKEND_BASE + n


def monitor_port(n: int) -> int:
    return GCS_MONITOR_BASE + n


def mission_planner_port(n: int) -> int:
    return MISSION_PLANNER_BASE + n


def verify_port(n: int) -> int:
    """Dedicated UDP port the launcher listens on to confirm every instance in
    chat *n* is actually streaming telemetry (see swarm_run.py)."""
    return GCS_VERIFY_BASE + n


def sysids_for_chat(n: int) -> list[int]:
    """The global sys_ids a chat's swarm uses: ``3N+1, 3N+2, 3N+3``."""
    base = VEHICLES_PER_CHAT * n
    return [base + i for i in range(1, VEHICLES_PER_CHAT + 1)]


def companion_port(sys_id: int) -> int:
    """UDP port the NavPy companion for *sys_id* binds (SITL serial0 streams in)."""
    return SITL_SERIAL0_BASE + SITL_SERIAL0_STEP * (sys_id - 1)


def companion_ports_for_chat(n: int) -> list[int]:
    """The per-vehicle companion UDP ports for chat *n*'s sys_ids."""
    return [companion_port(s) for s in sysids_for_chat(n)]


def is_companion_port(port: int) -> bool:
    """Whether *port* belongs to the companion band (5760 + 10*k, k=0..254).

    The inverse of :func:`companion_port`, kept beside it so a change to the
    band's base/stride can't drift apart from its membership predicate.
    """
    top = companion_port(MAX_SYSID)
    return (SITL_SERIAL0_BASE <= port <= top
            and (port - SITL_SERIAL0_BASE) % SITL_SERIAL0_STEP == 0)


def monitor_device(n: int) -> str:
    """Connection string the GCS uses to discover/monitor all UAVs in chat *n*."""
    return f"udp:0.0.0.0:{monitor_port(n)}"


def companion_device(sys_id: int) -> str:
    """Connection string a NavPy companion subprocess uses for *sys_id*.

    A local UDP server bind: the SITL instance's serial0 is launched as
    ``udpclient:<WIN_IP>:<port>`` and streams into it. One dedicated link per
    vehicle, not multiplexed with GCS traffic, and nothing to dial — the
    companion needs no WSL host detection.
    """
    return f"udp:0.0.0.0:{companion_port(sys_id)}"


def router_win_ports(n: int) -> list[int]:
    """Windows-side UDP ports the SITL router must publish for chat *n*.

    Only the two GCS-facing ports (Mission Planner + our monitor). Companions
    use dedicated per-vehicle UDP links (SITL serial0 -> companion bind) and
    are *not* router-exported. Passed to ``run_swarm.sh`` as ``ROUTER_WIN_PORTS``.
    """
    return [mission_planner_port(n), monitor_port(n)]


def ports_for_chat(n: int) -> dict:
    """All ports for chat *n* — used by the launcher to check availability/print."""
    return {
        "chat_index": n,
        "frontend": frontend_port(n),
        "backend": backend_port(n),
        "gcs_monitor": monitor_port(n),
        "mission_planner": mission_planner_port(n),
        "sysids": sysids_for_chat(n),
        "companions": companion_ports_for_chat(n),
        "swarm_offset": VEHICLES_PER_CHAT * n,  # -I offset for run_swarm.sh
    }
