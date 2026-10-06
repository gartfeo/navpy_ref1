"""Rendering for ``gcs_launch.py --list``: running slots and SITL verdicts.

Split out of ``gcs_launch.py`` so the launcher module stays within the SOLID
size limits; ``gcs_launch`` re-exports ``_sitl_status`` for its tests.
"""
from __future__ import annotations

from gcs.backend import instance_registry as reg


def _print_list() -> None:
    instances = reg.live()
    if not instances:
        print(f"No running GCS instances (registry: {reg.registry_path()}).")
        return
    print(f"Running GCS instances (registry: {reg.registry_path()}):")
    for e in instances:
        print(f"  chat {e['chat_index']}: frontend :{e['frontend']} backend :{e['backend']} "
              f"monitor :{e['monitor']} sysids {e['sysids']} "
              f"sitl={_sitl_status(e)} "
              f"[{e.get('branch') or '?'} | {e.get('clone') or '?'}]")


def _sitl_status(entry: dict) -> str:
    """Render a slot's SITL launch verdict for --list.

    This is where a failed swarm launch stays visible: swarm_run gets its own
    console (CREATE_NEW_CONSOLE) that closes the instant it exits, so its abort
    message flashes and is gone before anyone reads it.

    An ABSENT verdict key means the entry predates this evidence, which is not
    the same as a verification still in flight — the whole point is durable
    evidence, so "we never looked" must not read as "we are looking".

    A cleared ``sitl`` flag likewise is not automatically "nothing happened":
    a terminally failed launch retracts the flag but keeps its verdict
    (``instance_registry.retract_sitl``), and that verdict is then the last
    thing anyone will ever see about it.
    """
    speed = entry.get("sitl_speedup")
    speed_note = f" speedup={speed:g}" if isinstance(speed, (int, float)) else ""
    if not entry.get("sitl"):
        # The slot itself may well be up — only its swarm never came.
        if entry.get("sitl_verified") is False:
            return (f"no{speed_note} - LAST SITL LAUNCH FAILED VERIFICATION: "
                    f"{entry.get('sitl_error') or 'unknown'}")
        return "no"
    if "sitl_verified" not in entry:
        return "yes (verification unknown - legacy entry)"
    verified = entry["sitl_verified"]
    if verified is None:
        return "yes (verifying)"
    if verified:
        return f"yes{speed_note} verified"
    return (f"yes{speed_note} FAILED VERIFICATION: "
            f"{entry.get('sitl_error') or 'unknown'}")
