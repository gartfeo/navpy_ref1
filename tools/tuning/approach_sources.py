"""One owner of each approach role at the data level, after the run (hitl_guidance.md section 9, step 17).

A process that came and went between the owner censuses (approach_owners.py) still shows here: Unreal took its pose
from one stream on the run's port only and its gimbal from the emulator's one session, and the emulator took its
attitude from one bridge session of one system.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

from tools.tuning import approach_chain as chain

POSE_GENERATION = re.compile(r"NavPyRenderClock port=(\d+) event=generation")
POSE_ADMIT = re.compile(r"NavPyPoseAdmit port=(\d+)")
GIMBAL_SESSION = re.compile(r"NavPyGimbalHistory sys_id=\d+ event=session session=(\S+)")
SLOT_SESSION = re.compile(r"NavPyCaptureSlot outcome=sent .*?\bgimbal_session=(\S+)")


def sources(unreal_log: Path, emulator_log: Path) -> dict:
    """One pose stream on the run's port, one gimbal session (the emulator's) behind every rendered frame, and one
    bridge session of one system behind the emulator. A missing or unreadable log fails."""
    problems, session, summary = [], None, None
    try:
        for line in emulator_log.read_text(encoding="utf-8").splitlines():
            if '"event":"start"' not in line and '"event":"summary"' not in line:
                continue
            record = json.loads(line)
            if record.get("event") == "start":
                session = session or record.get("session_id")
            elif record.get("event") == "summary":
                summary = record
        text = unreal_log.read_text(encoding="utf-8", errors="replace")
    except (OSError, ValueError) as exc:
        return {"ok": False, "problems": [f"source logs unreadable: {type(exc).__name__}: {exc}"]}
    generations = [int(port) for port in POSE_GENERATION.findall(text)]
    pose_ports = sorted({int(port) for port in POSE_ADMIT.findall(text)})
    gimbal = GIMBAL_SESSION.findall(text)
    slots = sorted(set(SLOT_SESSION.findall(text)))
    if generations != [chain.POSE_PORT]:
        problems.append(f"Unreal pose streams {generations}: exactly 1 on port {chain.POSE_PORT}")
    if pose_ports not in ([], [chain.POSE_PORT]):
        problems.append(f"Unreal took poses on ports {pose_ports}")
    if not session or gimbal != [session]:
        problems.append(f"Unreal gimbal sessions {gimbal}: exactly the emulator's ({session})")
    if any(slot != session for slot in slots):
        problems.append(f"frames rendered from gimbal sessions {slots}")
    vehicle = (summary or {}).get("vehicle_state")
    if not vehicle:
        problems.append("no emulator summary")
    elif (vehicle.get("sessions"), vehicle.get("other_system"), vehicle.get("board_time_regressions")) != (1, 0, 0):
        problems.append(f"the emulator's attitude came from {vehicle.get('sessions')} bridge session(s), "
                        f"{vehicle.get('other_system')} from another system, {vehicle.get('board_time_regressions')} "
                        "board time regression(s)")
    return {"ok": not problems, "problems": problems, "emulator_session": session, "pose_generations": generations,
            "pose_ports": pose_ports, "gimbal_sessions": gimbal, "slot_sessions": slots,
            "bridge_sessions": (vehicle or {}).get("sessions")}


__all__ = ["sources"]
