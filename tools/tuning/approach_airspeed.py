"""The approach runner's side of eligible airspeed (hitl_guidance.md section 9, step 20).

Before arming (``approach._fly``, before ``takeoff``) ``pin_board`` refuses a board whose effective pitot parameters
are not the simulated profile navpy admits (``airspeed_profile.SIMULATED_PROFILES``), or whose firmware is not the
pinned mapping: the launched binary's sha256 must be a known build, and the board's AUTOPILOT_VERSION must report that
build's embedded commit. After the run ``outcome`` reads navpy's NAVPY_AIRSPEED_V1 records: a revoke, or a source
never admitted, fails the trial (the source is never readmitted, so one revoke ends the trial's law input).
"""
from __future__ import annotations

from pathlib import Path

from pymavlink import mavutil

from navpy.app.approach.law_airspeed import parse_airspeed_record
from navpy.plugins.vehicle.ardupilot.airspeed_profile import profile_for_mode, pin_matches
from tools.tuning.board import Board

# The launched binary -> the commit it embeds and reports (AUTOPILOT_VERSION.flight_custom_version, first 8 hex).
# My value: the pc-sitl runner's default feat/sim-hil build, provenance in step 19's Built record.
FIRMWARE_PINS = {"3c2facbbcacb6d69402446b98e3a392b3a86a22454fe60c6c8faf0eb515a6ccb": "28fbab23"}
VERSION_TIMEOUT_S = 5.0     # wait for the requested AUTOPILOT_VERSION
AUTOPILOT_VERSION_ID = 148


class AirspeedPinError(RuntimeError):
    """The board is not the pinned simulated pitot source: the run stops before arming."""


def reported_commit(version: object) -> str:
    """The 8-hex commit ArduPilot puts in flight_custom_version (GCS_Common.cpp send_autopilot_version)."""
    raw = getattr(version, "flight_custom_version", b"") or b""
    data = bytes(raw) if not isinstance(raw, str) else raw.encode("latin-1")
    return data.rstrip(b"\x00")[:8].decode("ascii", "replace")


def firmware_problems(binary: dict, version: object | None) -> list[str]:
    expected = FIRMWARE_PINS.get(binary.get("sha256") or "")
    if expected is None:
        return [f"firmware {binary.get('path')} (sha256 {binary.get('sha256')}) is not a pinned build"]
    if version is None:
        return ["the board sent no AUTOPILOT_VERSION"]
    got = reported_commit(version)
    return [] if got == expected else [f"the board reports commit {got!r}, the pinned binary embeds {expected!r}"]


def param_problems(mode: str, read: dict[str, float | None]) -> list[str]:
    expected = profile_for_mode(mode).pins()
    return [f"{name} is {read.get(name)!r}, the profile pins {value}" for name, value in expected.items()
            if not pin_matches(read.get(name), value)]


def pin_board(board: Board, mode: str, binary: dict) -> dict:
    """Read the pins and the firmware report from the board; raise AirspeedPinError on any mismatch."""
    read: dict[str, float | None] = {}
    for name in profile_for_mode(mode).pins():
        try:
            read[name] = board.get_param(name)
        except RuntimeError:
            read[name] = None
    board.mav.mav.command_long_send(board.mav.target_system, board.mav.target_component,
                                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, AUTOPILOT_VERSION_ID,
                                    0, 0, 0, 0, 0, 0)
    version = board.wait_message("AUTOPILOT_VERSION", VERSION_TIMEOUT_S)
    problems = param_problems(mode, read) + firmware_problems(binary, version)
    record = {"params": read, "commit": None if version is None else reported_commit(version),
              "binary_sha256": binary.get("sha256"), "problems": problems}
    if problems:
        raise AirspeedPinError("; ".join(problems))
    return record


def outcome(navpy_out: Path) -> dict:
    """navpy's source events, and the problems that fail the trial."""
    records = []
    if navpy_out.exists():
        lines = navpy_out.read_text(encoding="utf-8", errors="replace").splitlines()
        records = [r for r in map(parse_airspeed_record, lines) if r is not None]
    events = [r.get("event") for r in records]
    revokes = [r.get("reason") for r in records if r.get("event") == "revoked"]
    problems = [f"airspeed source revoked: {reason}" for reason in revokes]
    if "admitted" not in events:
        problems.append("airspeed source never admitted: no law input was eligible")
    return {"events": events, "revoked": revokes, "problems": problems}


__all__ = ["AirspeedPinError", "FIRMWARE_PINS", "firmware_problems", "outcome", "param_problems", "pin_board",
           "reported_commit"]
