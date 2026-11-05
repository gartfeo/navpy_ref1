"""Requests from the app that a lockstep run refuses (``deterministic_dev_sim.md`` section 5).

The coordinator's admission asks :func:`refusal` for each relay request; a
refused one never enters the run and is answered ``501``, journaled like
any answer. The table is the step-10 inventory's rows that simulation does
not support (``gcs_request_inventory.md``, "Sim today"): the two zip
downloads, stamped with wall time (todo 241), and the vision-profile edits,
which write ``vision_profiles.json`` in the source tree, outside the run
(todo 283). Since step 13 the full-parameter read (MAVFTP) and the app's
NavPy-sim start and stop run in the sim. Two kinds of vehicle command are
refused by name (``REFUSED_COMMANDS``): every ``MAV_CMD_PREFLIGHT_CALIBRATION``
blocks the firmware's main loop for seconds, a clock step the fleet grid
refuses (todo 285), and a reboot restarts the firmware, which closes its relay.
A settings edit that turns on the launch's own calibration (``launch.auto_preflight_cal``)
is refused for the same reason.

Anything that is not an HTTP request with a method and a target is refused
too: the relay sends nothing else.
"""

from __future__ import annotations

import json
import re
from typing import Any
from urllib.parse import unquote, urlsplit

#: ``(method, path pattern, reason)``; the pattern matches the decoded path without its query.
REFUSED: tuple[tuple[str, re.Pattern[str], str], ...] = tuple(
    (method, re.compile(pattern), reason) for method, pattern, reason in (
        ("GET", r"/api/diagnostics/export", "a zip download (todo 241)"),
        ("GET", r"/api/logs/download/[^/]+", "a zip download (todo 241)"),
        ("PUT", r"/api/vision-profiles/.+", "writes vision_profiles.json in the source tree (todo 283)"),
    ))
#: ``POST /api/control/command`` commands refused by their ``command`` field.
_CALIBRATION = "a preflight calibration blocks the firmware's main loop (todo 285)"
_REBOOT = "a reboot restarts the firmware, which closes its relay (todo 285)"
REFUSED_COMMANDS: dict[str, str] = {
    "preflight_cal": _CALIBRATION,
    "accel_level": _CALIBRATION,
    "accel_cal_start": _CALIBRATION,
    "accel_cal_pos": _CALIBRATION,
    "reboot": _REBOOT,
    "compass_cal_reboot": _REBOOT,
}
COMMAND_PATH = "/api/control/command"
#: ``PUT /api/settings`` may not turn on the launch's own preflight calibration
#: (``control_preflight._maybe_auto_preflight_cal``, sent at launch and trigger).
SETTINGS_PATH = "/api/settings"
NOT_HTTP = "only HTTP requests enter the run"
NOT_ASCII = "a request target is ASCII (percent-encoded)"
STATUS = 501


def answer(reason: str) -> dict[str, Any]:
    return {"status": STATUS, "body": json.dumps(
        {"detail": f"not supported in simulation: {reason}"})}


def refusal(payload: dict[str, Any]) -> dict[str, Any] | None:
    """The answer that refuses ``payload``, or None to admit it."""
    method, target = payload.get("method"), payload.get("target")
    if payload.get("kind") != "http" or not isinstance(method, str) or not isinstance(target, str):
        return answer(NOT_HTTP)
    if not target.isascii():
        return answer(NOT_ASCII)
    # Decoded once, as the backend's scope decodes it (``lockstep_asgi._scope``).
    path = unquote(urlsplit(target).path)
    # Starlette answers HEAD with the GET route's handler.
    method = "GET" if method.upper() == "HEAD" else method.upper()
    for refused_method, pattern, reason in REFUSED:
        if method == refused_method and pattern.fullmatch(path):
            return answer(reason)
    if method == "POST" and path == COMMAND_PATH:
        body = payload.get("json")
        command = body.get("command") if isinstance(body, dict) else None
        if isinstance(command, str) and command in REFUSED_COMMANDS:
            return answer(REFUSED_COMMANDS[command])
    if method == "PUT" and path == SETTINGS_PATH:
        body = payload.get("json")
        launch = body.get("launch") if isinstance(body, dict) else None
        # Anything but an explicit false could validate as on (pydantic coerces 1, "true").
        if isinstance(launch, dict) and launch.get("auto_preflight_cal", False) is not False:
            return answer(_CALIBRATION)
    return None


__all__ = ["COMMAND_PATH", "NOT_ASCII", "NOT_HTTP", "REFUSED", "REFUSED_COMMANDS", "SETTINGS_PATH", "STATUS",
           "answer", "refusal"]
