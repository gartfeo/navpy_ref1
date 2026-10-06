"""Vehicle status text, calibration events, and freshness constants."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Optional
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

if TYPE_CHECKING:
    from gcs.backend.vehicle_entry import VehicleEntry

log = logging.getLogger("gcs.backend.vehicle_manager")

import os

GIMBAL_TELEMETRY_STALE_S = 2.0

# Opt-in diagnostic: when GCS_LOG_GIMBAL_TELEM is truthy, log every gimbal
# attitude telemetry sample — the 5 Hz stream that drives the live map camera
# footprint — at INFO, so footprint smoothness can be measured OFFLINE from the
# backend log (no live map/browser needed). OFF by default: at 5 Hz per gimbal
# this would otherwise spam the production backend log.
_LOG_GIMBAL_TELEM = os.environ.get("GCS_LOG_GIMBAL_TELEM", "").strip().lower() in (
    "1", "true", "yes", "on",
)
CAMERA_OPTICS_STALE_S = 2.0

# Companion (NavPy) liveness windows. The companion emits a 1 Hz heartbeat that
# rides the *shared*, lossy telemetry link (multiplexed with every vehicle's
# full-rate flight data through a router). A short run of missed heartbeats is
# ordinary packet loss, not a dead companion, so liveness is a tri-state:
#   age <= COMPANION_OK_WINDOW_S      -> "ok"       (fresh)
#   age <= COMPANION_DOWN_WINDOW_S    -> "checking" (transient loss; not faulty)
#   otherwise / never seen            -> "down"     (sustained silence)
# This stops the indicator (and launch-readiness) from flapping to "faulty" on a
# few dropped 1 Hz packets, while still surfacing a genuinely dead companion
# within COMPANION_DOWN_WINDOW_S. Widening only the *down* transition is safe:
# the state recovers to "ok" on the very next heartbeat.
COMPANION_OK_WINDOW_S = 5.0
COMPANION_DOWN_WINDOW_S = 15.0

# ArduPilot exposes RC1..RC16 calibration parameters; the RC_CHANNELS message
# can carry up to 18, but only the first 16 map to writable RCn_* params, so
# the radio-calibration snapshot is capped here.
RC_CHANNELS_MAX = 16

# Accelerometer calibration --------------------------------------------------
# Position codes match ArduPilot's AccelCalibrator vehicle-position enum and
# the MAV_CMD_ACCELCAL_VEHICLE_POS param1 values the GCS sends back to advance
# each step.
ACCEL_CAL_POS_LEVEL = 1
ACCEL_CAL_POS_LEFT = 2
ACCEL_CAL_POS_RIGHT = 3
ACCEL_CAL_POS_NOSEDOWN = 4
ACCEL_CAL_POS_NOSEUP = 5
ACCEL_CAL_POS_BACK = 6

# Ordered (substring -> position code) table for the autopilot's
# "Place vehicle <orientation> and press any key." STATUSTEXT prompts. Checked
# in order so the more specific "nose down/up" entries win before "back"/"level"
# and each prompt maps to exactly one position.
_ACCEL_CAL_PROMPT_KEYWORDS = (
    ("left", ACCEL_CAL_POS_LEFT),
    ("right", ACCEL_CAL_POS_RIGHT),
    ("nose down", ACCEL_CAL_POS_NOSEDOWN),
    ("nose up", ACCEL_CAL_POS_NOSEUP),
    ("back", ACCEL_CAL_POS_BACK),
    ("level", ACCEL_CAL_POS_LEVEL),
)

# A "result" STATUSTEXT (calibration successful/failed) is surfaced as a cal
# event only while a calibration is considered active — set when a cal command
# is sent or a position prompt arrives, and held this long so an unrelated
# boot-time "Calibration successful" can't be misattributed to the GCS wizard.
ACCEL_CAL_ACTIVE_WINDOW_S = 180.0

# CONF-03 recognition-gate blocked state (D-15/D-16/D-17) -------------------
# NavController emits ``CONFIRM_BLOCKED:<reason>|<detail>|<task_id>`` (DRONE
# dest, rate-limited to ~1 Hz while blocking) and ``CONFIRM_BLOCKED:clear`` the
# instant the gate passes/POI is lost/POI changes. CONFIRM_BLOCKED_STALE_S
# is a safety net alongside that explicit clear signal (several missed
# rate-limit cycles' worth of margin) in case the clear STATUSTEXT itself is
# lost on the shared lossy link.
CONFIRM_BLOCKED_PREFIX = "CONFIRM_BLOCKED:"
CONFIRM_BLOCKED_STALE_S = 5.0


def _classify_accel_cal_statustext(text: str) -> Optional[dict]:
    """Classify an autopilot STATUSTEXT as an accel-cal event, or return None.

    Returns ``{"status": "prompt"|"success"|"failed", "step": code|None,
    "prompt_text": text}`` for the 6-position prompts and the final
    success/failure lines, and ``None`` for everything else.
    """
    low = text.strip().lower()
    if not low:
        return None
    if low.startswith("place vehicle"):
        step = None
        for keyword, code in _ACCEL_CAL_PROMPT_KEYWORDS:
            if keyword in low:
                step = code
                break
        return {"status": "prompt", "step": step, "prompt_text": text.strip()}
    # Final result lines. Compass calibration uses its own "Compass ..." wording,
    # so exclude it to keep this scoped to the accelerometer.
    if "compass" not in low:
        if "calibration successful" in low:
            return {"status": "success", "step": None, "prompt_text": text.strip()}
        if "calibration failed" in low or "calibration unsuccessful" in low:
            return {"status": "failed", "step": None, "prompt_text": text.strip()}
    return None


def _on_statustext(entry: VehicleEntry, msg: MAVLink_message) -> None:
    sev = getattr(msg, "severity", 6)
    text = getattr(msg, "text", "").rstrip("\x00")
    label = entry._SEVERITY_LABELS.get(sev, "INFO")
    entry._status_texts.append({"severity": sev, "label": label, "text": text, "ts": time.time()})
    # Track PreArm message categories
    if text.startswith("PreArm:"):
        reason = text[7:].strip().lower()
        if "mode not armable" in reason:
            entry._prearm_mode_only = True
        else:
            entry._prearm_other = True
    # CONF-03 recognition-gate blocked state (D-15/D-16/D-17). CRITICAL:
    # gated behind _from_own_companion — every companion's STATUSTEXT
    # cross-delivers onto every vehicle's connection over the shared
    # link (Pitfall 2); without this filter one UAV's blocked reason
    # would flicker onto another UAV's card.
    if text.startswith(CONFIRM_BLOCKED_PREFIX) and entry._from_own_companion(msg):
        entry._handle_confirm_blocked_statustext(text[len(CONFIRM_BLOCKED_PREFIX):])
    # Surface accelerometer-calibration prompts/results to the GCS wizard.
    event = _classify_accel_cal_statustext(text)
    if event is not None:
        now_monotonic = time.monotonic()
        if event["status"] == "prompt":
            # A prompt both carries a step and (re)confirms the cal is live.
            entry._accel_cal_active_until = now_monotonic + ACCEL_CAL_ACTIVE_WINDOW_S
            entry._accel_cal_events.append(event)
        elif now_monotonic <= entry._accel_cal_active_until:
            # Result line: surface only while a cal is active, then close it.
            entry._accel_cal_active_until = 0.0
            entry._accel_cal_events.append(event)

def _handle_confirm_blocked_statustext(entry: VehicleEntry, body: str) -> None:
    """Parse a ``CONFIRM_BLOCKED:`` STATUSTEXT body (CONF-03, D-15/D-16/D-17).

    ``body`` is everything after the ``CONFIRM_BLOCKED:`` prefix:
    ``clear`` resets the state, otherwise ``<reason>|<detail>|<task_id>``
    (``reason`` is ``pixels`` or ``zoom``; ``detail`` carries the pixel
    counts for ``pixels`` and is empty for ``zoom``; ``task_id`` is the
    blocked POI, used to address the "Ask me anyway" override so it
    forces the right POI without guessing).
    """
    body = body.strip()
    if body == "clear":
        entry._confirm_blocked = None
        entry._confirm_blocked_updated_at = time.monotonic()
        return
    parts = body.split("|")
    reason = parts[0].strip() if parts else ""
    if not reason:
        return
    detail = parts[1].strip() if len(parts) > 1 else ""
    task_id: Optional[int] = None
    if len(parts) > 2 and parts[2].strip():
        try:
            task_id = int(parts[2].strip())
        except ValueError:
            task_id = None
    entry._confirm_blocked = {"reason": reason, "detail": detail, "task_id": task_id}
    entry._confirm_blocked_updated_at = time.monotonic()
