"""MAVLink telemetry transport and position-stream sampling."""

from __future__ import annotations

import math
import sys
import time
from collections import deque
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from pymavlink import mavutil

from scripts import eval_certificate as cert

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Do it here rather than relying on another script
# having been imported first: without this the module (and its test) fails
# standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_models import PositionSample, PositionStreamAnchor  # noqa: E402
from eval_navigation_scoring import COORDINATE_SCORE_RATE_HZ, CoordinateScorer  # noqa: E402


SCORING_INTERVAL_SAMPLE_MAX_AGE_S = 0.25
# One blocking receive.  Every wait below is a loop of these, so a wait
# overruns its deadline by at most one slice, and a caller can hand down
# less than a slice to spend what is left of its own budget.
RECV_SLICE_S = 0.5
# Default wait for a SET_MESSAGE_INTERVAL acknowledgement.  Callers that
# run their own deadline clamp this to the time they have left.
MESSAGE_INTERVAL_ACK_TIMEOUT_S = 5.0


def message_from_target(message: Any, target_system: int) -> bool:
    """Return whether a decoded MAVLink message came from one target system."""
    try:
        return int(message.get_srcSystem()) == int(target_system)
    except (AttributeError, TypeError, ValueError):
        return False


def recv_target_message(
    master: Any,
    message_types: Any,
    timeout_s: float,
) -> Any | None:
    """Receive one requested message type from the selected vehicle only."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        message = master.recv_match(
            type=message_types,
            blocking=True,
            timeout=min(RECV_SLICE_S, remaining),
        )
        if message is not None and message_from_target(
            message, master.target_system
        ):
            return message
    return None


def command_long(master: Any, command: int, *params: float) -> None:
    """Send a MAVLink COMMAND_LONG with zero-padded parameters."""
    padded = list(params) + [0.0] * (7 - len(params))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *padded[:7],
    )


def position_sample_from_message(
    message: Any,
    received_wall_time_s: float,
) -> PositionSample:
    """Convert GLOBAL_POSITION_INT to the evaluator's source-time record."""
    raw_source_time_ms = getattr(message, "time_boot_ms", None)
    try:
        source_time_s = float(raw_source_time_ms) / 1000.0
    except (TypeError, ValueError):
        source_time_s = None
    if source_time_s is not None and not math.isfinite(source_time_s):
        source_time_s = None
    return PositionSample(
        lat_deg=float(message.lat) / 1e7,
        lon_deg=float(message.lon) / 1e7,
        abs_alt_m=float(message.alt) / 1000.0,
        rel_alt_m=float(message.relative_alt) / 1000.0,
        received_wall_time_s=received_wall_time_s,
        source_time_s=source_time_s,
    )


def position_stream_is_live(
    live_anchors: Sequence[PositionStreamAnchor],
    observed_wall_time_s: float,
    *,
    max_age_s: float = SCORING_INTERVAL_SAMPLE_MAX_AGE_S,
) -> bool:
    """Return whether the last fully drained source sample is recent."""
    if not live_anchors:
        return False
    age_s = observed_wall_time_s - live_anchors[-1].live_edge_wall_time_s
    return 0.0 <= age_s <= max_age_s


def wait_for_heartbeat(control_device: str, timeout_s: float) -> Any | None:
    """Open the evaluator GCS connection and select the heartbeat component."""
    master = mavutil.mavlink_connection(
        control_device,
        source_system=250,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    heartbeat = master.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        master.close()
        return None
    master.target_system = int(heartbeat.get_srcSystem())
    master.target_component = int(heartbeat.get_srcComponent())
    return master


def request_message_interval_stream(
    master: Any,
    message_id: int,
    rate_hz: float,
    *,
    timeout_s: float = MESSAGE_INTERVAL_ACK_TIMEOUT_S,
) -> bool:
    """Request one message stream on the evaluator link and await the ACK.

    ``SET_MESSAGE_INTERVAL`` is per-ArduPilot-channel, so requests here cannot
    change the companion's dedicated ``serial0`` streams -- only other
    watchers of this same monitor channel can (the Mission Planner incident).
    The ACK confirms the command was accepted, not that the stream is live at
    the requested rate; consumers must certify the observed stream.
    """
    if timeout_s <= 0.0:
        # A caller whose budget is already gone buys nothing here: the ACK
        # could not be waited for, and the send itself can block on a slow
        # link.  Refuse before spending anything.
        return False
    interval_us = int(round(1_000_000.0 / rate_hz))
    # Started before the send, which can block on a slow link: a caller that
    # hands down what is left of its own budget must not pay for it twice.
    deadline = time.monotonic() + timeout_s
    command_long(
        master,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        message_id,
        interval_us,
    )
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        message = master.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=min(RECV_SLICE_S, remaining),
        )
        if message is None or not message_from_target(
            message, master.target_system
        ):
            continue
        if int(getattr(message, "command", -1)) != int(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ):
            continue
        result = int(getattr(message, "result", -1))
        if result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            continue
        return result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    return False


def request_coordinate_score_stream(
    master: Any,
    *,
    timeout_s: float = 5.0,
) -> bool:
    """Request and confirm 30 Hz GLOBAL_POSITION_INT on the evaluator link."""
    return request_message_interval_stream(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        COORDINATE_SCORE_RATE_HZ,
        timeout_s=timeout_s,
    )


# MUST be a list: pymavlink's recv_match wraps any non-list/non-set filter --
# a tuple included -- as a single element (mavutil.py `type = [type]`), so a
# tuple filter matches nothing and the drain silently discards every message.
# Proven live 2026-09-03: the first truth-scoring flight recorded zero
# SIM_STATE rows through a healthy 40 Hz stream because of exactly this.
_DRAIN_MESSAGE_TYPES = ["GLOBAL_POSITION_INT", "SIM_STATE"]


def drain_position_messages(
    master: Any,
    *,
    sysid: int,
    live_anchors: deque[PositionStreamAnchor],
    scorer: CoordinateScorer | None,
    rate_tracker: cert.ClockRateTracker | None = None,
    track: Any | None = None,
    truth: Any | None = None,
    truth_scoring_active: bool = False,
) -> bool:
    """Drain position telemetry, feed scoring/rate evidence, and mark live edge.

    ``track`` receives the same GLOBAL_POSITION_INT messages, which carry the
    velocity fields the scorer's PositionSample drops.  Calling a run
    "tailwind" because SIM_WIND_DIR was 180 assumes the aircraft flew due
    north; the collinear mission makes that likely but never certain, so the
    relative wind has to be scored against the measured ground track.

    ``truth`` receives SIM_STATE messages from the same single typed drain:
    ``recv_match`` discards messages that do not match its filter, so a second
    type-filtered drain would silently starve one consumer of the other's
    messages.  SIM_STATE is drained whether or not the leg is scored --
    otherwise a pre-scoring interval backlog would flood the scored window at the
    marker -- and the recorder files each sample under the ``truth_scoring_active``
    flag it arrived with.
    """
    saw_sample = False
    newest_source_s: float | None = None
    for _ in range(500):
        message = master.recv_match(type=_DRAIN_MESSAGE_TYPES, blocking=False)
        if message is None:
            _mark_live_edge(live_anchors, saw_sample, newest_source_s)
            return True
        if not message_from_target(message, sysid):
            continue
        if message.get_type() == "SIM_STATE":
            if truth is not None:
                truth.add_message(message, time.time(), scoring_active=truth_scoring_active)
            continue
        sample = position_sample_from_message(message, time.time())
        saw_sample = True
        source = sample.source_time_s
        if source is not None and math.isfinite(source):
            newest_source_s = (
                source
                if newest_source_s is None
                else max(newest_source_s, source)
            )
        if scorer is not None:
            scorer.add(sample)
        if track is not None:
            track.add(message)
        if rate_tracker is not None:
            rate_tracker.add(sample.source_time_s, time.monotonic())
    live_anchors.clear()
    return False


def _mark_live_edge(
    live_anchors: deque[PositionStreamAnchor],
    saw_sample: bool,
    newest_source_s: float | None,
) -> None:
    if not saw_sample:
        return
    if newest_source_s is None:
        live_anchors.clear()
        return
    live_anchors.append(PositionStreamAnchor(newest_source_s, time.time()))
