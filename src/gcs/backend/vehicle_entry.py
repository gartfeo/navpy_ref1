"""One connected vehicle: metadata, callback registration and state ownership."""
from __future__ import annotations

import collections
import threading
import time
from typing import Optional
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message
from navpy.modules.vehicle.vehicle_mav import (
    VehicleMav, PREARM_STATE_OK, PREARM_STATE_CHECKS_DISABLED, PREARM_STATE_FAILED,
)
from gcs.backend.companion_identity import COMPANION_TELEMETRY_COMPONENT_IDS, is_from_companion
from gcs.backend.companion_swarm_state import CompanionSwarmState
from gcs.backend.vehicle_status import ACCEL_CAL_ACTIVE_WINDOW_S, RC_CHANNELS_MAX, CONFIRM_BLOCKED_STALE_S
from gcs.backend import vehicle_status, vehicle_camera, vehicle_snapshot


class VehicleEntry:
    """Own one VehicleMav's metadata and received telemetry state."""
    _SEVERITY_LABELS = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR",
                        4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}

    def __init__(self, sys_id: int, vehicle: VehicleMav, name: str, device: str = "") -> None:
        self.sys_id = sys_id
        self.vehicle = vehicle
        self.name = name
        self.device = device
        self.mission_uploaded: bool = False
        self.probed_search_pattern: Optional[str] = None
        self.cached_mission: Optional[dict] = None
        # True while the async connect-time mission probe is in flight. Drives the
        # UI's "Downloading…" load stage in auto-connect mode, where the frontend
        # never runs the /mission download and so has no local download state to
        # key off. Set when the probe is kicked (see _probe_mission_async),
        # cleared in that thread's finally.
        self.is_probing: bool = False
        # [current, total] waypoint counts while a mission download is actually in
        # flight (either path: the connect-time probe, or the frontend's /mission
        # route) — None the rest of the time. Lets the UI show real progress
        # ("12/45") instead of a static "Downloading…" with no numbers.
        self.mission_download_progress: Optional[list[int]] = None
        self.connected_at = time.time()
        self._status_texts: collections.deque = collections.deque(maxlen=20)
        self._accel_cal_events: collections.deque = collections.deque(maxlen=20)
        self._accel_cal_active_until: float = 0.0
        self._prearm_mode_only: bool = False
        self._prearm_other: bool = False
        # CONF-03 blocked-state (D-15/D-16/D-17): {"reason", "detail",
        # "task_id"} while the drone's pixel/zoom recognition gate is
        # actually blocking; None otherwise. Source-filtered by
        # _from_own_companion (Pitfall 2) so one UAV's blocked reason never
        # flickers onto another's card.
        self._confirm_blocked: Optional[dict] = None
        self._confirm_blocked_updated_at: float = 0.0
        self._gimbals: dict[int, dict] = {}
        self._gimbal_lock = threading.Lock()
        self._rc_channels: list[Optional[int]] = []
        self._rc_lock = threading.Lock()
        # FREE/BUSY from the own companion's SWARM_HEARTBEAT.
        self._swarm_state = CompanionSwarmState(sys_id)
        vehicle.on_message("STATUSTEXT", self._on_statustext)
        vehicle.on_message(
            "GIMBAL_DEVICE_ATTITUDE_STATUS",
            self._on_gimbal_device_attitude_status,
        )
        vehicle.on_message("CAMERA_FOV_STATUS", self._on_camera_fov_status)
        vehicle.on_message("CAMERA_SETTINGS", self._on_camera_settings)
        vehicle.on_message("RC_CHANNELS", self._on_rc_channels)
        vehicle.on_message("SWARM_HEARTBEAT", self._swarm_state.on_heartbeat)

    def mark_accel_cal_active(self) -> None:
        """Mark that a GCS-initiated accel calibration is in progress.

        Lets result STATUSTEXTs (calibration successful/failed) be surfaced as
        cal events for a bounded window even when no position prompt preceded
        them (e.g. the quick level cal). Called from the control route when an
        accel-cal command is sent.
        """
        self._accel_cal_active_until = time.monotonic() + ACCEL_CAL_ACTIVE_WINDOW_S

    def _on_statustext(self, msg: MAVLink_message) -> None:
        return vehicle_status._on_statustext(self, msg)

    def _handle_confirm_blocked_statustext(self, body: str) -> None:
        return vehicle_status._handle_confirm_blocked_statustext(self, body)

    def _from_own_companion(self, msg: MAVLink_message) -> bool:
        """True if *msg*'s MAVLink source is THIS vehicle's own companion.

        Every companion's gimbal/camera telemetry cross-delivers onto every
        vehicle's connection over the shared MAVLink link (see the
        "shared lossy link" note in ``companion_status``). Attribute gimbal
        state by MAVLink SOURCE, not by the receiving connection — otherwise a
        vehicle's map footprint is fed all companions' gimbal poses/optics
        (last-writer-wins), flickering between vehicles and showing near-
        identical poses across UAVs.

        The companion shares its aircraft's system id, so the COMPONENT id is
        what separates it from the autopilot: matching on system id alone
        would let the aircraft's own traffic alias as companion gimbal
        telemetry. Messages without a MAVLink source (test / non-mav paths)
        are accepted unchanged.
        """
        if not hasattr(msg, "get_srcSystem"):
            return True
        return is_from_companion(
            msg, self.sys_id, COMPANION_TELEMETRY_COMPONENT_IDS,
        )

    def _on_gimbal_device_attitude_status(self, msg: MAVLink_message) -> None:
        return vehicle_camera._on_gimbal_device_attitude_status(self, msg)

    def _on_camera_fov_status(self, msg: MAVLink_message) -> None:
        return vehicle_camera._on_camera_fov_status(self, msg)

    def _on_camera_settings(self, msg: MAVLink_message) -> None:
        return vehicle_camera._on_camera_settings(self, msg)

    def _on_rc_channels(self, msg: MAVLink_message) -> None:
        """Capture the latest raw RC input (PWM µs) for radio calibration.

        Runs on the MAVLink reader thread; :meth:`snapshot` reads the result on
        the asyncio loop thread, so the channel list is swapped under a lock.
        Channels reported as UINT16_MAX (65535 = "not available") are stored as
        ``None`` so the UI can distinguish an unwired channel from a real 0.
        """
        raw_count = getattr(msg, "chancount", 0)
        try:
            count = int(raw_count)
        except (TypeError, ValueError):
            count = 0
        if count < 1 or count > RC_CHANNELS_MAX:
            count = RC_CHANNELS_MAX

        channels: list[Optional[int]] = []
        for i in range(1, count + 1):
            raw = getattr(msg, f"chan{i}_raw", None)
            try:
                value = int(raw)
            except (TypeError, ValueError):
                channels.append(None)
                continue
            channels.append(None if value == 65535 else value)

        with self._rc_lock:
            self._rc_channels = channels

    def _rc_channels_snapshot(self) -> Optional[dict]:
        """Return the latest RC input as ``{"channels": [...], "count": N}``.

        ``None`` until the first RC_CHANNELS message arrives, so the snapshot
        omits the key entirely on vehicles with no RC link. Uses defensive
        ``getattr`` (mirroring :meth:`_gimbal_snapshot`) so it tolerates entries
        built via ``__new__`` in tests that bypass ``__init__``.
        """
        rc_channels = getattr(self, "_rc_channels", None)
        if not rc_channels:
            return None
        lock = getattr(self, "_rc_lock", None)
        if lock is None:
            channels = list(rc_channels)
        else:
            with lock:
                channels = list(self._rc_channels)
        return {"channels": channels, "count": len(channels)}

    def _refresh_prearm_flags(self) -> None:
        """Clear stale PreArm flags whenever the vehicle is armable.

        "Armable" is *any* armable state — both ``ok`` and ``checks_disabled``
        (keying off armability rather than ``prearm_ok is True`` matters because
        ``checks_disabled`` is armable yet leaves ``prearm_ok`` as ``None``).
        When armable there is no active pre-arm failure, so any
        ``_prearm_mode_only`` / ``_prearm_other`` flags are stale and are cleared
        — on *every* armable cycle, not just the transition, so a late or stale
        "mode not armable" STATUSTEXT that arrives while already armable can't
        survive and later exempt a real ``failed`` state. Called every telemetry
        cycle from :meth:`snapshot`, so the reset never depends on the launch
        gate being polled. Flags set by STATUSTEXT during an actual ``failed``
        state are left intact (that state is not armable).
        """
        if self.vehicle.prearm_check_state in (
            PREARM_STATE_OK, PREARM_STATE_CHECKS_DISABLED,
        ):
            self._prearm_mode_only = False
            self._prearm_other = False

    def _refresh_confirm_blocked(self) -> None:
        """Clear a stale CONFIRM_BLOCKED state (CONF-03, D-16) if the drone's
        explicit ``CONFIRM_BLOCKED:clear`` STATUSTEXT was itself lost on the
        shared lossy link — a safety net alongside that explicit clear
        signal. Called every telemetry cycle from :meth:`snapshot`, mirroring
        :meth:`_refresh_prearm_flags`.
        """
        if self._confirm_blocked is None:
            return
        if time.monotonic() - self._confirm_blocked_updated_at > CONFIRM_BLOCKED_STALE_S:
            self._confirm_blocked = None

    @property
    def prearm_mode_not_armable_only(self) -> bool:
        """True when prearm checks failed and the only PreArm reason is 'mode not armable'.

        Pure read of the current state and flags; stale-flag maintenance happens
        in :meth:`_refresh_prearm_flags` (driven by the telemetry cycle).
        """
        return (
            self.vehicle.prearm_check_state == PREARM_STATE_FAILED
            and self._prearm_mode_only
            and not self._prearm_other
        )

    def snapshot(self) -> dict:
        return vehicle_snapshot.snapshot(self)

    def _gimbal_snapshot(self) -> dict:
        return vehicle_camera._gimbal_snapshot(self)
