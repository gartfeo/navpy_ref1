"""One camera component of a vehicle, for the life of its GCS entry.

Display only (docs/design/gcs_preflight_camera_row.md, Backend, boot rules).
The record outlives discovery generations: it keeps the boot ids NavPy has
started generations with, whether the component has answered as NavPy's
camera, and whether it is lost. A lost camera stays Lost, with no discovery
updates or requests, until an admitted HEARTBEAT starts a new generation.

NavPy sends its boot id in the camera HEARTBEAT's custom_mode (0 for older
NavPy). Random ids have no order, so only a NavPy-qualified camera reads it:
an id new to the record means NavPy restarted, an id of an older generation
is a late packet of the old process and is dropped whole.

Pure state: every method takes ``now`` (monotonic seconds).
"""
from __future__ import annotations

from typing import Optional

from pymavlink.dialects.v20.ardupilotmega import MAV_STATE_ACTIVE, MAV_STATE_CRITICAL

from gcs.backend.video_camera_discovery import (
    PHASE_DONE, PHASE_FAILED, CameraDiscovery, CameraRequest, VideoStream,
)
from navpy.core.ports.camera_health import NAVPY_CAMERA_VENDOR, NAVPY_SIM_CAMERA_VENDOR

_VENDOR_KINDS = {NAVPY_CAMERA_VENDOR: "navpy", NAVPY_SIM_CAMERA_VENDOR: "navpy_sim"}
_STATUS_NAMES = {MAV_STATE_ACTIVE: "active", MAV_STATE_CRITICAL: "critical"}


class CameraRecord:
    """Boot rules and Lost state around one component's discovery generations."""

    def __init__(self, component: int) -> None:
        self.component = component
        self.discovery: Optional[CameraDiscovery] = None
        self.lost = False
        self._navpy = False  # answered as NavPy's camera; kept across generations
        self._boot_id = 0  # the current generation's
        self._boot_ids: set[int] = set()  # every non-zero generation boot id

    def admit_heartbeat(self, boot_id: int, system_status: Optional[int], now: float) -> bool:
        """Apply one camera HEARTBEAT; False when it is dropped whole."""
        discovery = self.discovery
        if discovery is None:
            self._start(boot_id, now)
        elif self._navpy and boot_id not in (0, self._boot_id):
            if boot_id in self._boot_ids:
                return False  # a late packet of an older NavPy process
            self._start(boot_id, now)  # NavPy restarted
        elif self.lost or discovery.is_lost(now):
            self._start(boot_id, now)
        assert self.discovery is not None
        self.discovery.heartbeat(now, system_status)
        return True

    def became_lost(self, now: float) -> bool:
        """Mark the camera lost after its heartbeats stopped; True on the change."""
        if self.lost or self.discovery is None or not self.discovery.is_lost(now):
            return False
        self.lost = True
        return True

    def on_camera_information(self, flags: int, vendor: str) -> None:
        if self.lost or self.discovery is None:
            return
        if self.discovery.on_camera_information(flags, vendor):
            self._navpy = vendor in _VENDOR_KINDS

    def on_ack(self, result: int) -> None:
        if not self.lost and self.discovery is not None:
            self.discovery.on_ack(result)

    def on_stream_information(
        self, stream_id: int, count: int, stream: Optional[VideoStream], rejected_reason: str,
    ) -> None:
        if not self.lost and self.discovery is not None:
            self.discovery.on_stream_information(stream_id, count, stream, rejected_reason)

    def poll(self, now: float) -> Optional[CameraRequest]:
        if self.lost or self.discovery is None:
            return None
        return self.discovery.poll(now)

    def snapshot(self) -> dict:
        discovery = self.discovery
        assert discovery is not None
        vendor = discovery.vendor
        return {
            "component": self.component,
            "lost": self.lost,
            "vendor": None if vendor is None else _VENDOR_KINDS.get(vendor, "other"),
            "status": _STATUS_NAMES.get(discovery.system_status, "other"),
            "discovery": (
                "done" if discovery.phase == PHASE_DONE
                else "failed" if discovery.phase == PHASE_FAILED
                else "running"
            ),
            "has_video": discovery.has_video,
            "rejected": [
                {"stream_id": stream_id, "reason": reason}
                for stream_id, reason in discovery.rejected()
            ],
        }

    def _start(self, boot_id: int, now: float) -> None:
        self.discovery = CameraDiscovery(self.component, now)
        self.lost = False
        self._boot_id = boot_id
        if boot_id:
            self._boot_ids.add(boot_id)
