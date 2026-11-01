"""Preflight camera fields that need a clock or more than one vehicle.

Display only (docs/design/gcs_preflight_camera_row.md, Backend).
:meth:`CameraPreflight.annotate` runs at the end of
``VehicleManager.get_all_snapshots``, after ``companion_status`` is set, and
adds to each snapshot:

- ``camera_wait_over``: ``CAMERA_ADVERTISE_GRACE_S`` have passed since the
  companion came up (its status left "down"). Timed here because the
  telemetry loop broadcasts only changes: a bool that flips once reaches the
  browser, a ticking age would not;
- ``video_stream_expected``: the sim test source is on for this sys_id;
- ``video_stream_duplicates``: ``[{key, sys_ids}]``, each stream another UAV
  also announces, with those other UAVs. Streams are grouped by WHEP URL:
  the fleet shares one WHEP port, so the URL is the RTSP host and path, and
  it is what the browser would play.
"""
from __future__ import annotations

from collections import defaultdict
from typing import Callable

from gcs.backend.sim_video_companion import video_stream_expected
from navpy.core.kernel import runtime_time as rt

# Companion up this long with no camera recorded: the detector camera row
# reads Not advertised. Provisional: the Orin bench check measures the
# companion-to-camera delay.
CAMERA_ADVERTISE_GRACE_S = 15.0


class CameraPreflight:
    """The companion wait clock per sys_id and the fleet duplicate check."""

    def __init__(
        self,
        clock: Callable[[], float] = rt.monotonic,
        expected: Callable[[int], bool] = video_stream_expected,
    ) -> None:
        self._clock = clock
        self._expected = expected
        self._up_since: dict[int, float] = {}

    def annotate(self, snapshots: list[dict]) -> None:
        now = self._clock()
        seen = {snap["sys_id"] for snap in snapshots}
        for sys_id in set(self._up_since) - seen:
            del self._up_since[sys_id]
        owners: dict[str, set[int]] = defaultdict(set)
        for snap in snapshots:
            for stream in snap.get("video_streams", ()):
                owners[stream["whep_url"]].add(snap["sys_id"])
        for snap in snapshots:
            sys_id = snap["sys_id"]
            if snap.get("companion_status") == "down":
                self._up_since.pop(sys_id, None)
            else:
                self._up_since.setdefault(sys_id, now)
            up_since = self._up_since.get(sys_id)
            snap["camera_wait_over"] = (
                up_since is not None and now - up_since >= CAMERA_ADVERTISE_GRACE_S
            )
            snap["video_stream_expected"] = self._expected(sys_id)
            snap["video_stream_duplicates"] = [
                {"key": stream["key"], "sys_ids": sorted(others)}
                for stream in snap.get("video_streams", ())
                if (others := owners[stream["whep_url"]] - {sys_id})
            ]
