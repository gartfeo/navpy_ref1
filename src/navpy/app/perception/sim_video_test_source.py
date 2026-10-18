"""Announce the sim video test stream from a simulated companion's camera.

The GCS launcher gives a sim companion an RTSP URI and a name in its
environment when ``simulation.video_test_source`` is on; this adds that one
stream to the profile's first camera device. Without the environment nothing
changes. Operator display only (docs/design/gcs_video_streams.md, section 6):
it feeds no navigation, and the camera protocol code is untouched.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping

from navpy.core.perception.profile.vision_profile_loader import get_devices
from navpy.core.perception.profile.vision_video_streams import VIDEO_STREAMS_KEY

SIM_VIDEO_URI_ENV = "NAVPY_SIM_VIDEO_URI"
SIM_VIDEO_NAME_ENV = "NAVPY_SIM_VIDEO_NAME"

# What the GCS launcher's test pattern publishes (gcs.backend.sim_video_config).
SIM_VIDEO_STREAM_ID = 1
SIM_VIDEO_WIDTH = 640
SIM_VIDEO_HEIGHT = 360
SIM_VIDEO_FRAMERATE = 15.0


def with_sim_video_stream(
    profile: Mapping[str, object],
    environ: Mapping[str, str],
) -> Mapping[str, object]:
    """*profile*, plus the env-named test stream on its first device.

    Returns *profile* itself when the environment names no stream or the first
    device already announces its own streams (a profile's streams win).
    """
    uri = environ.get(SIM_VIDEO_URI_ENV, "").strip()
    if not uri:
        return profile
    devices = get_devices(profile)
    if not devices or VIDEO_STREAMS_KEY in devices[0]:
        return profile
    name = environ.get(SIM_VIDEO_NAME_ENV, "").strip() or "SIM TEST"
    result = copy.deepcopy(dict(profile))
    get_devices(result)[0][VIDEO_STREAMS_KEY] = [{
        "stream_id": SIM_VIDEO_STREAM_ID,
        "name": name,
        "uri": uri,
        "encoding": "H264",
        "resolution_h": SIM_VIDEO_WIDTH,
        "resolution_v": SIM_VIDEO_HEIGHT,
        "framerate": SIM_VIDEO_FRAMERATE,
    }]
    return result
