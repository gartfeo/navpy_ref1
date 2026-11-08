"""The SIYI gimbal address from the command line, over the profile's ``siyi_ip``.

HITL runner only (tools/tuning/approach.py): navpy runs in WSL and its ZR10 emulator on Windows, which stamps the
PC's one clock (hitl_guidance.md, "One time axis"); the Windows host's address seen from WSL changes when
WSL restarts, so no profile can hold it. The profile file itself is untouched.
"""

from __future__ import annotations

import copy
import ipaddress
from collections.abc import Mapping

from navpy.core.perception.profile.vision_profile_loader import get_devices


def with_siyi_ip(profile: Mapping[str, object], siyi_ip: str) -> Mapping[str, object]:
    """*profile* itself when *siyi_ip* is empty; else a copy whose every SIYI gimbal device has that address."""
    if not siyi_ip:
        return profile
    ipaddress.ip_address(siyi_ip)                 # a malformed address fails at start, not at the first packet
    result = copy.deepcopy(dict(profile))
    gimbals = [device["gimbal"] for device in get_devices(result)
               if isinstance(device.get("gimbal"), dict) and device["gimbal"].get("type") == "siyi"]
    if not gimbals:
        raise ValueError(f"--siyi-ip {siyi_ip}: the vision profile has no SIYI gimbal")
    for gimbal in gimbals:
        gimbal["siyi_ip"] = siyi_ip
    return result


__all__ = ["with_siyi_ip"]
