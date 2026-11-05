"""The search sweep from a device's ``gimbal.search`` profile section (HITL step 21).

Built only beside integration timing, like fixed zoom: a device without the section gets no scan, and a device
with it needs the run's command fence as its revoke source, or the vision start is refused.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping

from navpy.core.command.gimbal.gimbal_search import GimbalSearchSetup
from navpy.core.command.gimbal.gimbal_search_pattern import GimbalSearchPattern


def build_gimbal_search(
    device: Mapping[str, object],
    revoked: Callable[[], bool] | None,
) -> GimbalSearchSetup | None:
    gimbal = device.get("gimbal")
    if not isinstance(gimbal, Mapping) or "search" not in gimbal:
        return None
    spec = gimbal["search"]
    if not isinstance(spec, Mapping):
        raise ValueError(f"gimbal search: the profile's search is not a mapping ({spec!r})")
    if revoked is None:
        raise ValueError("gimbal search needs the SIYI command fence as its revoke source")
    return GimbalSearchSetup(GimbalSearchPattern.from_profile(spec), revoked)


__all__ = ["build_gimbal_search"]
