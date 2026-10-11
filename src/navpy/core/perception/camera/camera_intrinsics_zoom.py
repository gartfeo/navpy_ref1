"""Interpolation policy for uncalibrated camera zoom commands."""

from __future__ import annotations

from collections.abc import Callable


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def lerp_entry(
    e0: dict[str, object],
    e1: dict[str, object],
    t: float,
    interpolate: Callable[[float, float, float], float] = lerp,
) -> dict[str, object]:
    """Linearly interpolate or extrapolate between calibrated entries."""
    distortion = e0["dist"] if t <= 0.5 else e1["dist"]
    return {
        "fx": interpolate(float(e0["fx"]), float(e1["fx"]), t),
        "fy": interpolate(float(e0["fy"]), float(e1["fy"]), t),
        "cx": interpolate(float(e0["cx"]), float(e1["cx"]), t),
        "cy": interpolate(float(e0["cy"]), float(e1["cy"]), t),
        "skew": interpolate(float(e0["skew"]), float(e1["skew"]), t),
        "dist": distortion,
        "image_width": e0["image_width"],
        "image_height": e0["image_height"],
    }


def scale_entry(
    entry: dict[str, object],
    scale: float,
) -> dict[str, object]:
    """Scale focal length from a single calibrated entry."""
    return {
        "fx": float(entry["fx"]) * scale,
        "fy": float(entry["fy"]) * scale,
        "cx": entry["cx"],
        "cy": entry["cy"],
        "skew": entry["skew"],
        "dist": entry["dist"],
        "image_width": entry["image_width"],
        "image_height": entry["image_height"],
    }


class CameraIntrinsicsZoomPolicyMixin:
    """Legacy zoom-estimation extension points without intrinsic state."""

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return lerp(a, b, t)

    @classmethod
    def _lerp_entry(
        cls,
        e0: dict[str, object],
        e1: dict[str, object],
        t: float,
    ) -> dict[str, object]:
        return lerp_entry(e0, e1, t, cls._lerp)

    @staticmethod
    def _scale_entry(
        entry: dict[str, object],
        scale: float,
    ) -> dict[str, object]:
        return scale_entry(entry, scale)


def estimate_intrinsics(
    zoom_map: dict[str, dict[str, object]],
    zoom: object,
    *,
    scale: Callable[
        [dict[str, object], float],
        dict[str, object],
    ] = scale_entry,
    interpolate: Callable[
        [dict[str, object], dict[str, object], float],
        dict[str, object],
    ] = lerp_entry,
) -> dict[str, object] | None:
    """Estimate an uncalibrated zoom using the legacy linear policy."""
    try:
        requested = float(zoom)
    except (TypeError, ValueError):
        return None

    levels: list[tuple[float, dict[str, object]]] = []
    for key, entry in zoom_map.items():
        try:
            levels.append((float(key), entry))
        except (TypeError, ValueError):
            continue
    if not levels:
        return None
    levels.sort(key=lambda item: item[0])

    if len(levels) == 1:
        calibrated, entry = levels[0]
        if calibrated == 0:
            return None
        return scale(entry, requested / calibrated)

    if requested <= levels[0][0]:
        z0, e0 = levels[0]
        z1, e1 = levels[1]
    elif requested >= levels[-1][0]:
        z0, e0 = levels[-2]
        z1, e1 = levels[-1]
    else:
        for index in range(len(levels) - 1):
            if levels[index][0] <= requested <= levels[index + 1][0]:
                z0, e0 = levels[index]
                z1, e1 = levels[index + 1]
                break
        else:
            return None

    delta = z1 - z0
    if delta == 0:
        return dict(e0)
    return interpolate(e0, e1, (requested - z0) / delta)


__all__ = [
    "CameraIntrinsicsZoomPolicyMixin",
    "estimate_intrinsics",
    "lerp",
    "lerp_entry",
    "scale_entry",
]
