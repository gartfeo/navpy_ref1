"""Normalization policy for JSON-shaped camera intrinsic calibrations."""

from __future__ import annotations


def normalize_zoom_map(
    zoom_map: dict[str, dict] | None,
    image_width: int | None,
    image_height: int | None,
) -> dict[str, dict]:
    """Normalize legacy profile aliases without changing validation behavior."""
    if not zoom_map:
        return {}

    normalized: dict[str, dict] = {}
    for key, entry in zoom_map.items():
        if not isinstance(entry, dict):
            raise ValueError(f"Zoom entry '{key}' must be a JSON object.")

        missing = [name for name in ("fx", "fy", "cx", "cy") if name not in entry]
        if missing:
            raise ValueError(
                f"Zoom entry '{key}' missing keys: {', '.join(missing)}"
            )

        skew = entry.get("skew", entry.get("alpha", entry.get("alpha_c", 0.0)))
        distortion = entry.get(
            "dist",
            entry.get("distortion", entry.get("distortion_coeffs")),
        )
        width = entry.get("image_width", image_width)
        height = entry.get("image_height", image_height)
        if width is None:
            width = int(round(float(entry["cx"]) * 2))
        if height is None:
            height = int(round(float(entry["cy"]) * 2))

        normalized[str(key)] = {
            "fx": entry["fx"],
            "fy": entry["fy"],
            "cx": entry["cx"],
            "cy": entry["cy"],
            "skew": skew,
            "dist": distortion,
            "image_width": width,
            "image_height": height,
        }
    return normalized


__all__ = ["normalize_zoom_map"]
