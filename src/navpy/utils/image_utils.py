"""Thumbnail orchestration, codecs, and persistence for POI confirmation."""

from __future__ import annotations

import base64

import cv2
import numpy as np

from navpy.utils.thumbnail_rendering import (
    PoiInfo,
    crop_poi,
    draw_low_res_banner,
    draw_poi_box,
)


THUMBNAIL_SIZE = (640, 480)
JPEG_QUALITY = 80


def create_detection_thumbnail(
    frame: np.ndarray,
    pois: list[PoiInfo],
    size: tuple[int, int] = THUMBNAIL_SIZE,
    jpeg_quality: int = JPEG_QUALITY,
) -> str | None:
    """Create a resized JPEG thumbnail with every supplied POI marked."""
    if frame is None or frame.size == 0:
        return None

    try:
        image = frame.copy()
        for poi in pois:
            draw_poi_box(
                image,
                poi.poi_id,
                poi.bbox,
                poi.confidence,
                poi.class_name,
            )
        resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        success, encoded = cv2.imencode(
            ".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        )
        if not success:
            return None
        return base64.b64encode(encoded.tobytes()).decode("utf-8")
    except Exception:
        return None


def create_confirmation_thumbnail(
    frame: np.ndarray,
    poi_id: int,
    bbox: tuple[float, float, float, float],
    confidence: float = 1.0,
    class_name: str = "Detection",
    frame_bboxes: list[tuple[float, float, float, float]] | None = None,
    size: tuple[int, int] = THUMBNAIL_SIZE,
    jpeg_quality: int = JPEG_QUALITY,
    crop_to_poi: bool = True,
    degraded: bool = False,
) -> str | None:
    """Create a confirmation thumbnail centered on the selected POI.

    ``frame_bboxes`` remains accepted for artifact-metadata compatibility but
    deliberately does not widen the crop around the selected POI.
    """
    del frame_bboxes
    if frame is None or frame.size == 0:
        return None

    if crop_to_poi:
        source, adjusted_bbox = crop_poi(frame, bbox, size)
    else:
        source, adjusted_bbox = frame, bbox

    poi = PoiInfo(poi_id, adjusted_bbox, confidence, class_name)
    encoded = create_detection_thumbnail(source, [poi], size, jpeg_quality)
    if degraded and encoded is not None:
        return _reencode_with_low_res_banner(encoded, size, jpeg_quality)
    return encoded


def _reencode_with_low_res_banner(
    b64_str: str,
    size: tuple[int, int],
    jpeg_quality: int,
) -> str | None:
    """Overlay a warning banner while preserving the prior fallback behavior."""
    del size
    image = decode_confirmation_thumbnail(b64_str)
    if image is None:
        return b64_str

    draw_low_res_banner(image)
    try:
        success, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        )
        if not success:
            return b64_str
        return base64.b64encode(encoded.tobytes()).decode("utf-8")
    except Exception:
        return b64_str


def decode_confirmation_thumbnail(b64_str: str) -> np.ndarray | None:
    """Decode a base64 JPEG thumbnail into a BGR image."""
    if not b64_str:
        return None
    try:
        image_bytes = base64.b64decode(b64_str)
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        return cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    except Exception:
        return None


def save_confirmation_image(b64_str: str, filepath: str) -> bool:
    """Decode and save a confirmation image."""
    image = decode_confirmation_thumbnail(b64_str)
    if image is None:
        return False
    try:
        cv2.imwrite(filepath, image)
        return True
    except Exception:
        return False


# Historical private aliases kept for compatible imports while rendering lives
# in its focused module.
_draw_low_res_banner = draw_low_res_banner
_draw_poi_box = draw_poi_box


__all__ = [
    "JPEG_QUALITY",
    "THUMBNAIL_SIZE",
    "PoiInfo",
    "create_confirmation_thumbnail",
    "create_detection_thumbnail",
    "decode_confirmation_thumbnail",
    "save_confirmation_image",
]
