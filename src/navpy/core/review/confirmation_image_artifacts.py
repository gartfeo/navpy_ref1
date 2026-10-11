from __future__ import annotations

import base64
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class ConfirmationImageArtifacts:
    source_path: Path
    sent_thumbnail_path: Path
    metadata_path: Path


def save_confirmation_image_artifacts(
        *,
        log_path: str | Path | None,
        sys_id: int,
        poi_id: int,
        source_frame: np.ndarray,
        sent_image_b64: str,
        bbox_cxcywh: Sequence[float],
        class_id: int | None,
        class_name: str | None,
        confidence: float | None,
        frame_bboxes: Iterable[Sequence[float]] | None,
        confirmation_degraded: bool,
        timestamp: datetime | None = None,
) -> ConfirmationImageArtifacts | None:
    """Save full-source and sent-thumbnail confirmation image artifacts."""
    if not log_path:
        return None
    if source_frame is None or source_frame.size == 0 or not sent_image_b64:
        return None

    log_dir = Path(log_path)
    log_dir.mkdir(parents=True, exist_ok=True)

    ts = (timestamp or datetime.now()).strftime("%H%M%S%f")
    prefix = f"uav_{int(sys_id)}_confirmation_t{int(poi_id)}_{ts}"
    source_path = log_dir / f"{prefix}_source.png"
    sent_thumbnail_path = log_dir / f"{prefix}_sent_thumb.jpg"
    metadata_path = log_dir / f"{prefix}_meta.json"

    if not cv2.imwrite(str(source_path), source_frame):
        raise OSError(f"failed to write source image: {source_path}")

    sent_bytes = base64.b64decode(sent_image_b64)
    sent_thumbnail_path.write_bytes(sent_bytes)

    bbox = _json_float_list(bbox_cxcywh)
    frame_bbox_values = None
    if frame_bboxes is not None:
        frame_bbox_values = [_json_float_list(b) for b in frame_bboxes]

    metadata = {
        "sys_id": int(sys_id),
        "poi_id": int(poi_id),
        "class_id": _json_optional_int(class_id),
        "class_name": class_name,
        "confidence": _json_optional_float(confidence),
        "confirmation_degraded": bool(confirmation_degraded),
        "bbox_cxcywh": bbox,
        "bbox_height_px": bbox[3] if len(bbox) >= 4 else None,
        "frame_shape": [int(v) for v in source_frame.shape],
        "frame_bboxes": frame_bbox_values,
        "artifacts": {
            "source": source_path.name,
            "sent_thumbnail": sent_thumbnail_path.name,
            "metadata": metadata_path.name,
        },
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")

    return ConfirmationImageArtifacts(
        source_path=source_path,
        sent_thumbnail_path=sent_thumbnail_path,
        metadata_path=metadata_path,
    )


def _json_float_list(values: Sequence[float]) -> list[float]:
    return [float(v) for v in values]


def _json_optional_float(value) -> float | None:
    if value is None:
        return None
    return float(value)


def _json_optional_int(value) -> int | None:
    if value is None:
        return None
    return int(value)
