from __future__ import annotations

import base64
import json
from dataclasses import dataclass
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
) -> ConfirmationImageArtifacts | None:
    """Save full-source and sent-thumbnail confirmation image artifacts.

    Named by vehicle, POI and confirmation round (``_r1``, ``_r2``...), never
    by wall time, so equal runs write equal names.
    """
    if not log_path:
        return None
    if source_frame is None or source_frame.size == 0 or not sent_image_b64:
        return None

    log_dir = Path(log_path)
    log_dir.mkdir(parents=True, exist_ok=True)

    bbox = _json_float_list(bbox_cxcywh)
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
        "frame_bboxes": (
            None if frame_bboxes is None
            else [_json_float_list(b) for b in frame_bboxes]
        ),
    }
    sent_bytes = base64.b64decode(sent_image_b64)

    prefix, metadata_path = _claim_round(log_dir, int(sys_id), int(poi_id))
    source_path = log_dir / f"{prefix}_source.png"
    sent_thumbnail_path = log_dir / f"{prefix}_sent_thumb.jpg"
    metadata["artifacts"] = {
        "source": source_path.name,
        "sent_thumbnail": sent_thumbnail_path.name,
        "metadata": metadata_path.name,
    }
    # One transaction: a failure removes this round's files, which frees the
    # round, so a retry neither skips a number nor leaves a partial artifact.
    try:
        if not cv2.imwrite(str(source_path), source_frame):
            raise OSError(f"failed to write source image: {source_path}")
        sent_thumbnail_path.write_bytes(sent_bytes)
        metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )
    except BaseException:
        for path in (source_path, sent_thumbnail_path, metadata_path):
            path.unlink(missing_ok=True)
        raise

    return ConfirmationImageArtifacts(
        source_path=source_path,
        sent_thumbnail_path=sent_thumbnail_path,
        metadata_path=metadata_path,
    )


def _claim_round(log_dir: Path, sys_id: int, poi_id: int) -> tuple[str, Path]:
    """The next free round for this POI, claimed by creating its metadata file."""
    round_number = 1
    while True:
        prefix = f"uav_{sys_id}_confirmation_t{poi_id}_r{round_number}"
        metadata_path = log_dir / f"{prefix}_meta.json"
        try:
            metadata_path.open("x").close()
        except FileExistsError:
            round_number += 1
            continue
        return prefix, metadata_path


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
