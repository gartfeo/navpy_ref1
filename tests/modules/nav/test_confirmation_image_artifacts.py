import base64
import json

import cv2
import numpy as np
import pytest

from navpy.core.review.confirmation_image_artifacts import save_confirmation_image_artifacts


def test_saves_source_thumbnail_and_metadata(tmp_path):
    source_frame = np.zeros((24, 32, 3), dtype=np.uint8)
    source_frame[2, 3] = (10, 20, 30)
    sent_thumb = np.zeros((12, 16, 3), dtype=np.uint8)
    sent_thumb[:, :] = (5, 15, 25)
    ok, encoded = cv2.imencode(".jpg", sent_thumb, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    sent_bytes = encoded.tobytes()

    artifacts = save_confirmation_image_artifacts(
        log_path=tmp_path,
        sys_id=7,
        poi_id=303,
        source_frame=source_frame,
        sent_image_b64=base64.b64encode(sent_bytes).decode("utf-8"),
        bbox_cxcywh=(11.0, 12.0, 30.0, 40.0),
        class_id=np.int32(2),
        class_name="vehicle",
        confidence=np.float32(0.875),
        frame_bboxes=[(1.0, 2.0, 3.0, 4.0)],
        confirmation_degraded=True,
    )

    assert artifacts is not None
    # Named by vehicle, POI and round, never by wall time.
    assert artifacts.source_path.name == "uav_7_confirmation_t303_r1_source.png"
    assert artifacts.sent_thumbnail_path.name == "uav_7_confirmation_t303_r1_sent_thumb.jpg"
    assert artifacts.metadata_path.name == "uav_7_confirmation_t303_r1_meta.json"

    saved_source = cv2.imread(str(artifacts.source_path), cv2.IMREAD_COLOR)
    assert saved_source.shape == source_frame.shape
    assert saved_source[2, 3].tolist() == [10, 20, 30]
    assert artifacts.sent_thumbnail_path.read_bytes() == sent_bytes

    metadata = json.loads(artifacts.metadata_path.read_text(encoding="utf-8"))
    assert metadata["sys_id"] == 7
    assert metadata["poi_id"] == 303
    assert metadata["class_id"] == 2
    assert metadata["class_name"] == "vehicle"
    assert metadata["confidence"] == float(np.float32(0.875))
    assert metadata["confirmation_degraded"] is True
    assert metadata["bbox_cxcywh"] == [11.0, 12.0, 30.0, 40.0]
    assert metadata["bbox_height_px"] == 40.0
    assert metadata["frame_shape"] == [24, 32, 3]
    assert metadata["frame_bboxes"] == [[1.0, 2.0, 3.0, 4.0]]
    assert metadata["artifacts"]["source"] == artifacts.source_path.name
    assert metadata["artifacts"]["sent_thumbnail"] == artifacts.sent_thumbnail_path.name
    assert metadata["artifacts"]["metadata"] == artifacts.metadata_path.name


def test_returns_none_without_log_path(tmp_path):
    frame = np.zeros((4, 4, 3), dtype=np.uint8)

    artifacts = save_confirmation_image_artifacts(
        log_path=None,
        sys_id=1,
        poi_id=2,
        source_frame=frame,
        sent_image_b64=base64.b64encode(b"jpeg bytes").decode("utf-8"),
        bbox_cxcywh=(1.0, 2.0, 3.0, 4.0),
        class_id=0,
        class_name="poi",
        confidence=1.0,
        frame_bboxes=None,
        confirmation_degraded=False,
    )

    assert artifacts is None
    assert list(tmp_path.iterdir()) == []


def _save(tmp_path, *, frame=None):
    return save_confirmation_image_artifacts(
        log_path=tmp_path,
        sys_id=3,
        poi_id=1,
        source_frame=np.zeros((4, 4, 3), dtype=np.uint8) if frame is None else frame,
        sent_image_b64=base64.b64encode(b"jpeg bytes").decode("utf-8"),
        bbox_cxcywh=(1.0, 2.0, 3.0, 4.0),
        class_id=0,
        class_name="poi",
        confidence=1.0,
        frame_bboxes=None,
        confirmation_degraded=False,
    )


def test_a_second_round_for_the_same_poi_gets_the_next_number(tmp_path):
    first = _save(tmp_path)
    second = _save(tmp_path)

    assert first.metadata_path.name == "uav_3_confirmation_t1_r1_meta.json"
    assert second.metadata_path.name == "uav_3_confirmation_t1_r2_meta.json"


def test_a_failed_write_releases_its_round(tmp_path, monkeypatch):
    monkeypatch.setattr(cv2, "imwrite", lambda *_args: False)
    with pytest.raises(OSError, match="source image"):
        _save(tmp_path)
    monkeypatch.undo()

    assert _save(tmp_path).metadata_path.name == "uav_3_confirmation_t1_r1_meta.json"


def test_a_failed_metadata_write_removes_the_round(tmp_path, monkeypatch):
    from pathlib import Path

    real_write_text = Path.write_text

    def fail_metadata(self, *args, **kwargs):
        if self.name.endswith("_meta.json"):
            raise OSError("disk full")
        return real_write_text(self, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail_metadata)
    with pytest.raises(OSError, match="disk full"):
        _save(tmp_path)
    monkeypatch.undo()

    assert list(tmp_path.iterdir()) == []
    assert _save(tmp_path).metadata_path.name == "uav_3_confirmation_t1_r1_meta.json"
