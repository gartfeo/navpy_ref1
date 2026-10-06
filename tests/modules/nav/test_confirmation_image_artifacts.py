import base64
import json
from datetime import datetime

import cv2
import numpy as np

from navpy.modules.nav.confirmation_image_artifacts import save_confirmation_image_artifacts


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
        target_id=303,
        source_frame=source_frame,
        sent_image_b64=base64.b64encode(sent_bytes).decode("utf-8"),
        bbox_cxcywh=(11.0, 12.0, 30.0, 40.0),
        class_id=np.int32(2),
        class_name="vehicle",
        confidence=np.float32(0.875),
        frame_bboxes=[(1.0, 2.0, 3.0, 4.0)],
        confirmation_degraded=True,
        timestamp=datetime(2026, 5, 20, 6, 7, 8, 9001),
    )

    assert artifacts is not None
    assert artifacts.source_path.name == "uav_7_confirmation_t303_060708009001_source.png"
    assert artifacts.sent_thumbnail_path.name == "uav_7_confirmation_t303_060708009001_sent_thumb.jpg"
    assert artifacts.metadata_path.name == "uav_7_confirmation_t303_060708009001_meta.json"

    saved_source = cv2.imread(str(artifacts.source_path), cv2.IMREAD_COLOR)
    assert saved_source.shape == source_frame.shape
    assert saved_source[2, 3].tolist() == [10, 20, 30]
    assert artifacts.sent_thumbnail_path.read_bytes() == sent_bytes

    metadata = json.loads(artifacts.metadata_path.read_text(encoding="utf-8"))
    assert metadata["sys_id"] == 7
    assert metadata["target_id"] == 303
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
        target_id=2,
        source_frame=frame,
        sent_image_b64=base64.b64encode(b"jpeg bytes").decode("utf-8"),
        bbox_cxcywh=(1.0, 2.0, 3.0, 4.0),
        class_id=0,
        class_name="target",
        confidence=1.0,
        frame_bboxes=None,
        confirmation_degraded=False,
    )

    assert artifacts is None
    assert list(tmp_path.iterdir()) == []
