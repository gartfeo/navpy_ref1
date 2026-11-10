"""The rfdetr backend (HITL step 23b): codec, tiling, class mapping, settings and routing.

Runs without the rfdetr package: a fake runtime returns raw outputs. The parity test against rfdetr's own boxes runs
only where rfdetr and the owner's private weights and clips exist.
"""
from __future__ import annotations

import math
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import pytest

from navpy.core.kernel.config.detector_backend import (
    DETECTOR_BACKENDS,
    RFDETR_BACKEND,
    backend_requires_model_path,
    validate_detector_backend,
)
from navpy.core.perception.profile.vision_class_profile import DOCK_DETECT_CLASS_ID
from navpy.plugins.detection.rfdetr import rfdetr_codec as codec
from navpy.plugins.detection.rfdetr.rfdetr_detector import RfDetrDetector
from navpy.app.perception.rfdetr_profile import whole_dock_settings as rfdetr_settings
from navpy.plugins.detection.rfdetr.rfdetr_settings import RfDetrSettings
from navpy.plugins.detection.rfdetr.rfdetr_tiling import Tile, merge_tiles, tiles

CAR_SLOT, TRUCK_SLOT, PERSON_SLOT = 3, 8, 1     # COCO category ids (the model's logit slots)
BLOCK = {"variant": "large", "classes": {"truck": "dock"}, "input": "squash"}


def _logit(p: float) -> float:
    return math.log(p / (1.0 - p))


def _outputs(rows, queries: int = 6):
    """Raw (1, Q, 4) boxes and (1, Q, 91) logits: rows of (cx, cy, w, h normalised, slot, score)."""
    boxes = np.full((1, queries, 4), 0.5, np.float32)
    logits = np.full((1, queries, codec.LOGIT_SLOTS), -20.0, np.float32)
    for q, (cx, cy, w, h, slot, score) in enumerate(rows):
        boxes[0, q] = (cx, cy, w, h)
        logits[0, q, slot] = _logit(score)
    return boxes, logits


class FakeRuntime:
    device = "cpu"

    def __init__(self, per_input):
        self.per_input = per_input        # callable(blob index) -> rows
        self.blobs = []
        self.closed = False

    def infer(self, blob):
        self.blobs.append(blob)
        outs = [_outputs(self.per_input(i)) for i in range(blob.shape[0])]
        return np.concatenate([o[0] for o in outs]), np.concatenate([o[1] for o in outs])

    def close(self):
        self.closed = True


def _settings(mode: str = "squash") -> RfDetrSettings:
    return rfdetr_settings({"rfdetr": {**BLOCK, "input": mode}})


# --- codec ---------------------------------------------------------------------------------------------------------

def test_decode_maps_coco_category_ids_to_the_80_index_and_keeps_the_floor():
    boxes, logits = _outputs([(0.5, 0.5, 0.1, 0.2, CAR_SLOT, 0.9), (0.2, 0.2, 0.1, 0.1, TRUCK_SLOT, 0.6),
                              (0.3, 0.3, 0.1, 0.1, CAR_SLOT, 0.04)])
    rows = codec.decode(boxes, logits, 1920, 1080, floor=0.05)
    assert [int(r[5]) for r in rows] == [2, 7]                     # car -> 2, truck -> 7; 0.04 under the floor
    assert rows[0][:4] == pytest.approx([960.0, 540.0, 192.0, 216.0])
    assert rows[0][4] == pytest.approx(0.9, abs=1e-5)


def test_decode_drops_slots_that_are_no_coco_class_and_clips_to_the_picture():
    boxes, logits = _outputs([(0.98, 0.5, 0.1, 0.1, 12, 0.9), (0.98, 0.5, 0.1, 0.1, CAR_SLOT, 0.8)])
    rows = codec.decode(boxes, logits, 100, 100, floor=0.05)
    assert len(rows) == 1 and int(rows[0][5]) == 2                 # slot 12 is not a COCO category
    assert rows[0][0] + rows[0][2] / 2 == pytest.approx(100.0)    # clipped at the right edge


def test_decode_keeps_num_select_best_pairs_ties_by_index():
    boxes, logits = _outputs([(0.1 * (q + 1), 0.5, 0.05, 0.05, CAR_SLOT, 0.7) for q in range(5)])
    rows = codec.decode(boxes, logits, 1000, 1000, floor=0.05, num_select=2)
    assert [round(r[0]) for r in rows] == [100, 200]


@pytest.mark.parametrize("boxes,logits", [
    (np.zeros((1, 5, 3)), np.zeros((1, 5, 91))), (np.zeros((1, 5, 4)), np.zeros((1, 5, 80))),
    (np.zeros((1, 5, 4)), np.zeros((1, 4, 91))), (np.full((1, 5, 4), np.nan), np.zeros((1, 5, 91))),
])
def test_decode_refuses_wrong_shapes_and_non_finite_outputs(boxes, logits):
    with pytest.raises(codec.RfDetrOutputError):
        codec.decode(boxes, logits, 10, 10, floor=0.05)


def test_preprocess_squashes_rgb_imagenet_normalised():
    frame = np.zeros((1080, 1920, 3), np.uint8)
    frame[..., 2] = 255                                            # red in BGR
    blob = codec.preprocess(frame, 704)
    assert blob.shape == (1, 3, 704, 704) and blob.dtype == np.float32
    assert blob[0, 0, 0, 0] == pytest.approx((1.0 - 0.485) / 0.229)  # R channel first
    assert blob[0, 2, 0, 0] == pytest.approx((0.0 - 0.406) / 0.225)
    with pytest.raises(ValueError):
        codec.preprocess(np.zeros((10, 10), np.uint8), 704)


# --- tiling --------------------------------------------------------------------------------------------------------

def test_tiles_cover_a_1920_by_1080_frame_with_the_fewest_overlapping_native_tiles():
    grid = tiles(1920, 1080, 704)
    assert [(t.x0, t.y0) for t in grid] == [(0, 0), (608, 0), (1216, 0), (0, 376), (608, 376), (1216, 376)]
    assert all((t.width, t.height) == (704, 704) for t in grid)
    assert tiles(640, 480, 704) == [Tile(0, 0, 640, 480)]


def test_a_box_cut_by_an_inner_tile_edge_yields_to_the_whole_one_in_the_next_tile():
    grid = [Tile(0, 0, 704, 704), Tile(608, 0, 704, 704)]
    cut = np.array([[690.0, 300.0, 28.0, 20.0, 0.95, 2.0]], np.float32)    # x 676-704: on tile 0's inner edge
    whole = np.array([[80.0, 300.0, 60.0, 20.0, 0.80, 2.0]], np.float32)   # frame x 658-718 in tile 1
    rows = merge_tiles([cut, whole], grid, 1920, 1080)
    assert len(rows) == 1 and rows[0][0] == pytest.approx(688.0) and rows[0][2] == pytest.approx(60.0)


def test_two_cut_boxes_never_drop_each_other():
    """Codex round 1: a car across the overlap is cut in both tiles; it must survive as one box."""
    grid = [Tile(0, 0, 704, 704), Tile(608, 0, 704, 704)]
    left = np.array([[652.0, 300.0, 104.0, 20.0, 0.8, 2.0]], np.float32)    # frame x 600-704, cut at 704
    right = np.array([[52.0, 300.0, 104.0, 20.0, 0.9, 2.0]], np.float32)    # frame x 608-712, cut at 608
    rows = merge_tiles([left, right], grid, 1920, 1080)
    assert len(rows) == 1 and rows[0][4] == pytest.approx(0.9)


def test_a_cut_box_alone_is_kept_and_overlapping_duplicates_merge_to_the_best():
    grid = [Tile(0, 0, 704, 704), Tile(608, 0, 704, 704)]
    alone = merge_tiles([np.array([[690.0, 300.0, 28.0, 20.0, 0.9, 2.0]], np.float32), np.zeros((0, 6))],
                        grid, 1920, 1080)
    assert len(alone) == 1
    twice = merge_tiles([np.array([[650.0, 100.0, 20.0, 20.0, 0.7, 2.0]], np.float32),
                         np.array([[42.0, 100.0, 20.0, 20.0, 0.9, 2.0]], np.float32)], grid, 1920, 1080)
    assert len(twice) == 1 and twice[0][4] == pytest.approx(0.9)


# --- detector -------------------------------------------------------------------------------------------------------

def test_squash_keeps_only_trucks_at_the_profile_confidence_as_the_dock_class():
    """Phase A's dock is the pickup, scored as COCO truck: a car is not the dock, however confident."""
    runtime = FakeRuntime(lambda _i: [(0.5, 0.5, 0.02, 0.02, TRUCK_SLOT, 0.8), (0.2, 0.2, 0.02, 0.02, TRUCK_SLOT, 0.3),
                                      (0.7, 0.7, 0.02, 0.02, CAR_SLOT, 0.9), (0.3, 0.6, 0.02, 0.02, PERSON_SLOT, 0.9)])
    detector = RfDetrDetector(runtime, _settings(), 0.45)
    found = detector.detect(np.zeros((1080, 1920, 3), np.uint8))
    assert [(d.class_id, round(d.cx), round(d.cy)) for d in found] == [(DOCK_DETECT_CLASS_ID, 960, 540)]
    assert runtime.blobs[0].shape == (1, 3, 704, 704)
    assert detector.device == "cpu"
    detector.close()
    assert runtime.closed


def test_tiles_run_one_batch_and_map_boxes_back_to_the_full_frame():
    runtime = FakeRuntime(lambda i: [(0.5, 0.5, 0.02, 0.02, TRUCK_SLOT, 0.9)] if i == 5 else [])
    found = RfDetrDetector(runtime, _settings("tiles"), 0.45).detect(np.zeros((1080, 1920, 3), np.uint8))
    assert runtime.blobs[0].shape == (6, 3, 704, 704)
    assert [(round(d.cx), round(d.cy), round(d.w)) for d in found] == [(1216 + 352, 376 + 352, 14)]


def test_no_detection_is_no_detection_there_is_no_fallback():
    assert RfDetrDetector(FakeRuntime(lambda _i: []), _settings(), 0.45).detect(
        np.zeros((1080, 1920, 3), np.uint8)) == []


# --- settings and routing -------------------------------------------------------------------------------------------

def test_settings_map_truck_to_the_dock_and_give_the_input_scale():
    squash, tiled = _settings(), _settings("tiles")
    assert squash.class_map == ((7, DOCK_DETECT_CLASS_ID),) and squash.input_px == 704
    assert squash.navpy_class(7) == DOCK_DETECT_CLASS_ID and squash.navpy_class(2) is None    # car is not the dock
    assert squash.input_scale(1920, 1080) == pytest.approx((704 / 1920, 704 / 1080))
    assert tiled.input_scale(1920, 1080) == (1.0, 1.0)


def test_the_hitl_profile_scores_the_pickup_as_coco_truck_only():
    """Step 23b (owner, 2026-11-09): phase A's dock is a pickup, scored as COCO truck; car is not mapped."""
    from navpy.core.perception.profile.vision_profile_loader import load_profiles
    settings = rfdetr_settings(load_profiles()[0]["siyi_zr10_hitl_encoded"]["detector"])
    assert settings.class_map == ((codec.COCO80_NAMES["truck"], DOCK_DETECT_CLASS_ID),)


@pytest.mark.parametrize("block", [
    None, {}, {**BLOCK, "variant": "medium"}, {**BLOCK, "input": "crop"}, {**BLOCK, "classes": {}},
    {**BLOCK, "classes": {"car": "truck"}}, {**BLOCK, "classes": {"boat": "dock"}}, {**BLOCK, "extra": 1},
    {**BLOCK, "classes": {"truck": "dock", "car": "dock"}},     # one class only: no car-and-truck fallback
])
def test_malformed_rfdetr_blocks_are_refused(block):
    with pytest.raises(ValueError):
        rfdetr_settings({} if block is None else {"rfdetr": block})


@pytest.mark.parametrize("device, expected", [(0, "cuda:0"), ("0", "cuda:0"), ("1", "cuda:1"), ("cpu", "cpu"),
                                              ("cuda:0", "cuda:0")])
def test_cuda_indexes_as_ints_or_numeric_strings_become_torch_devices(device, expected):
    """Codex round 1: navpy's device settings allow a numeric string."""
    from navpy.plugins.detection.rfdetr.rfdetr_runtime import _torch_device
    assert _torch_device(device) == expected


def test_native_tiles_are_normalised_as_each_tile_alone():
    tile = np.random.default_rng(1).integers(0, 256, (2, 704, 704, 3), dtype=np.uint8)
    together = codec.preprocess_tiles(tile)
    assert np.array_equal(together[1:], codec.preprocess(tile[1], 704))
    with pytest.raises(ValueError):
        codec.preprocess_tiles(np.zeros((1, 704, 600, 3), np.uint8))


def test_the_rfdetr_backend_is_named_and_needs_weights():
    assert RFDETR_BACKEND in DETECTOR_BACKENDS and validate_detector_backend("rfdetr") == "rfdetr"
    assert backend_requires_model_path("rfdetr") and not backend_requires_model_path("charuco")


ACQUISITION = {"min_lock_s": 1.0, "min_box_px": 16.0, "recall_min_box_px": 12.0, "recall_evidence": "x"}


def test_factory_builds_rfdetr_with_its_profile_blocks(tmp_path):
    from tests.modules.vision.test_detector_backend_routing import _created_config
    from navpy.app.perception.real_detector_factory import RealDetectorFactory

    weights = tmp_path / "w.pth"
    weights.write_bytes(b"x")
    factory = RealDetectorFactory(Mock(), Mock(), {}, str(weights), False, backend="rfdetr")
    config = _created_config(factory, {"conf": 0.45, "rfdetr": BLOCK, "acquisition": ACQUISITION})
    assert config.model.backend == "rfdetr" and config.model.model_path == str(weights)
    assert config.model.rfdetr == _settings()
    gate = config.acquisition
    assert (gate.min_lock_s, gate.min_box_px, gate.min_confidence, gate.dock_class_id) == (1.0, 16.0, 0.45, 0)
    with pytest.raises(ValueError):
        _created_config(factory, {"conf": 0.45, "rfdetr": BLOCK})                    # no acquisition gates
    with pytest.raises(ValueError):
        RealDetectorFactory(Mock(), Mock(), {}, "", False, backend="rfdetr")         # no weights
    charuco = RealDetectorFactory(Mock(), Mock(), {}, "", False, backend="charuco")
    with pytest.raises(ValueError):
        _created_config(charuco, {"acquisition": ACQUISITION})                       # the gates are rfdetr's


def test_build_models_runs_the_weights_through_the_rfdetr_runtime():
    from navpy.app.perception.real_detector_model_builder import build_models
    from navpy.core.perception.pipeline.real_detector_config import DetectorModelConfig, RealDetectorConfig
    from navpy.core.perception.pipeline.real_detector_runtime_state import RuntimeMetrics

    runtime = FakeRuntime(lambda _i: [])
    with patch("navpy.plugins.detection.rfdetr.rfdetr_runtime.TorchRfDetrRuntime", return_value=runtime) as ctor:
        models = build_models(Mock(), RealDetectorConfig(DetectorModelConfig(
            "w.pth", backend="rfdetr", rfdetr=_settings(), conf=0.45, device="cpu",
            tracker={"backend": "custom"}, appearance={"enabled": False})), RuntimeMetrics())
    ctor.assert_called_once_with("w.pth", "large", "cpu")
    assert isinstance(models.frame_detector, RfDetrDetector) and models.board_pose is None
    with pytest.raises(ValueError):
        DetectorModelConfig("w.pth", backend="rfdetr")                              # settings go with the backend


# --- parity with rfdetr's own predict(), where the package, weights and clips exist ---------------------------------

EVAL = Path.home() / "AppData" / "Local" / "cartrack-eval"
WEIGHTS = EVAL / "models" / "rfdetr_l" / "rf-detr-large-2026.pth"
CLIP = EVAL / "clips" / "d5_view"


def test_squash_matches_rfdetr_own_boxes_on_a_real_zr10_frame():
    pytest.importorskip("rfdetr")
    if not (WEIGHTS.is_file() and (CLIP / "dets_rfdetr_l.npz").is_file()):
        pytest.skip("the owner's private RF-DETR-L weights and clips are not here")
    import cv2
    from navpy.plugins.detection.rfdetr.rfdetr_runtime import TorchRfDetrRuntime

    ref = np.load(CLIP / "dets_rfdetr_l.npz")
    rows = ref["dets"][:ref["n"][0]]
    rows = rows[(rows[:, 5] == 2) & (rows[:, 4] >= 0.45)]
    cars = rfdetr_settings({"rfdetr": {**BLOCK, "classes": {"car": "dock"}}})    # the clip's reference is cars
    runtime = TorchRfDetrRuntime(str(WEIGHTS), "large", "auto")
    try:
        found = RfDetrDetector(runtime, cars, 0.45).detect(
            cv2.imread(str(CLIP / "frames" / f"{int(ref['index'][0])}.jpg")))
    finally:
        runtime.close()
    assert len(found) == len(rows)
    ours = sorted((d.cx, d.cy, d.w, d.h) for d in found)
    theirs = sorted(tuple(r[:4]) for r in rows)
    assert np.abs(np.array(ours) - np.array(theirs)).max() < 2.0
