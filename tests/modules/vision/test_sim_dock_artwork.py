"""Exercise the production confirmation renderer with actual image files."""

import cv2
import numpy as np

from navpy.modules.vision.sim.sim_frame_generator import SimFrameGenerator


def test_background_alone_renders_default_dock_at_existing_canvas_size(tmp_path):
    background = np.zeros((480, 640, 3), dtype=np.uint8)
    assert cv2.imwrite(str(tmp_path / "background.png"), background)
    renderer = SimFrameGenerator(str(tmp_path), (640, 480))
    assert renderer.is_available
    assert (renderer.sprite_width_for(None), renderer.sprite_height_for(None)) == (652, 442)
    frame, boxes = renderer.generate_frame([(0.5, 0.5, 0.25)])
    assert boxes == [(320, 240, 163, 110)]
    assert np.any(frame[185:295, 239:402])
    assert not np.any(frame[:180])


def test_custom_dock_artwork_is_loaded_without_changing_compositing(tmp_path):
    assert cv2.imwrite(str(tmp_path / "background.png"), np.zeros((480, 640, 3), np.uint8))
    sprite = np.full((60, 80, 4), 255, np.uint8)
    assert cv2.imwrite(str(tmp_path / "dock.png"), sprite)
    renderer = SimFrameGenerator(str(tmp_path), (640, 480))
    frame, boxes = renderer.generate_frame([(0.25, 0.5, 0.5)])
    assert boxes == [(160, 240, 40, 30)]
    assert np.all(frame[225:255, 140:180] == 255)


def test_vehicle_uses_builtin_artwork_and_preserves_canvas_geometry(tmp_path):
    assert cv2.imwrite(str(tmp_path / "background.png"), np.zeros((480, 640, 3), np.uint8))
    # An old external illustration must not replace the built-in delivery vehicle.
    assert cv2.imwrite(str(tmp_path / "vehicle.png"), np.full((30, 40, 4), 255, np.uint8))
    renderer = SimFrameGenerator(str(tmp_path), (640, 480))
    assert (renderer.sprite_width_for("vehicle"), renderer.sprite_height_for("vehicle")) == (600, 600)
    frame, boxes = renderer.generate_frame([(0.5, 0.5, 0.25)], location_type="vehicle")
    assert boxes == [(320, 240, 150, 150)]
    assert np.any(frame[165:315, 245:395])
    assert not np.any(frame[:165])


def test_explicit_delivery_vehicle_artwork_override(tmp_path):
    assert cv2.imwrite(str(tmp_path / "background.png"), np.zeros((480, 640, 3), np.uint8))
    assert cv2.imwrite(str(tmp_path / "delivery_vehicle.png"), np.full((60, 80, 4), 255, np.uint8))
    renderer = SimFrameGenerator(str(tmp_path), (640, 480))
    frame, boxes = renderer.generate_frame([(0.25, 0.5, 0.5)], location_type="vehicle")
    assert boxes == [(160, 240, 40, 30)]
    assert np.all(frame[225:255, 140:180] == 255)
