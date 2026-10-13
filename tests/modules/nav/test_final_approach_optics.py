"""Current-optics reader for the a7 completion check."""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from navpy.app.flow.final_approach_optics import mount_half_fov_reader
from navpy.core.approach.vision_nav.visual_pass import PASS_MAX_ZOOM_OVER_WIDEST


class _Mount:
    def __init__(
        self, name: str, fx: float, size=(1280, 720), zoom: str | None = "1.00"
    ) -> None:
        self.name = name
        self.fx = fx
        self.image_width, self.image_height = size
        self.zoom = zoom
        self.levels = ["1", "2", "3", "10"]

    def get_gimbal_data(self) -> SimpleNamespace:
        return SimpleNamespace(name=self.name)

    def get_k_for_frame(self, frame_w: int, frame_h: int) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, frame_w / 2.0], [0.0, self.fx, frame_h / 2.0], [0.0, 0.0, 1.0]]
        )

    def get_zoom_levels(self) -> list[str]:
        return self.levels

    def get_fresh_zoom_command(self) -> str | None:
        """Mirrors CameraMount: None when the readback is stale or missing."""
        return self.zoom


def test_reader_returns_the_named_camera_short_half_fov() -> None:
    mounts = [_Mount("other", 500.0), _Mount("gimbal_0", 1407.42)]
    read = mount_half_fov_reader(lambda: mounts)

    assert math.isclose(
        read("gimbal_0"), math.degrees(math.atan(360.0 / 1407.42))
    )


def test_reader_sees_an_intrinsics_change_at_the_widest_zoom() -> None:
    mount = _Mount("gimbal_0", 1407.42)
    read = mount_half_fov_reader(lambda: [mount])
    before = read("gimbal_0")
    mount.fx = 1500.0

    assert read("gimbal_0") < before


def test_widest_zoom_guard_constant() -> None:
    assert PASS_MAX_ZOOM_OVER_WIDEST == 1.2


@pytest.mark.parametrize("zoom", ["1", "1.00", "1.10", "1.20"])
def test_fully_zoomed_out_camera_allows_the_centred_loss_check(zoom: str) -> None:
    mount = _Mount("gimbal_0", 1407.42, zoom=zoom)

    assert mount_half_fov_reader(lambda: [mount])("gimbal_0") is not None


@pytest.mark.parametrize("zoom", ["1.30", "2.00", "6.50", "10.00"])
def test_zoomed_in_camera_never_allows_a_centred_loss_pass(zoom: str) -> None:
    """Finding 1 guard: a far dock lost centred while zoomed in is no pass."""
    mount = _Mount("gimbal_0", 1407.42, zoom=zoom)

    assert mount_half_fov_reader(lambda: [mount])("gimbal_0") is None


def test_stale_or_missing_zoom_readback_fails_closed() -> None:
    stale = _Mount("gimbal_0", 1407.42, zoom=None)

    assert mount_half_fov_reader(lambda: [stale])("gimbal_0") is None


def test_two_level_zoom_camera_is_still_guarded() -> None:
    """A 2x camera has zoom control even though CameraMount.has_zoom is
    False for it (has_zoom means a focal span of at least 2.5)."""
    two_x = _Mount("gimbal_0", 1407.42, zoom="2.00")
    two_x.levels = ["1", "2"]

    assert mount_half_fov_reader(lambda: [two_x])("gimbal_0") is None


@pytest.mark.parametrize("levels", [["1"], []])
def test_camera_without_zoom_control_is_always_at_its_widest(levels) -> None:
    fixed = _Mount("gimbal_0", 1407.42, zoom=None)
    fixed.levels = levels

    assert mount_half_fov_reader(lambda: [fixed])("gimbal_0") is not None


def test_unknown_camera_or_optics_fail_closed() -> None:
    broken = _Mount("gimbal_0", 1407.42)
    broken.get_k_for_frame = None  # type: ignore[assignment]
    unsized = _Mount("gimbal_0", 1407.42, size=(None, None))

    assert mount_half_fov_reader(lambda: [])("gimbal_0") is None
    assert mount_half_fov_reader(lambda: [broken])("gimbal_0") is None
    assert mount_half_fov_reader(lambda: [unsized])("gimbal_0") is None
