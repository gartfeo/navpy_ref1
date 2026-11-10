"""The profile's ``detector.rfdetr`` block (HITL step 23b; docs/design/hitl_dock_detector.md section 3).

The rfdetr backend runs only with this block; nothing is implied::

    "rfdetr": {
        "variant": "large",          # RF-DETR-L: its 704 px square input
        "classes": {"truck": "dock"},  # phase A: COCO truck only, mapped to the dock class
        "input": "tiles"             # "squash" (the whole frame to one square) or "tiles" (native 704 px tiles)
    }

``classes`` names exactly one COCO class: one detector, one class, no fallback chain. The detector's minimum
confidence is the profile's ``detector.conf``. A malformed block is refused when the detector
is built.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from navpy.plugins.detection.rfdetr.rfdetr_codec import COCO80_NAMES

RFDETR_KEY = "rfdetr"
# The rfdetr variants' square input (rfdetr 1.12.0 config ``resolution``) and decode (``num_select``).
VARIANTS = {"large": 704}
NUM_SELECT = 300
INPUT_SQUASH = "squash"
INPUT_TILES = "tiles"
INPUT_MODES = (INPUT_SQUASH, INPUT_TILES)


@dataclass(frozen=True)
class RfDetrSettings:
    variant: str
    input_px: int
    # COCO 0-79 index -> navpy class id; every other class is dropped.
    class_map: tuple[tuple[int, int], ...]
    input_mode: str
    num_select: int = NUM_SELECT

    def navpy_class(self, coco_index: int) -> int | None:
        return dict(self.class_map).get(int(coco_index))

    def input_scale(self, width: int, height: int) -> tuple[float, float]:
        """The model's input pixels per full-frame pixel, across and down: the squash's (size / axis), or a tile's
        (size / the tile's axis, 1 when the frame is at least a tile long)."""
        if self.input_mode == INPUT_TILES:
            return self.input_px / min(self.input_px, width), self.input_px / min(self.input_px, height)
        return self.input_px / width, self.input_px / height


def rfdetr_settings(detector: Mapping[str, object], navpy_classes: Mapping[str, int]) -> RfDetrSettings:
    """The block from the profile's detector settings; ValueError when it is missing or malformed. navpy_classes:
    the navpy class names a COCO class may map to, with their ids (the app passes only the dock)."""
    block = detector.get(RFDETR_KEY) if isinstance(detector, Mapping) else None
    if not isinstance(block, Mapping):
        raise ValueError(f"the rfdetr backend needs the profile's detector.{RFDETR_KEY} block")
    unknown = set(block) - {"variant", "classes", "input"}
    if unknown:
        raise ValueError(f"detector.{RFDETR_KEY}: unknown {sorted(unknown)}")
    variant = block.get("variant")
    if variant not in VARIANTS:
        raise ValueError(f"detector.{RFDETR_KEY}.variant must be one of {sorted(VARIANTS)}")
    mode = block.get("input")
    if mode not in INPUT_MODES:
        raise ValueError(f"detector.{RFDETR_KEY}.input must be one of {INPUT_MODES}")
    classes = block.get("classes")
    if not isinstance(classes, Mapping) or len(classes) != 1:
        raise ValueError(f"detector.{RFDETR_KEY}.classes must map exactly one COCO class to a navpy class")
    pairs = []
    for coco, navpy in classes.items():
        if coco not in COCO80_NAMES or navpy not in navpy_classes:
            raise ValueError(f"detector.{RFDETR_KEY}.classes: {coco!r} -> {navpy!r} is not a known COCO class "
                             f"({sorted(COCO80_NAMES)}) mapped to {sorted(navpy_classes)}")
        pairs.append((COCO80_NAMES[coco], int(navpy_classes[navpy])))
    return RfDetrSettings(str(variant), VARIANTS[str(variant)], tuple(sorted(pairs)), str(mode))


__all__ = ["INPUT_MODES", "INPUT_SQUASH", "INPUT_TILES", "NUM_SELECT", "RFDETR_KEY", "RfDetrSettings", "VARIANTS",
           "rfdetr_settings"]
