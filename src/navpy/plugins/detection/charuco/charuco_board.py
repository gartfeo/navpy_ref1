"""ChArUco reference-board geometry for the default real detector backend.

The defaults describe the A3 bench board: ``DICT_4X4_50``, 5 x 7 squares,
50 mm squares carrying 37 mm ArUco markers. A vision profile may override
any field through an optional ``detector.charuco`` block::

    "detector": {"charuco": {"squares_x": 5, "squares_y": 7,
                             "square_mm": 50.0, "marker_mm": 37.0,
                             "dictionary": "DICT_4X4_50"}}

The board is the dock reference object, so every detection it produces is
reported as the single dock detector class.

How the markers are found is a separate, optional ``detector.charuco_detection``
block (``CharucoDetectionSettings``); its defaults are the long-range settings
measured in docs/design/hitl_unreal_camera.md "Detector settings for range".
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

# A3 bench board (print at 100 % scale; squares must measure 50 mm).
CHARUCO_DICTIONARY_NAME = "DICT_4X4_50"
CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7
CHARUCO_SQUARE_MM = 50.0
CHARUCO_MARKER_MM = 37.0

# The board stands in for the dock; the dock is detector class 0.
CHARUCO_DOCK_CLASS_ID = 0

# Profile key holding optional board overrides inside the detector block.
CHARUCO_SETTINGS_KEY = "charuco"

_SETTINGS_FIELDS = frozenset(
    {"dictionary", "squares_x", "squares_y", "square_mm", "marker_mm"}
)


# Profile key holding optional marker-detection overrides inside the detector block.
CHARUCO_DETECTION_SETTINGS_KEY = "charuco_detection"

# Long-range marker detection (hitl_unreal_camera.md "Detector settings for range", 2026-10-14). OpenCV's
# default drops a marker whose outline is under 3 % of the frame's larger side (38 px at 1280): the dock
# board's markers at 35-40 m on the ZR10 at zoom 1. 1.5 % keeps them; smaller markers cannot be read anyway.
CHARUCO_MIN_MARKER_PERIMETER_RATE = 0.015
# Pixels per marker bit cell when a marker is unwarped for reading (OpenCV default 4): 8 reads small, soft
# markers more reliably.
CHARUCO_PIXELS_PER_CELL = 8
# Search again for the board's missing markers where the found ones place them (refineDetectedMarkers): the
# largest gain on blurred and H.264/H.265-compressed frames, with no wrong board in the sweep.
CHARUCO_REFINE_MARKERS = True

_DETECTION_FIELDS = frozenset(
    {"min_marker_perimeter_rate", "pixels_per_cell", "refine_markers"}
)


@dataclass(frozen=True)
class CharucoDetectionSettings:
    """How the ChArUco detector finds the board's markers in a frame."""

    min_marker_perimeter_rate: float = CHARUCO_MIN_MARKER_PERIMETER_RATE
    pixels_per_cell: int = CHARUCO_PIXELS_PER_CELL
    refine_markers: bool = CHARUCO_REFINE_MARKERS

    def __post_init__(self) -> None:
        if not 0.0 < self.min_marker_perimeter_rate < 1.0:
            raise ValueError(
                "ChArUco min_marker_perimeter_rate must be in (0, 1)"
            )
        if self.pixels_per_cell < 1:
            raise ValueError("ChArUco pixels_per_cell must be at least 1")


@dataclass(frozen=True)
class CharucoBoardSpec:
    dictionary: str = CHARUCO_DICTIONARY_NAME
    squares_x: int = CHARUCO_SQUARES_X
    squares_y: int = CHARUCO_SQUARES_Y
    square_mm: float = CHARUCO_SQUARE_MM
    marker_mm: float = CHARUCO_MARKER_MM

    def __post_init__(self) -> None:
        if self.squares_x < 2 or self.squares_y < 2:
            raise ValueError("ChArUco board needs at least 2 x 2 squares")
        if not 0.0 < self.marker_mm < self.square_mm:
            raise ValueError(
                "ChArUco marker_mm must be positive and smaller than square_mm"
            )

    @property
    def interior_corner_count(self) -> int:
        """Number of ChArUco (chessboard interior) corners on the board."""
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def size_mm(self) -> tuple[float, float]:
        """Board outline (width, height) in millimetres."""
        return (
            self.squares_x * self.square_mm,
            self.squares_y * self.square_mm,
        )


def charuco_board_spec_from_settings(
    detector_settings: Mapping[str, object],
) -> CharucoBoardSpec:
    """Build the board spec from an optional profile ``charuco`` block."""
    block = detector_settings.get(CHARUCO_SETTINGS_KEY)
    if block is None:
        return CharucoBoardSpec()
    if not isinstance(block, Mapping):
        raise ValueError("Vision profile detector.charuco must be a mapping")
    unknown = set(block) - _SETTINGS_FIELDS
    if unknown:
        raise ValueError(
            f"Unknown detector.charuco fields: {sorted(unknown)}"
        )
    defaults = CharucoBoardSpec()
    return CharucoBoardSpec(
        dictionary=str(block.get("dictionary", defaults.dictionary)),
        squares_x=int(block.get("squares_x", defaults.squares_x)),
        squares_y=int(block.get("squares_y", defaults.squares_y)),
        square_mm=float(block.get("square_mm", defaults.square_mm)),
        marker_mm=float(block.get("marker_mm", defaults.marker_mm)),
    )


def charuco_detection_settings_from_settings(
    detector_settings: Mapping[str, object],
) -> CharucoDetectionSettings:
    """Build the detection settings from an optional profile block."""
    block = detector_settings.get(CHARUCO_DETECTION_SETTINGS_KEY)
    if block is None:
        return CharucoDetectionSettings()
    if not isinstance(block, Mapping):
        raise ValueError(
            "Vision profile detector.charuco_detection must be a mapping"
        )
    unknown = set(block) - _DETECTION_FIELDS
    if unknown:
        raise ValueError(
            f"Unknown detector.charuco_detection fields: {sorted(unknown)}"
        )
    refine = block.get("refine_markers", CHARUCO_REFINE_MARKERS)
    if not isinstance(refine, bool):
        raise ValueError(
            "detector.charuco_detection.refine_markers must be true or false"
        )
    cells = block.get("pixels_per_cell", CHARUCO_PIXELS_PER_CELL)
    if isinstance(cells, bool) or not isinstance(cells, int):
        raise ValueError(
            "detector.charuco_detection.pixels_per_cell must be an integer"
        )
    rate = block.get(
        "min_marker_perimeter_rate", CHARUCO_MIN_MARKER_PERIMETER_RATE,
    )
    if isinstance(rate, bool) or not isinstance(rate, (int, float)):
        raise ValueError(
            "detector.charuco_detection.min_marker_perimeter_rate must be a number"
        )
    return CharucoDetectionSettings(
        min_marker_perimeter_rate=float(rate),
        pixels_per_cell=cells,
        refine_markers=refine,
    )


__all__ = [
    "CHARUCO_DETECTION_SETTINGS_KEY",
    "CHARUCO_MIN_MARKER_PERIMETER_RATE",
    "CHARUCO_PIXELS_PER_CELL",
    "CHARUCO_REFINE_MARKERS",
    "CharucoDetectionSettings",
    "charuco_detection_settings_from_settings",
    "CHARUCO_DICTIONARY_NAME",
    "CHARUCO_DOCK_CLASS_ID",
    "CHARUCO_MARKER_MM",
    "CHARUCO_SETTINGS_KEY",
    "CHARUCO_SQUARES_X",
    "CHARUCO_SQUARES_Y",
    "CHARUCO_SQUARE_MM",
    "CharucoBoardSpec",
    "charuco_board_spec_from_settings",
]
