"""ChArUco reference-board geometry for the default real detector backend.

The defaults describe the A3 bench board: ``DICT_4X4_50``, 5 x 7 squares,
50 mm squares carrying 37 mm ArUco markers. A vision profile may override
any field through an optional ``detector.charuco`` block::

    "detector": {"charuco": {"squares_x": 5, "squares_y": 7,
                             "square_mm": 50.0, "marker_mm": 37.0,
                             "dictionary": "DICT_4X4_50"}}

The board is the dock reference object, so every detection it produces is
reported as the single dock detector class.
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


__all__ = [
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
