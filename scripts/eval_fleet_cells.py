"""Which wind each aircraft in a fleet flies, and in what order.

A fleet launch flies many conditions at once, so something has to decide which
aircraft gets which condition. That decision is an experimental design choice,
not bookkeeping, which is why it lives on its own and is tested on its own.
"""

from __future__ import annotations

import random

DEFAULT_CELLS = "0@0,8@0,8@90,8@180,8@270,12@0,12@90,12@180,12@270"


def parse_cells(raw: str) -> list[tuple[float, float]]:
    """``"8@90"`` is 8 m/s blowing FROM 090, the SIM_WIND_DIR convention."""
    cells: list[tuple[float, float]] = []
    for chunk in raw.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        speed, _, direction = chunk.partition("@")
        cells.append((float(speed), float(direction or 0.0)))
    if not cells:
        raise ValueError("no wind cells given")
    return cells


def assign(
    cells: list[tuple[float, float]], reps: int, *, seed: int
) -> list[tuple[float, float]]:
    """A randomised complete block design: each replicate is its own shuffle.

    Every cell appears exactly `reps` times, so the matrix stays balanced, but
    no cell keeps a fixed launch position across replicates. That second part
    is the whole point. Sysid tracks launch order, and launch order has carried
    real effects here before -- the aircraft that needed a MISSION_START retry
    were exactly the lowest sysids in their wave. A plain round-robin looks
    like it handles that and does not: it puts cell `i` at positions
    `i, i+len, i+2*len ...` in every run, so each cell holds a constant mean
    launch position and anything periodic in the fleet width aliases onto cell
    index exactly.

    Seeded, so a matrix is randomised but still re-flyable.
    """
    generator = random.Random(seed)
    winds: list[tuple[float, float]] = []
    for _ in range(reps):
        block = list(cells)
        generator.shuffle(block)
        winds.extend(block)
    return winds


__all__ = ["DEFAULT_CELLS", "assign", "parse_cells"]
