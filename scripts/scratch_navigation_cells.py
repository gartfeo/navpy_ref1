"""What one aircraft is asked to fly: its condition, timing and noise.

Separate from the arm concern because an arm is a different CODE variant while a
cell is a different QUESTION put to the same code. They are crossed rather than
merged -- see `scratch_navigation_arms.assign_cases`.
"""

from __future__ import annotations

from dataclasses import dataclass

# Validated HERE, at parse time, rather than in the child: a misspelt mode
# reaching the child would fall through to the untreated branch, and the launch
# would report a clean null for a treatment that never ran.
from scripts.scratch_navigation_oracle import MODES as ORACLE_MODES


@dataclass(frozen=True)
class Cell:
    """One aircraft's condition inside a launch.

    Timing and noise belong here beside wind because they are the questions the
    bench exists to answer, and answering them one launch per arm is what made
    the earlier sweeps unusable: an 8x timing result flown one arm per launch
    cannot be separated from a 5.5x batch spread. Per aircraft, they are paired.
    """

    name: str
    wind_speed_mps: float | None
    wind_dir_deg: float
    # None means "leave the launch-wide value alone", which is not the same as
    # zero: zero is an explicit arm of the sweep.
    delay_poses: int | None = None
    jitter_deg: float | None = None
    lag_s: float | None = None
    # FIDELITY per aircraft, so the same timing lag can be applied to a source
    # whose ray and attitude are ONE message. That is the only way one launch can
    # separate a pairing fix from a loop-phase effect: a pairing correction
    # cannot help a source that has no pairing error to correct.
    source: str | None = None
    # `angles` (default) lags only the de-rotating pitch/roll; `full` lags the
    # body rates by the same interval, putting every law input on one clock.
    lag_scope: str | None = None
    # GEOMETRY and ENERGY per aircraft, added last so the positional grammar
    # above keeps its meaning. Both were launch-wide flags, which is what made
    # an angle sweep cost one launch per angle and forced the mirror of a
    # +30 matrix into a separate run -- and two launches are the one thing
    # this bench has repeatedly proven it cannot compare.
    off_boresight_deg: float | None = None
    throttle: float | None = None
    # TRUTH-FED DIAGNOSTIC. `scratch_navigation_oracle` states what it violates
    # and why it is worth violating once. A CELL rather than an ARM because the
    # treatment needs no code variant -- it is a different NUMBER handed to the
    # same law -- and a cell keeps treated and untreated inside ONE launch,
    # which the 5.5x batch spread makes mandatory rather than merely tidy.
    oracle: str | None = None


def parse_cells(spec: str | None) -> tuple[Cell, ...]:
    """Parse 'name:spd:dir[:delay[:jit[:lag[:src[:scope[:off[:thr[:oracle]]]]]]]]'.

    Positional and long, which is ugly, but every field is a knob this bench
    already had -- naming them would be a second syntax to keep in step with the
    child's own options.
    """
    if not spec:
        return ()
    cells = []
    for field in spec.split(","):
        parts = field.split(":")
        if not 3 <= len(parts) <= 11:
            raise ValueError(
                f"cell {field!r} must be name:wind_speed:wind_dir"
                "[:delay_poses[:jitter_deg[:lag_s[:source[:lag_scope"
                "[:off_boresight_deg[:throttle[:oracle]]]]]]]]"
            )
        name, speed, direction = parts[0], parts[1], parts[2]
        if not name:
            raise ValueError(f"cell {field!r} needs a name")
        delay = int(parts[3]) if len(parts) > 3 and parts[3] != "" else None
        jitter = float(parts[4]) if len(parts) > 4 and parts[4] != "" else None
        lag = float(parts[5]) if len(parts) > 5 and parts[5] != "" else None
        source = parts[6] if len(parts) > 6 and parts[6] != "" else None
        scope = parts[7] if len(parts) > 7 and parts[7] != "" else None
        off = float(parts[8]) if len(parts) > 8 and parts[8] != "" else None
        throttle = float(parts[9]) if len(parts) > 9 and parts[9] != "" else None
        oracle = parts[10] if len(parts) > 10 and parts[10] != "" else None
        if oracle is not None and oracle not in ORACLE_MODES:
            raise ValueError(
                f"cell {field!r} oracle must be one of {ORACLE_MODES}"
            )
        cells.append(
            Cell(name, float(speed), float(direction), delay, jitter, lag,
                 source, scope, off, throttle, oracle)
        )
    return tuple(cells)
