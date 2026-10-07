"""Verify every logged direct-pixel command from its visual law terms.

The point is to prove each command is fully explained by frame-local visual
terms plus the previous command -- that no POI truth, range, ground speed or
compass heading leaked into the command path. A residual here means the command
came from somewhere the log does not account for.

That only works if the model matches the law. It stopped matching, silently,
and every run since carried a false failure:

* the law integrates from the previous COMMAND (`anchor + increment`), while
  this modelled pitch as an offset from measured aircraft pitch through the
  autopilot's pitch time constant
* the law's roll is `atan(N*V*rate/g)` with no bearing term; this still added
  `sin(bearing)`, a proportional term deliberately removed from the law
* N was 3.0 here and 4.0 in the law
* four separate branches reissue the previous command -- bootstrap, regressed
  timestamp, outlier, and a roll-only hold -- and none were modelled
* commands are clipped to the configured limits narrowed by the law's own
  structural caps (35 deg roll, [-70, 3] pitch); this clipped to the configured
  limits alone, which by itself produced a standing 10.0 deg residual against a
  45 deg configured roll limit

The evidence payload now carries the branch, the anchor, the interval, the
effective limits and the gains actually applied, so the model is reconstructed
from what ran rather than from a second copy of the constants that can rot.
"""

from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass
from pathlib import Path


EVENT = "EVENT:FINAL_APPROACH_RESPONSE_STATE"
GRAVITY_MSS = 9.80665
TOLERANCE_DEG = 1e-6

# Branches that reissue the anchor's command instead of integrating. `held` is
# the helper's default and is kept so an unlabelled hold is still read as a
# hold rather than scored as a broken integration.
BOTH_AXES_HELD = frozenset({"bootstrap", "dt_regressed", "outlier", "held"})
# Roll reissues its anchor, pitch integrates normally.
ROLL_ONLY_HELD = "lateral_held"

_REQUIRED = (
    "obs_ts", "control_bearing_deg", "lateral_rate_deg_s",
    "aircraft_turn_rate_deg_s", "inertial_los_rate_deg_s",
    "raw_inertial_los_rate_deg_s",
    "aircraft_roll_deg", "aircraft_pitch_deg", "air_speed_mps",
    "control_elevation_deg", "vertical_rate_deg_s",
    "effective_roll_limit_deg", "effective_pitch_min_deg",
    "effective_pitch_max_deg",
    "lateral_nav_constant", "vertical_nav_constant",
    "raw_roll_deg", "raw_pitch_deg", "cmd_roll_deg", "cmd_pitch_deg",
)
# Absent on the held branches, which never compute an increment.
_OPTIONAL = ("anchor_cmd_roll_deg", "anchor_cmd_pitch_deg", "dt_s")


@dataclass(frozen=True)
class CommandCausality:
    sample_count: int
    roll_max_residual_deg: float
    pitch_max_residual_deg: float
    roll_max_step_deg: float
    pitch_max_step_deg: float
    held_sample_count: int


def analyze(path: Path) -> CommandCausality:
    samples = _samples(path)
    if len(samples) < 10:
        raise ValueError(f"insufficient command-causality samples: {len(samples)}")
    roll_residuals: list[float] = []
    pitch_residuals: list[float] = []
    held = 0
    for sample in samples:
        reason = sample["plan_reason"]
        if reason in BOTH_AXES_HELD or reason == ROLL_ONLY_HELD:
            held += 1
        roll_residuals.append(abs(sample["raw_roll_deg"] - _expected_raw_roll(sample)))
        pitch_residuals.append(
            abs(sample["raw_pitch_deg"] - _expected_raw_pitch(sample))
        )
        # The law's own turn-rate identity: the inertial LOS rate is the visual
        # rate plus the aircraft's turn rate, and nothing else.
        roll_residuals.append(abs(
            sample["raw_inertial_los_rate_deg_s"]
            - (sample["lateral_rate_deg_s"] + sample["aircraft_turn_rate_deg_s"])
        ))
        roll_residuals.append(abs(sample["cmd_roll_deg"] - _clip(
            sample["raw_roll_deg"],
            -sample["effective_roll_limit_deg"],
            sample["effective_roll_limit_deg"],
        )))
        pitch_residuals.append(abs(sample["cmd_pitch_deg"] - _clip(
            sample["raw_pitch_deg"],
            sample["effective_pitch_min_deg"],
            sample["effective_pitch_max_deg"],
        )))
    result = CommandCausality(
        sample_count=len(samples),
        roll_max_residual_deg=max(roll_residuals),
        pitch_max_residual_deg=max(pitch_residuals),
        roll_max_step_deg=_max_step(samples, "cmd_roll_deg"),
        pitch_max_step_deg=_max_step(samples, "cmd_pitch_deg"),
        held_sample_count=held,
    )
    if result.roll_max_residual_deg > TOLERANCE_DEG:
        raise ValueError(f"unexplained roll command residual: {result}")
    if result.pitch_max_residual_deg > TOLERANCE_DEG:
        raise ValueError(f"unexplained pitch command residual: {result}")
    return result


def _expected_raw_roll(sample: dict) -> float:
    """Roll from the law: a held anchor, or pure PN on the inertial LOS rate.

    No bearing term. A steady non-zero bearing IS the converged geometry in
    wind, so rolling to null it would leave the constant-bearing course; the law drops
    that term deliberately.
    """
    if sample["plan_reason"] in BOTH_AXES_HELD or (
        sample["plan_reason"] == ROLL_ONLY_HELD
    ):
        return _anchor(sample, "anchor_cmd_roll_deg")
    return math.degrees(math.atan(
        sample["lateral_nav_constant"]
        * sample["air_speed_mps"]
        * math.radians(sample["inertial_los_rate_deg_s"])
        / GRAVITY_MSS
    ))


def _expected_raw_pitch(sample: dict) -> float:
    """Pitch from the law: a held anchor, or the anchor plus one increment.

    The command integrates from the PREVIOUS COMMAND, not from measured
    aircraft pitch. The clamped command becomes the next anchor, which is what
    keeps a saturated cycle from winding the integrator up.
    """
    if sample["plan_reason"] in BOTH_AXES_HELD:
        return _anchor(sample, "anchor_cmd_pitch_deg")
    return _anchor(sample, "anchor_cmd_pitch_deg") - math.degrees(
        sample["vertical_nav_constant"]
        * math.radians(sample["vertical_rate_deg_s"])
        * _anchor(sample, "dt_s")
    )


def _anchor(sample: dict, name: str) -> float:
    value = sample.get(name)
    if value is None:
        raise ValueError(f"{name} missing on a {sample['plan_reason']} sample")
    return value


def causality_evidence(
    path: Path, errors: list[str]
) -> dict[str, object]:
    """`analyze`, reported into `errors` instead of raised.

    The caller has already computed the coordinate CPA, the stability scan and
    the freshness check by the time it audits. Letting `analyze` raise sent all
    of them to the harness's broad handler, which returns the audit message
    ALONE and never writes verdict.json -- so a truth-scored miss that HAD been
    computed was discarded, and the run read as though it never produced a
    score. Seen on direct-pixel-pn-20260818-145026: a completed scoring interval with
    no verdict.json in its case directory.

    The audit still fails the case, through the same `errors` list every other
    check uses. It just no longer destroys the evidence gathered before it.
    """
    try:
        return asdict(analyze(path))
    except (OSError, ValueError) as error:
        errors.append(f"command causality: {error}")
        return {"error": str(error)}


def _samples(path: Path) -> list[dict]:
    samples: list[dict] = []
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 3 or row[1] != EVENT:
                continue
            payload = _payload(row[2])
            sample: dict = {name: _number(payload, name) for name in _REQUIRED}
            sample["plan_reason"] = _reason(payload)
            for name in _OPTIONAL:
                sample[name] = _optional_number(payload, name)
            samples.append(sample)
    return samples


def _payload(value: str) -> dict[str, str]:
    return dict(token.split("=", 1) for token in value.split(";") if "=" in token)


def _reason(payload: dict[str, str]) -> str:
    # An unlabelled row predates the branch label. It cannot be audited, since
    # a hold and an integration are indistinguishable without it, and guessing
    # would turn a stale log into a silent pass.
    if "plan_reason" not in payload or not payload["plan_reason"]:
        raise ValueError("missing plan_reason: log predates branch labelling")
    return payload["plan_reason"]


def _number(payload: dict[str, str], name: str) -> float:
    # A row tagged with the event but short a field is BAD DATA -- a truncated
    # write, an older log format -- so it has to surface as ValueError like
    # every other malformed-row case. Indexing raised KeyError, which is
    # neither OSError nor ValueError, so it escaped `causality_evidence` and
    # took verdict.json down with it: exactly the failure that function exists
    # to prevent.
    if name not in payload:
        raise ValueError(f"missing {name}")
    value = float(payload[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def _optional_number(payload: dict[str, str], name: str) -> float | None:
    """A field the held branches legitimately leave empty."""
    raw = payload.get(name, "")
    if raw in ("", "None"):
        return None
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _max_step(samples: list[dict], name: str) -> float:
    return max(
        abs(right[name] - left[name])
        for left, right in zip(samples, samples[1:])
    )
