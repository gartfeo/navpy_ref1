"""Non-accuracy validity gates: is a direct-pixel run evaluable at all?

Everything here is orthogonal to which source scores the run: the child must
have completed its pass, the sensor must have delivered fresh frames, the
commands must be stable and causally explained, and the evaluator's own
diagnostic stream must be certified.  Accuracy policy lives in
``eval_direct_pixel_verdict``.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Do it here rather than relying on another script
# having been imported first: without this the module (and its test) fails
# standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from scripts.eval_direct_pixel_command_causality import causality_evidence  # noqa: E402
from eval_direct_pixel_stability import (  # noqa: E402
    MAX_MIDCOURSE_PITCH_STEP_DEG, _command_stability,
)
from eval_observation_freshness import (  # noqa: E402
    freshness_errors,
    observation_freshness as _observation_freshness,
)


def validity_errors(
    case_dir: Path,
    *,
    child_result: dict[str, Any],
    scorer: Any,
    track: Any,
    wind_speed: float,
    wind_dir_deg: float,
    speedup: float,
) -> tuple[list[str], dict[str, Any]]:
    """Non-accuracy gates plus the evidence blocks they were computed from."""
    errors: list[str] = []
    # Wind relative to where the aircraft actually went, not to an assumed
    # 000 track: a run is only "tailwind" if the measurement says so.
    ground_track = None
    try:
        track.write(case_dir / "ground_track.csv")
        ground_track = track.summary(wind_speed, wind_dir_deg)
    except (OSError, RuntimeError, ValueError) as error:
        errors.append(f"ground track unavailable: {error}")
    if child_result.get("passed") is not True:
        errors.append(f"POI pass false: {child_result.get('error', '')}")
    source = child_result.get("source")
    if not isinstance(source, dict):
        errors.append("direct pixel source metrics missing")
    else:
        if source.get("projection_failures") != 0:
            errors.append(
                "POI left renderable sight: "
                f"{source.get('projection_failures')} projection failures"
            )
        if not isinstance(source.get("delivered_frames"), int) or (
            source["delivered_frames"] < 10
        ):
            errors.append(
                "insufficient direct pixel deliveries: "
                f"{source.get('delivered_frames')!r}"
            )
    if scorer.certification_error:
        errors.append(scorer.certification_error)
    stability = _command_stability(case_dir)
    freshness = None
    try:
        freshness = _observation_freshness(
            case_dir,
            speedup,
            source if isinstance(source, dict) else {},
        )
    except (OSError, RuntimeError, ValueError) as error:
        errors.append(f"observation freshness unavailable: {error}")
    if freshness is not None:
        errors.extend(freshness_errors(freshness))
    debug_paths = list((case_dir / "navpy-logs").glob("*_navigation_debug.csv"))
    if len(debug_paths) != 1:
        raise RuntimeError(
            f"expected one debug navigation log, found {len(debug_paths)}"
        )
    causality = causality_evidence(debug_paths[0], errors)
    if stability["pitch_max_step_deg"] > MAX_MIDCOURSE_PITCH_STEP_DEG:
        errors.append(
            "midcourse pitch command discontinuity: "
            f"{stability['pitch_max_step_deg']:.3f}deg exceeds "
            f"{MAX_MIDCOURSE_PITCH_STEP_DEG:.3f}deg"
        )
    evidence = {
        "ground_track": ground_track,
        "command_stability": stability,
        "observation_freshness": freshness,
        "command_causality": causality,
    }
    return errors, evidence


__all__ = ["validity_errors"]
