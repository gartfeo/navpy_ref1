from __future__ import annotations

from pathlib import Path
import math

import pytest

from navpy.modules.navigation.nav.vision_nav.command_anchor import FinalApproachLimits
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    FinalApproachLawEvidence,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_capture import (
    law_evidence_payload,
)
from navpy.modules.navigation.nav.vision_nav.law_plan import FinalApproachPlanOrigin
from scripts.eval_direct_pixel_command_causality import (
    _OPTIONAL,
    _REQUIRED,
    analyze,
    causality_evidence,
)


N_LATERAL = 4.0
N_VERTICAL = 4.0
DT_S = 0.1
EFFECTIVE_ROLL_LIMIT = 35.0
EFFECTIVE_PITCH_MIN = -55.0
EFFECTIVE_PITCH_MAX = 3.0


def _row(
    index: int,
    *,
    reason: str = "normal",
    bad_roll: bool = False,
    omit: str = "",
    lateral_rate: float = 0.2,
) -> str:
    """One response row built from the CURRENT law, not the retired one.

    Roll is pure PN on the inertial LOS rate -- no bearing term, which the law
    dropped deliberately. Pitch integrates from the previous COMMAND over dt,
    not from measured aircraft pitch through the autopilot time constant.
    """
    bearing = 2.0 + index * 0.01
    elevation = 6.0 + index * 0.02
    rate = 0.1
    aircraft_roll = 5.0
    aircraft_pitch = -8.0
    airspeed = 25.0
    anchor_roll = 1.5
    anchor_pitch = -7.0
    turn_rate = math.degrees(
        9.80665 * math.tan(math.radians(aircraft_roll)) / airspeed
    )
    inertial_rate = lateral_rate + turn_rate

    if reason == "normal":
        raw_roll = math.degrees(math.atan(
            N_LATERAL * airspeed * math.radians(inertial_rate) / 9.80665))
        raw_pitch = anchor_pitch - math.degrees(
            N_VERTICAL * math.radians(rate) * DT_S)
    elif reason == "lateral_held":
        raw_roll = anchor_roll
        raw_pitch = anchor_pitch - math.degrees(
            N_VERTICAL * math.radians(rate) * DT_S)
    else:  # bootstrap / dt_regressed / outlier -- both axes reissue the anchor
        raw_roll = anchor_roll
        raw_pitch = anchor_pitch
    raw_roll += 1.0 if bad_roll and index == 5 else 0.0
    # Clip exactly as the law does -- at the EFFECTIVE limits, which are the
    # configured ones narrowed by the law's structural caps. A fixture that
    # clipped anywhere else would be testing the wrong contract.
    cmd_roll = min(max(raw_roll, -EFFECTIVE_ROLL_LIMIT), EFFECTIVE_ROLL_LIMIT)
    raw_pitch_cmd = min(
        max(raw_pitch, EFFECTIVE_PITCH_MIN), EFFECTIVE_PITCH_MAX)

    fields = [
        "source=direct_poi_pixel", f"obs_ts={index * DT_S}",
        f"control_bearing_deg={bearing}",
        f"lateral_rate_deg_s={lateral_rate}",
        f"aircraft_turn_rate_deg_s={turn_rate}",
        f"raw_inertial_los_rate_deg_s={inertial_rate}",
        f"inertial_los_rate_deg_s={inertial_rate}",
        f"aircraft_roll_deg={aircraft_roll}", f"air_speed_mps={airspeed}",
        f"aircraft_pitch_deg={aircraft_pitch}",
        f"control_elevation_deg={elevation}",
        f"vertical_rate_deg_s={rate}", "pitch_time_constant_s=0.5",
        "roll_limit_deg=45.0", "pitch_min_deg=-55.0", "pitch_max_deg=25.0",
        f"effective_roll_limit_deg={EFFECTIVE_ROLL_LIMIT}",
        f"effective_pitch_min_deg={EFFECTIVE_PITCH_MIN}",
        f"effective_pitch_max_deg={EFFECTIVE_PITCH_MAX}",
        f"lateral_nav_constant={N_LATERAL}",
        f"vertical_nav_constant={N_VERTICAL}",
        f"plan_reason={reason}",
        f"anchor_cmd_roll_deg={anchor_roll}",
        f"anchor_cmd_pitch_deg={anchor_pitch}",
        f"dt_s={DT_S}",
        f"raw_roll_deg={raw_roll}",
        f"raw_pitch_deg={raw_pitch}", f"cmd_roll_deg={cmd_roll}",
        f"cmd_pitch_deg={raw_pitch_cmd}",
    ]
    if omit:
        fields = [f for f in fields if not f.startswith(f"{omit}=")]
    return ("12:00:00.000,EVENT:FINAL_APPROACH_RESPONSE_STATE,"
            + ";".join(fields) + ",,,,,,,,,\n")


def _write(path: Path, *, bad_roll: bool = False) -> None:
    path.write_text(
        "".join(["ts,event,payload,,,,,,,,,\n"]
                + [_row(index, bad_roll=bad_roll) for index in range(12)]),
        encoding="utf-8",
    )


def test_command_causality_accepts_commands_explained_by_visual_terms(
    tmp_path: Path,
) -> None:
    path = tmp_path / "navigation_debug.csv"
    _write(path)

    result = analyze(path)

    assert result.sample_count == 12
    assert result.roll_max_residual_deg < 1e-9
    assert result.pitch_max_residual_deg < 1e-9
    assert result.held_sample_count == 0


def test_command_causality_rejects_unexplained_law_output(tmp_path: Path) -> None:
    path = tmp_path / "navigation_debug.csv"
    _write(path, bad_roll=True)

    with pytest.raises(ValueError, match="unexplained roll"):
        analyze(path)


def _causality_rows(path: Path, *, explained: bool, omit: str = "") -> None:
    """A response log the audit accepts, or one it refuses.

    `omit` drops one field from every payload, standing in for a truncated or
    older-format row.
    """
    path.write_text(
        "".join(["ts,event,payload,,,,,,,,,\n"]
                + [_row(index, bad_roll=not explained, omit=omit)
                   for index in range(12)]),
        encoding="utf-8",
    )


def test_causality_evidence_reports_a_clean_audit_without_an_error(
    tmp_path: Path,
) -> None:
    path = tmp_path / "navigation_debug.csv"
    _causality_rows(path, explained=True)

    errors: list[str] = []

    causality = causality_evidence(path, errors)

    assert errors == []
    assert causality["sample_count"] == 12


def test_causality_failure_is_returned_not_raised(tmp_path: Path) -> None:
    """The audit must fail the case WITHOUT unwinding past the scored evidence.

    A raise here reached run_case's broad handler, which returns the audit
    message alone and never writes verdict.json -- discarding a coordinate CPA
    that had already been computed. Observed on direct-pixel-pn-20260818-145026.
    """
    path = tmp_path / "navigation_debug.csv"
    _causality_rows(path, explained=False)

    errors: list[str] = []

    causality = causality_evidence(path, errors)

    assert len(errors) == 1
    assert "command causality" in errors[0]
    assert "error" in causality


def test_a_row_missing_a_field_is_returned_not_raised(tmp_path: Path) -> None:
    """A MISSING field must be contained too, not just a malformed value.

    `_number` indexes the payload directly, so a row tagged with the event but
    short one field raised KeyError -- which is not OSError or ValueError, so it
    sailed straight past the containment above and took verdict.json with it.
    That is precisely the failure `causality_evidence` exists to prevent, so a
    truncated or older-format row must land in `errors` like any other bad data.
    """
    path = tmp_path / "navigation_debug.csv"
    _causality_rows(path, explained=True, omit="control_bearing_deg")

    errors: list[str] = []

    causality = causality_evidence(path, errors)

    assert len(errors) == 1
    assert "command causality" in errors[0]
    assert "control_bearing_deg" in errors[0]
    assert "error" in causality


def _log(path: Path, **row_kwargs) -> None:
    path.write_text(
        "".join(["ts,event,payload,,,,,,,,,\n"]
                + [_row(index, **row_kwargs) for index in range(12)]),
        encoding="utf-8",
    )


@pytest.mark.parametrize(
    "reason", ["bootstrap", "dt_regressed", "outlier", "held", "lateral_held"]
)
def test_every_hold_branch_is_reproduced_by_the_audit(
    tmp_path: Path, reason: str
) -> None:
    """Four branches reissue the anchor and one holds roll only.

    None were modelled before, so a held command read as a broken integration.
    `bootstrap`, `dt_regressed`, `outlier` and `held` reissue both axes;
    `lateral_held` reissues roll while pitch keeps integrating.
    """
    path = tmp_path / "navigation_debug.csv"
    _log(path, reason=reason)

    result = analyze(path)

    assert result.sample_count == 12
    assert result.held_sample_count == 12
    assert result.roll_max_residual_deg < 1e-9
    assert result.pitch_max_residual_deg < 1e-9


def test_a_command_clipped_by_the_structural_cap_is_explained(
    tmp_path: Path,
) -> None:
    """The exact regression: clipping at the CONFIGURED limit, not the real one.

    The law caps roll at 35 deg on top of the autopilot's configured limit.
    Auditing against the configured 45 made every saturated command look
    10.0 deg unexplained -- a standing false failure on every run, which is
    precisely what would hide a real one. Here the raw roll is 42 and the
    command is 35: explained by the cap, and nothing else.
    """
    path = tmp_path / "navigation_debug.csv"
    # 5.06 deg/s of LOS rate drives raw roll past 42 deg through the PN model
    # itself, rather than planting a number the model does not produce.
    _log(path, lateral_rate=5.0)

    result = analyze(path)

    assert result.sample_count == 12
    # Prove the case is real before trusting the pass: if nothing saturated,
    # the assertion below holds trivially and tests nothing.
    commands = [float(line.split(";cmd_roll_deg=")[1].split(";")[0])
                for line in path.read_text(encoding="utf-8").splitlines()[1:]]
    assert max(commands) == EFFECTIVE_ROLL_LIMIT, commands[:3]
    # The clamp explains the command exactly; a 45 deg clip would leave 7.0.
    assert result.roll_max_residual_deg < 1e-9


def test_a_log_without_a_branch_label_is_refused(tmp_path: Path) -> None:
    """A log predating the branch label cannot be audited, and must say so.

    Without `plan_reason` a hold and an integration are indistinguishable, so
    guessing would turn a stale log into a silent pass.
    """
    path = tmp_path / "navigation_debug.csv"
    _log(path, omit="plan_reason")

    errors: list[str] = []

    causality_evidence(path, errors)

    assert len(errors) == 1
    assert "plan_reason" in errors[0]


def test_the_logged_payload_carries_every_field_the_audit_reads() -> None:
    """Producer and consumer are two files; nothing else holds them together.

    The audit reads flat CSV keys while the record keeps value objects, so a
    rename inside `FinalApproachLimits` or `FinalApproachPlanOrigin` would quietly change
    the column set. The audit would then refuse every log, or -- worse, for a
    field it treats as optional -- score a held command as an integration.
    """
    payload = law_evidence_payload(FinalApproachLawEvidence(
        control_bearing_deg=3.0,
        lateral_rate_deg_s=0.2,
        aircraft_turn_rate_deg_s=1.0,
        raw_inertial_los_rate_deg_s=1.2,
        inertial_los_rate_deg_s=1.2,
        aircraft_roll_deg=5.0,
        aircraft_pitch_deg=-8.0,
        air_speed_mps=25.0,
        control_elevation_deg=6.0,
        vertical_rate_deg_s=0.1,
        pitch_time_constant_s=0.5,
        configured_limits=FinalApproachLimits(45.0, -55.0, 25.0),
        effective_limits=FinalApproachLimits(
            EFFECTIVE_ROLL_LIMIT, EFFECTIVE_PITCH_MIN, EFFECTIVE_PITCH_MAX),
        raw_roll_deg=12.0,
        raw_pitch_deg=-7.6,
        origin=FinalApproachPlanOrigin(
            reason="normal",
            anchor_cmd_roll_deg=11.0,
            anchor_cmd_pitch_deg=-7.0,
            dt_s=DT_S,
            lateral_nav_constant=N_LATERAL,
            vertical_nav_constant=N_VERTICAL,
        ),
    ))

    # obs_ts and the two issued commands are written by the caller, not by the
    # evidence payload.
    written_elsewhere = {"obs_ts", "cmd_roll_deg", "cmd_pitch_deg"}
    missing = [
        name for name in _REQUIRED + _OPTIONAL + ("plan_reason",)
        if name not in written_elsewhere and name not in payload
    ]

    assert not missing, missing
    # The flattened values come from inside the value objects, not from a stale
    # second copy sitting beside them.
    assert payload["effective_roll_limit_deg"] == EFFECTIVE_ROLL_LIMIT
    assert payload["roll_limit_deg"] == 45.0
    assert payload["plan_reason"] == "normal"
    assert payload["lateral_nav_constant"] == N_LATERAL
