"""What makes a certificate run ADMISSIBLE, and a certificate VALID.

Accuracy is not the only criterion. A sub-metre miss flown at 7x, or flown from
a build nobody can name, is a good result that certifies nothing -- and before
these checks existed both would have been counted as a pass and folded into the
aggregate. The measured rate and the identity artifact were computed and then
ignored; these tests exist so they decide something.
"""
from __future__ import annotations

import pytest

from scripts import eval_certificate as cert


# --------------------------------------------------------------------------
# The measured scoring interval rate must decide.
# --------------------------------------------------------------------------


def test_a_rate_inside_the_band_is_admissible():
    assert cert.certificate_rate_error(10.0, 10) is None
    assert cert.certificate_rate_error(9.0, 10) is None
    assert cert.certificate_rate_error(11.0, 10) is None


@pytest.mark.parametrize("rate", [12.0, 7.0, 11.01, 8.99])
def test_a_rate_outside_the_band_disqualifies_the_run(rate):
    # A different clock is a different experiment, however good the miss was.
    assert "outside" in cert.certificate_rate_error(rate, 10)


@pytest.mark.parametrize("rate", [None, float("nan"), "", float("inf")])
def test_an_unmeasured_rate_disqualifies_rather_than_passes(rate):
    # "We could not measure it" is exactly the thing Phase 1a exists to stop
    # being treated as "it was 10x".
    assert "not measured" in cert.certificate_rate_error(rate, 10)


def test_the_certificate_band_is_tighter_than_the_launcher_gate():
    # The launcher's 25% catches an order-of-magnitude mix-up over a few seconds
    # at boot. This one is measured over the whole scored scoring interval and is a
    # criterion rather than a sanity check, so it has to be tighter.
    assert cert.CERTIFICATE_RATE_TOLERANCE == pytest.approx(0.10)


# --------------------------------------------------------------------------
# The identity artifact must decide.
# --------------------------------------------------------------------------


def _identity(*, dirty=False, commit="abc123", complete=True,
              autopilot_available=True, wsl_available=True):
    return {
        "navpy": {"commit": commit, "dirty": dirty, "dirty_files": []},
        "mission": {"item_count": 5, "sha256": "m" * 8},
        "parameters": {"complete": complete, "parameter_count": 3,
                       "vehicle_param_count": 3 if complete else 4,
                       "sha256": "p" * 8},
        "autopilot": {"available": autopilot_available,
                      "reason": "" if autopilot_available else "no answer",
                      "flight_sw_version": 0x04050000,
                      "flight_custom_version": "f" * 16},
        "ardupilot_source": {"available": wsl_available,
                             "reason": "" if wsl_available else "no wsl"},
        "eeprom": {"available": wsl_available},
    }


def test_a_clean_identity_is_admissible():
    assert cert.certificate_identity_errors(_identity()) == []


def test_a_dirty_navpy_tree_disqualifies_the_run():
    # The certificate names a commit; a run flown with uncommitted edits was not
    # flown from that commit.
    errors = cert.certificate_identity_errors(_identity(dirty=True))
    assert any("dirty" in error for error in errors)


def test_an_unknown_tree_state_disqualifies_too():
    identity = _identity()
    identity["navpy"]["dirty"] = None
    assert cert.certificate_identity_errors(identity)


def test_a_missing_commit_disqualifies():
    assert cert.certificate_identity_errors(_identity(commit=None))


def test_an_incomplete_parameter_snapshot_disqualifies_the_run():
    errors = cert.certificate_identity_errors(_identity(complete=False))
    assert any("incomplete" in error for error in errors)


def test_missing_firmware_identity_disqualifies_the_run():
    errors = cert.certificate_identity_errors(_identity(autopilot_available=False))
    assert any("firmware identity unavailable" in error for error in errors)


def test_absent_wsl_probes_alone_do_not_disqualify():
    # They describe a checkout sitting beside the binary, not the binary. As
    # long as the vehicle itself said what it runs, the result is attributable.
    assert cert.certificate_identity_errors(_identity(wsl_available=False)) == []


def test_no_identity_at_all_disqualifies():
    assert cert.certificate_identity_errors(None)
    assert cert.certificate_identity_errors({})


# --------------------------------------------------------------------------
# Repetitions must be repetitions OF THE SAME THING.
# --------------------------------------------------------------------------


def _row(index, *, commit="abc", mission="m1", params="p1", autopilot="fw1",
         passed=True, invalid=""):
    return {
        "index": 0,
        "repetition": index,
        "name": f"run-{index}",
        "passed": passed,
        "dist_3d_m": 0.2,
        "coordinate_dist_3d_m": 0.22,
        "measured_clock_rate": 10.0,
        "speedup": 10,
        "error": "",
        "certificate_invalid_reason": invalid,
        "identity_navpy_commit": commit,
        "identity_mission_sha": mission,
        "identity_parameters_sha": params,
        "identity_autopilot": autopilot,
    }


def test_consistent_repetitions_produce_a_valid_certificate():
    summary = cert.certificate_summary([_row(1), _row(2), _row(3)])
    assert summary["valid"]
    assert summary["consistency_errors"] == []
    assert summary["invalid_runs"] == 0


@pytest.mark.parametrize("field",
                         ["commit", "mission", "params", "autopilot"])
def test_any_identity_change_between_repetitions_voids_the_certificate(field):
    # Runs of different builds, missions or configurations are not repetitions;
    # their spread measures whatever moved, not the build's repeatability.
    summary = cert.certificate_summary([_row(1), _row(2, **{field: "other"})])
    assert not summary["valid"]
    assert summary["consistency_errors"]


def test_one_inadmissible_run_voids_the_whole_certificate():
    summary = cert.certificate_summary([
        _row(1),
        _row(2, invalid="engagement clock rate 12.00x is outside 9.00-11.00x"),
    ])
    assert not summary["valid"]
    assert summary["invalid_runs"] == 1
    assert summary["invalid_reasons"] == [
        "engagement clock rate 12.00x is outside 9.00-11.00x"]


def test_a_failed_run_voids_the_certificate_even_when_identity_matches():
    assert not cert.certificate_summary([_row(1), _row(2, passed=False)])["valid"]


def test_an_empty_certificate_is_not_valid():
    # Zero runs prove nothing -- and `all([])` is True, so this needs saying.
    assert not cert.certificate_summary([])["valid"]


# --------------------------------------------------------------------------
# Mission identity covers the WHOLE row.
# --------------------------------------------------------------------------


class _Item:
    def __init__(self, **fields):
        for name in cert.MISSION_IDENTITY_FIELDS:
            setattr(self, name, fields.get(name))


def _base(**overrides):
    fields = dict(seq=0, command=16, frame=3, lat_deg=40.0, lon_deg=44.0,
                  mission_alt_m=100.0)
    fields.update(overrides)
    return _Item(**fields)


def test_two_missions_differing_only_in_hold_time_hash_differently():
    # param1 on NAV_WAYPOINT is hold time. Hashing position only made these two
    # missions -- which the aircraft flies differently -- indistinguishable.
    assert (cert.mission_identity([_base(param1=0.0)])["sha256"]
            != cert.mission_identity([_base(param1=30.0)])["sha256"])


@pytest.mark.parametrize(
    "field", ["param1", "param2", "param3", "param4", "current",
              "autocontinue", "mission_type"])
def test_every_previously_dropped_field_reaches_the_hash(field):
    assert (cert.mission_identity([_base()])["sha256"]
            != cert.mission_identity([_base(**{field: 7})])["sha256"]), field


def test_identical_missions_still_hash_identically():
    assert (cert.mission_identity([_base(param1=1.0), _base(seq=1)])
            == cert.mission_identity([_base(param1=1.0), _base(seq=1)]))
