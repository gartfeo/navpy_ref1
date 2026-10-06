"""A case that cruised at 20x must not read back as a 1x case.

`reference-geometry-sets-the-miss-scale` came out of archived runs whose misses
differed by 10x with nothing in the artefacts to say why. Adding a second clock
to the harness adds a second way for that to happen, so the manifest has to
record both speeds, not just the one the directory is named after.

D7b adds a TRACED case's own identity: its name, which is its directory's,
and its DEFINITION, the harness's parsed arguments less what does not change
what one case flies, with the sha256 of their strict canonical JSON. A traced
child records both as given (``pixel_pn_run_identity``). An untraced case's
manifest is, key for key, what it was before (delivery step 6's review).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

from navpy.modules.vision.sim import determinism_trace
from scripts import pixel_pn_case_manifest as case_module
from scripts.pixel_pn_case_manifest import (
    MANIFEST_NAME,
    case_definition,
    definition_sha256,
    write_case_manifest,
)
from scripts.pixel_pn_terminal_speed import TerminalSpeedPlan


@dataclass(frozen=True)
class FakeTarget:
    lat_deg: float = 43.0
    lon_deg: float = 34.0
    rel_alt_m: float = 60.0
    abs_alt_m: float = 60.0


IDENTITY = {"sha256": "abc123", "files": 7}


PUSHED = (("ARMING_CHECK", 0.0), ("SIM_RATE_HZ", 1000.0))
# What an untraced case's manifest holds, in order: all it held before D7b.
UNTRACED_KEYS = [
    "speedup",
    "launch_speedup",
    "slow_seq",
    "repetition",
    "target",
    "engage_seq",
    "sitl_params",
    "sitl_params_pushed",
    "source_identity",
]
# A traced case's: named first, its definition last.
TRACED_KEYS = [
    "case_name",
    *UNTRACED_KEYS,
    "case_definition",
    "case_definition_sha256",
    "case_definition_error",
]


@pytest.fixture
def tracing(monkeypatch: pytest.MonkeyPatch) -> None:
    """Tracing on, as the harness's gate reads it: off the module, when the
    manifest is written."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)


@pytest.fixture
def untraced(monkeypatch: pytest.MonkeyPatch) -> None:
    """Tracing off, whatever the environment the tests run in says."""
    monkeypatch.setattr(determinism_trace, "ENABLED", False)


def _refused(constant: str) -> float:
    """``json.loads``'s hook for NaN and the infinities: a strict reader
    refuses them."""
    raise ValueError(f"not strict JSON: {constant}")


def _write(
    tmp_path: Path,
    plan: TerminalSpeedPlan,
    launch: float,
    sitl_params: list[str] | None = None,
) -> dict:
    write_case_manifest(
        tmp_path,
        speedup=1.0,
        launch_speedup=launch,
        repetition=0,
        target=FakeTarget(),
        scoring_start_seq=4,
        identity=IDENTITY,
        speed_plan=plan,
        sitl_params=sitl_params or [],
        sitl_params_pushed=PUSHED,
    )
    return json.loads((tmp_path / MANIFEST_NAME).read_text(encoding="utf-8"))


def _harness_args(**overrides: Any) -> argparse.Namespace:
    """Arguments shaped like the harness's parse: the two a definition
    drops, the sweep's speed list, and some that define the case."""
    fields: dict[str, Any] = {
        "python": Path("C:/Python311/python.exe"),
        "speedups": "10,1",
        "repetitions": 3,
        "timeout": 180.0,
        "target_alt": 60.0,
        "sitl_param": ["SIM_RATE_HZ=1200"],
        "scoring_policy": "sitl-truth",
        **overrides,
    }
    return argparse.Namespace(**fields)


def _fields(**overrides: Any) -> dict[str, Any]:
    """Every field ``write_case_manifest`` takes but the source of the raw
    overrides, which each caller gives."""
    return {
        "speedup": 1.0,
        "launch_speedup": 1.0,
        "repetition": 1,
        "target": FakeTarget(),
        "scoring_start_seq": 4,
        "identity": IDENTITY,
        "speed_plan": TerminalSpeedPlan(),
        "sitl_params_pushed": PUSHED,
        **overrides,
    }


def _written(case_dir: Path, **fields: Any) -> dict:
    """Write the manifest into ``case_dir`` and read it back from disk."""
    case_dir.mkdir(parents=True, exist_ok=True)
    write_case_manifest(case_dir, **_fields(**fields))
    return json.loads((case_dir / MANIFEST_NAME).read_text(encoding="utf-8"))


def test_a_fast_cruise_is_recorded_alongside_the_speed_it_was_scored_at(
    tmp_path: Path,
) -> None:
    manifest = _write(
        tmp_path, TerminalSpeedPlan(del_speedup=1.0, slow_seq=3), 20.0
    )

    assert manifest["speedup"] == 1.0
    assert manifest["launch_speedup"] == 20.0
    assert manifest["slow_seq"] == 3


def test_a_case_flown_at_one_speed_throughout_says_so(tmp_path: Path) -> None:
    """`slow_seq: null` is what tells a reader nothing was stepped down."""
    manifest = _write(tmp_path, TerminalSpeedPlan(), 1.0)

    assert manifest["speedup"] == manifest["launch_speedup"] == 1.0
    assert manifest["slow_seq"] is None


def test_the_manifest_still_carries_what_it_always_carried(tmp_path: Path) -> None:
    """Archived readers index on these; the new keys are additions, not a swap."""
    manifest = _write(tmp_path, TerminalSpeedPlan(), 1.0)

    assert manifest["repetition"] == 0
    assert manifest["engage_seq"] == 4
    assert manifest["source_identity"] == IDENTITY
    assert manifest["target"]["rel_alt_m"] == 60.0


def test_a_sitl_param_override_is_recorded_verbatim(tmp_path: Path) -> None:
    """An override like SIM_RATE_HZ changes what the run measures and leaves
    no trace in the verdict, so the manifest is the only place it can live."""
    manifest = _write(
        tmp_path, TerminalSpeedPlan(), 1.0, sitl_params=["SIM_RATE_HZ=1200"]
    )

    assert manifest["sitl_params"] == ["SIM_RATE_HZ=1200"]


def test_no_override_reads_back_as_an_empty_list(tmp_path: Path) -> None:
    manifest = _write(tmp_path, TerminalSpeedPlan(), 1.0)

    assert manifest["sitl_params"] == []


def test_the_effective_push_list_is_recorded_even_with_no_overrides(
    tmp_path: Path,
) -> None:
    """Raw strings alone cannot establish the clock: a default run pushes
    SIM_RATE_HZ without any `--sitl-param`, so only the effective list makes
    archives flown under different defaults distinguishable."""
    manifest = _write(tmp_path, TerminalSpeedPlan(), 1.0)

    assert manifest["sitl_params_pushed"] == [
        ["ARMING_CHECK", 0.0], ["SIM_RATE_HZ", 1000.0],
    ]


def test_the_definition_is_the_parsed_arguments_for_this_one_case() -> None:
    definition = case_definition(_harness_args(), 1.0)

    assert definition == {
        "speedups": 1.0,
        "timeout": 180.0,
        "target_alt": 60.0,
        "sitl_param": ["SIM_RATE_HZ=1200"],
        "scoring_policy": "sitl-truth",
    }


def test_the_definition_hash_is_of_its_canonical_json() -> None:
    definition = case_definition(_harness_args(), 1.0)

    canonical = json.dumps(
        definition,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    assert definition_sha256(definition) == (
        hashlib.sha256(canonical.encode("ascii")).hexdigest()
    )


@pytest.mark.parametrize(
    "value", [math.nan, math.inf, -math.inf], ids=["nan", "inf", "minus_inf"]
)
def test_the_definition_hash_refuses_a_number_strict_json_cannot_hold(
    value: float,
) -> None:
    """A float flag accepts "nan" and "inf", and json's default wrote them
    as NaN and Infinity, which no strict reader accepts (delivery step 6's
    review)."""
    with pytest.raises(ValueError):
        definition_sha256({"timeout": value})


@pytest.mark.parametrize(
    "change",
    [
        {"python": Path("D:/other/python.exe")},
        {"repetitions": 30},
        {"speedups": "1"},
    ],
    ids=["python", "repetitions", "other_speeds"],
)
def test_what_does_not_change_the_case_does_not_change_its_hash(
    change: dict[str, Any],
) -> None:
    """The interpreter, how often the sweep repeats, and which other speeds
    it flies: none changes what THIS case flies."""
    reference = definition_sha256(case_definition(_harness_args(), 1.0))

    changed = case_definition(_harness_args(**change), 1.0)

    assert definition_sha256(changed) == reference


@pytest.mark.parametrize(
    ("change", "speedup"),
    [({"timeout": 181.0}, 1.0), ({"sitl_param": None}, 1.0), ({}, 10.0)],
    ids=["timeout", "sitl_param", "speedup"],
)
def test_what_changes_the_case_changes_its_hash(
    change: dict[str, Any], speedup: float
) -> None:
    reference = definition_sha256(case_definition(_harness_args(), 1.0))

    changed = case_definition(_harness_args(**change), speedup)

    assert definition_sha256(changed) != reference


def test_the_manifest_carries_the_definition_and_its_hash(
    tracing, tmp_path: Path
) -> None:
    args = _harness_args()

    manifest = _written(tmp_path / "speed-1-run-1", args=args)

    definition = case_definition(args, 1.0)
    assert list(manifest) == TRACED_KEYS
    assert manifest["case_definition"] == definition
    assert manifest["case_definition_sha256"] == definition_sha256(definition)
    assert manifest["case_definition_error"] is None


def test_the_raw_overrides_are_read_from_the_arguments(tmp_path: Path) -> None:
    manifest = _written(tmp_path, args=_harness_args())

    assert manifest["sitl_params"] == ["SIM_RATE_HZ=1200"]


def test_no_override_in_the_arguments_reads_back_as_an_empty_list(
    tmp_path: Path,
) -> None:
    """argparse leaves an unused repeatable flag at None."""
    manifest = _written(tmp_path, args=_harness_args(sitl_param=None))

    assert manifest["sitl_params"] == []


@pytest.mark.parametrize("given", ["both", "neither"])
def test_the_raw_overrides_come_from_exactly_one_place(
    tmp_path: Path, given: str
) -> None:
    """Given both, one would be recorded and the other silently not; given
    neither, the overrides would go unrecorded."""
    extra = (
        {"args": _harness_args(), "sitl_params": []} if given == "both" else {}
    )

    with pytest.raises(TypeError):
        write_case_manifest(tmp_path, **_fields(**extra))


def test_the_case_is_named_by_its_directory(tracing, tmp_path: Path) -> None:
    manifest = _written(tmp_path / "speed-10-run-2", args=_harness_args())

    assert manifest["case_name"] == "speed-10-run-2"


def test_a_case_given_only_raw_overrides_has_no_definition(
    tracing, tmp_path: Path
) -> None:
    """The fleet harness's path: it names no definition, so a traced child
    records none, which D8.13 reads as ineligible. Not an error: there is
    nothing to define."""
    manifest = _written(tmp_path / "cell-0", sitl_params=["SIM_RATE_HZ=1200"])

    assert list(manifest) == TRACED_KEYS
    assert (
        manifest["case_definition"],
        manifest["case_definition_sha256"],
        manifest["case_definition_error"],
    ) == (None, None, None)
    assert manifest["case_name"] == "cell-0"
    assert manifest["sitl_params"] == ["SIM_RATE_HZ=1200"]


@pytest.mark.parametrize("source", ["args", "sitl_params"])
def test_tracing_off_writes_the_manifest_it_always_wrote(
    untraced, tmp_path: Path, monkeypatch, source: str
) -> None:
    """Delivery step 6's review. Tracing is default-off and the untraced
    path works as it did, yet the harness named, defined and hashed every
    case. An untraced case's manifest is now, key for key, the one it wrote
    before D7b, and nothing is defined or hashed."""

    def gathered(*_: object) -> object:
        pytest.fail("a case was defined or hashed with tracing off")

    monkeypatch.setattr(case_module, "case_definition", gathered)
    monkeypatch.setattr(case_module, "definition_sha256", gathered)
    given: dict[str, Any] = (
        {"args": _harness_args()}
        if source == "args"
        else {"sitl_params": ["SIM_RATE_HZ=1200"]}
    )

    manifest = _written(tmp_path / "speed-1-run-1", **given)

    assert list(manifest) == UNTRACED_KEYS
    assert manifest["sitl_params"] == ["SIM_RATE_HZ=1200"]


@pytest.mark.parametrize(
    ("override", "error"),
    [
        ({"timeout": math.nan}, "ValueError("),
        ({"timeout": math.inf}, "ValueError("),
        ({"timeout": -math.inf}, "ValueError("),
        ({"log_dir": Path("D:/logs")}, "TypeError("),
    ],
    ids=["nan", "inf", "minus_inf", "path"],
)
def test_a_definition_with_no_strict_json_is_recorded_as_why_not(
    tracing, tmp_path: Path, override: dict[str, Any], error: str
) -> None:
    """Delivery step 6's review: a float flag accepts "nan", and json's
    default wrote NaN, which no strict reader accepts, into the hash's
    canonical form and into case.json. A definition with no strict form is
    left out and the reason recorded, and the manifest is still written, so
    the case still flies, with no definition for D8.13 to accept."""
    case_dir = tmp_path / "speed-1-run-1"

    manifest = _written(case_dir, args=_harness_args(**override))

    assert list(manifest) == TRACED_KEYS
    assert (
        manifest["case_definition"], manifest["case_definition_sha256"]
    ) == (None, None)
    reason = manifest["case_definition_error"]
    assert reason.startswith(error), reason
    assert manifest["case_name"] == "speed-1-run-1"
    json.loads(
        (case_dir / MANIFEST_NAME).read_text(encoding="utf-8"),
        parse_constant=_refused,
    )
