"""What one scored case flew, written next to its logs as `case.json`.

Archived runs are read back long after the sweep that produced them, and the
manifest is the only place that says which geometry and which clock they were
flown under. `reference-geometry-sets-the-miss-scale` came out of runs whose
misses differed by 10x for no reason visible in the numbers themselves: the
level and dive experiments share a harness and differ only in flags.

So this records the flags, not just the result.

A TRACED case is also named (D7b of the simtime Landing 2 plan): by its
directory, and by its DEFINITION -- the harness's parsed arguments less the
two that do not change what one case flies, the interpreter and the
repetition count, with the sweep's speed list replaced by this case's speed
-- and the sha256 of that definition's strict canonical JSON. A traced child
reads both back and records them as given (`pixel_pn_run_identity`).

With tracing off (`determinism_trace.ENABLED`, read when the manifest is
written) the manifest is what it always was, and nothing is named, defined
or hashed: tracing is default-off, and the untraced path works as it did
(delivery step 6's review).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from navpy.modules.vision.sim import determinism_trace

MANIFEST_NAME = "case.json"
# Parsed arguments that do not change what one case flies.
NOT_DEFINING = ("python", "repetitions")


def case_manifest(
    *,
    speedup: float,
    launch_speedup: float,
    repetition: int,
    poi: Any,
    scoring_start_seq: int,
    identity: dict[str, object],
    speed_plan: Any,
    sitl_params_pushed: Any,
    sitl_params: Any = None,
    args: Any = None,
) -> dict[str, object]:
    """The manifest body every case records, traced or not.

    `speedup` is the SCORED speed -- the one observation freshness and the
    scoring interval span are measured against. `launch_speedup` is what SITL booted
    with. They differ only when the case cruised fast and stepped down to the
    scored speed at `slow_seq`, so a reader can tell a genuine 1x run from a
    20x cruise that was slowed for its final-approach leg.

    The raw `--sitl-param NAME=VALUE` strings pushed on top of the built-in
    parameters come from exactly one of `args`, the harness's parsed
    arguments, which also give a traced case its definition, or
    `sitl_params`, for a caller with no such parse, whose case then has
    none. One of the two is required on purpose: an override like
    SIM_RATE_HZ changes what the run measures and leaves no trace in the
    verdict, so a run that forgets to record it is unreadable afterwards.

    `sitl_params_pushed` records the EFFECTIVE list -- built-ins plus
    overrides, in push order. Raw strings alone are not enough: a default run
    pushes SIM_RATE_HZ=1000 while recording `sitl_params: []`, so two archives
    flown under different defaults would read as comparable.
    """
    if (args is None) == (sitl_params is None):
        raise TypeError("give case_manifest exactly one of args and sitl_params")
    if args is not None:
        sitl_params = args.sitl_param or []
    return {
        "speedup": speedup,
        "launch_speedup": launch_speedup,
        "slow_seq": speed_plan.slow_seq or None,
        "repetition": repetition,
        "poi": asdict(poi),
        "engage_seq": scoring_start_seq,
        "sitl_params": list(sitl_params),
        "sitl_params_pushed": [list(pair) for pair in sitl_params_pushed],
        "source_identity": identity,
    }


def case_definition(args: Any, speedup: float) -> dict[str, Any]:
    """The harness's parsed arguments, as they define this one case."""
    definition = {
        name: value
        for name, value in vars(args).items()
        if name not in NOT_DEFINING
    }
    definition["speedups"] = speedup
    return definition


def definition_sha256(definition: dict[str, Any]) -> str:
    """The sha256 of the definition's canonical JSON: keys sorted, no
    spaces, ASCII, and strict. A value JSON cannot encode raises TypeError,
    and a NaN or an infinity ValueError, where json's default wrote NaN or
    Infinity, which no strict reader accepts (delivery step 6's review)."""
    canonical = json.dumps(
        definition,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("ascii")).hexdigest()


def defined_case(args: Any, speedup: float) -> dict[str, object]:
    """A traced case's definition, its sha256, and why there is none when
    there is none.

    With no parsed arguments, the fleet's path, there is nothing to define
    and no error. A definition with no strict canonical form is left out
    and the reason recorded: the case still flies, and D8.13 finds no
    definition to accept, so it is inspectable, never comparable.
    """
    record: dict[str, object] = {
        "case_definition": None,
        "case_definition_sha256": None,
        "case_definition_error": None,
    }
    if args is None:
        return record
    try:
        definition = case_definition(args, speedup)
        digest = definition_sha256(definition)
    except (TypeError, ValueError) as exc:
        # json's and vars()'s own errors, whose text they made: a repr of
        # one cannot raise.
        record["case_definition_error"] = repr(exc)
        return record
    record["case_definition"] = definition
    record["case_definition_sha256"] = digest
    return record


def write_case_manifest(case_dir: Path, **fields: Any) -> dict[str, object]:
    """Write the manifest and return it, so a caller can assert on what it
    wrote. A traced case's is named first and defined last (D7b)."""
    manifest = case_manifest(**fields)
    if determinism_trace.ENABLED:
        manifest = {
            "case_name": case_dir.name,
            **manifest,
            **defined_case(fields.get("args"), fields["speedup"]),
        }
    case_dir.joinpath(MANIFEST_NAME).write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    return manifest


__all__ = [
    "MANIFEST_NAME",
    "NOT_DEFINING",
    "case_definition",
    "case_manifest",
    "defined_case",
    "definition_sha256",
    "write_case_manifest",
]
