"""Post-teardown orchestration of the SIM_CPA cross-check for one case.

The one place the pipeline order lives: confirm the slot's SITL is gone,
re-hash the deployed binary, acquire the bound BIN, parse and certify the
module evidence, and compare it to the certified stream score over the
exact common episode.  The live driver and the offline rescore CLI both
come through here so a re-scored case is byte-for-byte the same pipeline
as a live one, just with an explicitly named BIN instead of a binding.

Everything is best-effort: a failure at any stage lands in the block as
that stage's status and the stream verdict is untouched (cross-check-only
authority).
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_models import TargetLocation  # noqa: E402
from eval_navigation_truth_samples import read_track_records  # noqa: E402
from eval_sim_cpa_artifact import acquire_bin  # noqa: E402
from eval_sim_cpa_bin import (  # noqa: E402
    certify_epoch, parse_sim_cpa_records, raw_epoch_global,
)
from eval_sim_cpa_block import (  # noqa: E402
    ARTIFACT_ACQUIRED, ARTIFACT_EXIT_TIMEOUT, COMPARISON_NO_STREAM,
    CONFIG_CONFIGURED,
    EVIDENCE_ACCEPTED, EVIDENCE_PARSE_FAILED, EVIDENCE_PROVENANCE,
    default_artifact, default_comparison, default_evidence,
    default_provenance, sim_cpa_block,
)
from eval_sim_cpa_compare import windowed_comparison  # noqa: E402
from eval_sim_cpa_slot_io import (  # noqa: E402
    arduplane_sha256, wait_for_sitl_exit,
)


def _expected_target(case_dir: Path) -> tuple[int, int, int]:
    from eval_sim_cpa_config import target_ints

    case = json.loads((case_dir / "case.json").read_text(encoding="utf-8"))
    target = case["target"]
    return target_ints(
        target["lat_deg"], target["lon_deg"], target["abs_alt_m"]
    )


def _case_target(case_dir: Path) -> TargetLocation:
    case = json.loads((case_dir / "case.json").read_text(encoding="utf-8"))
    return TargetLocation(**case["target"])


def _score_bin(
    bin_path: Path,
    case_dir: Path,
    truth_block: dict[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """(evidence, comparison) layers for one closed case-local BIN."""
    evidence = default_evidence()
    comparison = default_comparison()
    try:
        expected = _expected_target(case_dir)
        target = _case_target(case_dir)
        scpc, scpa = parse_sim_cpa_records(bin_path)
    except Exception as error:
        evidence["status"] = EVIDENCE_PARSE_FAILED
        evidence["errors"] = [f"{type(error).__name__}: {error}"]
        return evidence, comparison
    certified = certify_epoch(scpc, scpa, expected)
    evidence.update({
        "status": certified.status,
        "errors": list(certified.errors),
        "epoch": certified.epoch,
        "epoch_us": certified.epoch_us,
        "selected_rows": len(certified.rows) or None,
        "first_seq": certified.first_seq,
        "last_seq": certified.last_seq,
    })
    if certified.rows:
        evidence["raw_epoch_global"] = raw_epoch_global(certified.rows)
    if certified.status != EVIDENCE_ACCEPTED:
        return evidence, comparison
    certification_error = (
        None if truth_block is None
        else truth_block.get("certification_error")
    )
    if truth_block is None or certification_error:
        # An uncertified stream defines no certified scored episode, so
        # there is nothing admissible to compare against; the module
        # evidence itself stays recorded (diagnostic value, R/g).
        comparison["status"] = COMPARISON_NO_STREAM
        comparison["error"] = (
            "stream truth uncertified"
            if truth_block is not None else "no stream truth block"
        )
        return evidence, comparison
    try:
        records = read_track_records(case_dir / "truth_track.csv")
    except Exception as error:
        comparison["status"] = COMPARISON_NO_STREAM
        comparison["error"] = (
            f"truth track unreadable: {type(error).__name__}: {error}"
        )
        return evidence, comparison
    episode, module_score, comparison = windowed_comparison(
        certified, records, target, truth_block
    )
    evidence["common_episode"] = episode
    evidence["score"] = module_score
    return evidence, comparison


def score_after_teardown(
    case_dir: Path,
    *,
    sysid: int,
    binding: dict[str, Any],
    configuration: dict[str, Any],
    truth_block: dict[str, Any] | None,
    case_start_wall_s: float,
    arduplane_sha_before: str | None,
) -> dict[str, Any]:
    """The complete ``sim_cpa`` block for a just-flown case."""
    provenance = default_provenance()
    provenance["arduplane_sha256_before"] = arduplane_sha_before
    if configuration.get("status") != CONFIG_CONFIGURED:
        # Disabled, unsupported or failed configuration armed nothing:
        # there is no module evidence to chase, and parsing the (bound but
        # module-silent) BIN would only dress absence up as rejection.
        return sim_cpa_block(
            configuration=configuration, provenance=provenance,
            artifact=default_artifact() | {"binding": binding},
        )
    exited, exit_error = wait_for_sitl_exit(sysid)
    if not exited:
        artifact = default_artifact()
        artifact.update({
            "status": ARTIFACT_EXIT_TIMEOUT,
            "binding": binding,
            "error": exit_error,
        })
        return sim_cpa_block(
            configuration=configuration, provenance=provenance,
            artifact=artifact,
        )
    sha_after, _ = arduplane_sha256()
    provenance["arduplane_sha256_after"] = sha_after
    artifact = acquire_bin(
        binding, case_dir, case_start_wall_s=case_start_wall_s
    )
    if artifact["status"] != ARTIFACT_ACQUIRED:
        return sim_cpa_block(
            configuration=configuration, provenance=provenance,
            artifact=artifact,
        )
    if not arduplane_sha_before or not sha_after or (
        arduplane_sha_before != sha_after
    ):
        evidence = default_evidence()
        evidence["status"] = EVIDENCE_PROVENANCE
        evidence["errors"] = [
            "deployed arduplane identity is not stable across the case "
            f"(before={arduplane_sha_before!r}, after={sha_after!r}); "
            "evidence from an unidentified binary is rejected"
        ]
        return sim_cpa_block(
            configuration=configuration, provenance=provenance,
            artifact=artifact, evidence=evidence,
        )
    evidence, comparison = _score_bin(
        Path(artifact["copied_path"]), case_dir, truth_block
    )
    return sim_cpa_block(
        configuration=configuration, provenance=provenance,
        artifact=artifact, evidence=evidence, comparison=comparison,
    )


def score_offline(
    case_dir: Path,
    bin_path: Path,
    *,
    verdict: dict[str, Any],
) -> dict[str, Any]:
    """Re-run evidence + comparison for a preserved case dir and BIN.

    Binding is operator-asserted (mode offline-explicit): the caller says
    this BIN is the case's artifact, and the SCPC target match still
    certifies the content.  Configuration and provenance layers keep
    whatever the original verdict recorded -- a rescore cannot re-observe
    the flight.
    """
    import hashlib

    existing = verdict.get("sim_cpa") or {}
    configuration = existing.get("configuration") or None
    provenance = existing.get("provenance") or None
    artifact = default_artifact()
    data = bin_path.read_bytes()
    artifact.update({
        "status": ARTIFACT_ACQUIRED,
        "source_path": str(bin_path),
        "copied_path": str(bin_path),
        "size_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "binding": {
            "mode": "offline_explicit",
            "status": "operator_asserted",
            "recorded_wall_time_s": time.time(),
        },
    })
    evidence, comparison = _score_bin(
        bin_path, case_dir, verdict.get("truth")
    )
    return sim_cpa_block(
        configuration=configuration, provenance=provenance,
        artifact=artifact, evidence=evidence, comparison=comparison,
    )


__all__ = ["score_after_teardown", "score_offline"]
