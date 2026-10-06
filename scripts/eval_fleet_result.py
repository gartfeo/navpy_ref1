"""Persist the completed fleet verdict after ordered teardown and sealing."""
from __future__ import annotations

import json
from pathlib import Path


def persist_fleet_result(
    case_dir: Path, vehicles: dict, scoring_policy: str,
    identity: dict[str, object], final_identity: dict[str, object],
    rates: dict[int, float], sys_ids: list[int], seed: int, chat: int,
) -> dict[str, object]:
    result = {
        "passed": all(row["passed"] for row in vehicles.values()),
        "scoring_policy": scoring_policy,
        "vehicles": vehicles,
        "source_identity": identity,
        "final_source_identity": final_identity,
        "clock_rates": {str(s): rates[s] for s in sys_ids},
        "assign_seed": seed,
        "chat": chat,
    }
    (case_dir / "verdict.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8")
    return result
