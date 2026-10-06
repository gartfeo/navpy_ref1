"""The SIM_CPA cross-check as one lifecycle stage of a harness case.

The single-UAV driver's run_case owns flying; this module owns the
cross-check's touchpoints in that lifecycle, in the reconciled order
(R3/R7):

- pre_flight, after the generic parameter pushes and before the child
  exists: successor binding and the binary hash are read BEFORE the
  module is configured and long before arming -- LASTLOG only advances
  when logging starts, so this is the last point the pre-flight number
  is unambiguous -- then the module is configured from the case POI.
- post_teardown, only after the chat's SITL stack was stopped: score the
  bound BIN.  A crash inside module scoring lands in the returned block;
  the cross-check may never cost the case its stream verdict.
- error_block, on the case's failure path: no module scoring (there is
  no certified episode to cross-check), but how far configuration got is
  still part of the audit trail.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_direct_pixel_verdict import SCORING_POLICY_SITL_TRUTH  # noqa: E402
from eval_navigation_models import PoiLocation  # noqa: E402
from eval_sim_cpa_artifact import binding_pre_flight  # noqa: E402
from eval_sim_cpa_block import (  # noqa: E402
    CONFIG_INTERNAL_ERROR, EVIDENCE_INTERNAL_ERROR, default_configuration,
    sim_cpa_block,
)
from eval_sim_cpa_config import MODE_AUTO, configure_sim_cpa  # noqa: E402
from eval_sim_cpa_score import score_after_teardown  # noqa: E402
from eval_sim_cpa_slot_io import arduplane_sha256  # noqa: E402


class SimCpaStage:
    """Cross-check context for one case, bound before the case's try block
    so an exception at any point still records how far it got."""

    def __init__(self, args: Any) -> None:
        mode = getattr(args, "sim_cpa_mode", None)
        self.mode = mode
        self.active = (
            mode is not None
            and args.scoring_policy == SCORING_POLICY_SITL_TRUTH
        )
        self.binding: dict[str, Any] | None = None
        self.configuration: dict[str, Any] | None = None
        self.arduplane_sha_before: str | None = None
        self.case_start_wall_s = time.time()

    def pre_flight(
        self,
        master: Any,
        *,
        sysid: int,
        poi: PoiLocation,
        case_dir: Path,
    ) -> None:
        """Arm binding, hash the binary, configure the module.

        Never raises: an unexpected crash in the cross-check's own
        pre-flight code must not abort the flight it exists to observe
        (cross-check-only authority; 90_review finding 3).  It lands as
        an internal_error configuration and the case flies stream-scored.
        """
        if not self.active:
            return
        try:
            if self.mode == MODE_AUTO:
                self.binding = binding_pre_flight(sysid)
                self.arduplane_sha_before, _ = arduplane_sha256()
            self.configuration = configure_sim_cpa(
                master,
                mode=self.mode,
                lat_deg=poi.lat_deg,
                lon_deg=poi.lon_deg,
                abs_alt_m=poi.abs_alt_m,
                case_dir=case_dir,
            )
        except Exception as error:
            configuration = default_configuration()
            configuration["mode"] = self.mode
            configuration["status"] = CONFIG_INTERNAL_ERROR
            configuration["error"] = (
                "pre-flight cross-check crashed: "
                f"{type(error).__name__}: {error}"
            )
            self.configuration = configuration

    def post_teardown(
        self,
        case_dir: Path,
        *,
        sysid: int,
        truth_block: dict[str, Any] | None,
    ) -> dict[str, Any]:
        if self.configuration is None:
            return sim_cpa_block()
        try:
            return score_after_teardown(
                case_dir,
                sysid=sysid,
                binding=self.binding or {
                    "status": "unavailable",
                    "error": "binding was never armed",
                },
                configuration=self.configuration,
                truth_block=truth_block,
                case_start_wall_s=self.case_start_wall_s,
                arduplane_sha_before=self.arduplane_sha_before,
            )
        except Exception as error:
            block = sim_cpa_block(configuration=self.configuration)
            # An explicit status, not just an error string: the summary's
            # rejected counter must see this row (90_review finding 4).
            block["evidence"]["status"] = EVIDENCE_INTERNAL_ERROR
            block["evidence"]["errors"] = [
                f"module scoring crashed: {type(error).__name__}: {error}"
            ]
            return block

    def error_block(self) -> dict[str, Any]:
        return sim_cpa_block(configuration=self.configuration)


__all__ = ["SimCpaStage"]
