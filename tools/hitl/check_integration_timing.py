"""Offline timing validity of explicit integration timing (hitl_final_approach.md section 9, stage 1 step 12).

    python -m tools.hitl.check_integration_timing fit   --run-record R.json --navpy N.jsonl --publisher F.csv ...
        --unreal U.log --emulator E.jsonl --out constants.json
    python -m tools.hitl.check_integration_timing check --run-record R.json --navpy N.jsonl --publisher F.csv ...
        --unreal U.log --emulator E.jsonl --constants constants.json [--observation] --out report.json

One receiver, one stream, one run per call (logs: tools/hitl/timing_inputs.py; joins: timing_joins.py).

``fit`` (calibration-only run): each source's receiver-local correction C = Ds - Dc (integration_timing.py), with
M = the frame's mapped receiver time ``mapped_s``:
- attitude: A = navpy's ATTITUDE capture receipts interpolated at the frame's represented board time B (Unreal's
  render stamp); pair A - M.
- gimbal: G = navpy's gimbal readback receipts interpolated at the rendered plant time T_r (Unreal's gimbal bracket
  on the emulator's steps), over the plant times the joined 0x0D frames reported; pair G - M.
Each receipt bracket is two adjacent samples of navpy's own history, in receipt order, with a rising source time
and a receipt width inside navpy's bracket limit. No host's clock is subtracted from another's: A and G are receiver
receipts placed by sample identity; B, T_r and the reported plant times stay on their own source clocks. C is the
median of at least MIN_PAIRS pairs; only the constants go into navpy (the next run record).

``check`` (held-out run, constants frozen in its run record): every frame that mapped must join completely.
- attitude skew = navpy's associated ATTITUDE time_boot - B;
- gimbal skew = the plant time of navpy's associated readbacks (by weight) - T_r, on the emulator's clock. Until
  step 18 every zoom-capable mount is refused before the gimbal lookup, so for those frames the check replays
  navpy's own ``interpolate_gimbal_at`` on the snapshot the attempt logged (``replayed``); a live association must
  equal its replay.
Median (signed), p95 and max |skew| per source; a pass needs max |skew| + uncertainty <= SKEW_BUDGET_S, with
uncertainty = the fit's interval + the join resolution, and the smallest represented interval >= 10 x the largest
PTS interval error plus its resolution. ``--observation`` adds the held-out 10-minute rule: >= 600 s of scored
frames per source, no gap between scored frames over COVERAGE_GAP_S, and no interruption of the map after it first
mapped. Missing, ambiguous or over-budget joins fail.

Exit: 0 pass, 1 fail, 2 unreadable input.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.capture_interpolation import BRACKET_MAX_WIDTH_S
from navpy.core.perception.camera.integration_timing import (
    STATUS_ABORTED,
    STATUS_UNAVAILABLE,
    STATUS_UNKNOWN_FRAME,
    STATUS_UNSETTLED,
    IntegrationTimingConfig,
    IntegrationTimingLimits,
)
from navpy.core.perception.pipeline.integration_association import (
    STATUS_ZOOM_UNSETTLED,
    interpolate_gimbal_at,
)
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory, GimbalHistorySample
from tools.hitl import approach_records
from tools.hitl.encoded_capture_score import INTERVAL_TOLERANCE_NS
from tools.hitl.timing_inputs import (
    Association,
    AttitudeReceipt,
    NavpyLog,
    TimingInputError,
    read_emulator_log,
    read_navpy_log,
    read_publisher_frames,
    read_unreal_slots,
)
from tools.hitl.timing_joins import (
    GimbalJoin,
    JoinError,
    NavpyFrame,
    join_gimbal,
    join_publisher,
    join_slots,
    rendered_plant_time,
)
from tools.hitl.timing_stats import MIN_BLOCKS, block_count, fit_median, summarize

FIT_SCHEMA = "navpy.hitl.integration_timing_fit.v1"
REPORT_SCHEMA = "navpy.hitl.integration_timing_check.v1"
MIN_PAIRS = 30                       # plan: >= 30 calibration-only pairs per source
SKEW_BUDGET_S = 0.008                # plan: max skew + join/fit uncertainty <= 8 ms
INTERVAL_RATIO = 10.0                # plan: minimum represented interval >= 10 x max PTS interval error
OBSERVATION_S = 600.0                # plan: held-out 10-minute observation
# My values, not set by the plan:
# - the moving-block length: the map's slowest memory, step 11's 10 s rate window;
BOOTSTRAP_BLOCK_S = IntegrationTimingLimits().rate_window_s
# - ArduPilot's ATTITUDE time_boot_ms is AP_HAL::millis(), whole milliseconds cut down from the board's micros, so an
#   associated time_boot can sit up to 1 ms below the board time it stands for (one-sided; taken as 1 ms);
ATTITUDE_JOIN_RESOLUTION_S = 0.001
# - the emulator's report and step times are floats on one clock: no resolution term;
GIMBAL_JOIN_RESOLUTION_S = 0.0
# - a hole between scored frames longer than this breaks the observation's coverage (2 x the 0.5 s PTS gap limit).
COVERAGE_GAP_S = 1.0
CORRECTION_MATCH_S = 1e-9            # a logged lookup - mapped equals the frozen C to float precision
NS_PER_S = 1e9

# Refusals a map gives before it has settled; after the first mapped frame any of them is an interruption.
_MAP_REFUSALS = frozenset({STATUS_UNSETTLED, STATUS_UNKNOWN_FRAME, STATUS_UNAVAILABLE, STATUS_ABORTED})


@dataclass
class Joined:
    """One receiver's logs with every mapped frame joined to its render stamp."""

    config: IntegrationTimingConfig
    log: NavpyLog
    frames: list[Association]                     # one association per navpy frame sequence, log order
    board_s: dict[int, float]                     # sequence -> represented board time B
    rendered_t: dict[int, float]                  # sequence -> rendered plant time T_r
    gimbal: GimbalJoin
    sender_frames_unlogged: int                   # pushed frames between joined ones without a navpy frame
    interrupted: list[str] = field(default_factory=list)

    @cached_property
    def gimbal_receipts(self) -> list[float]:
        return [sample.receipt_s for sample in self.gimbal.samples]

    @cached_property
    def gimbal_index(self) -> dict[float, int]:
        """Receipt -> position in the joined gimbal samples (receipts rise strictly, so each is unique)."""
        return {sample.receipt_s: index for index, sample in enumerate(self.gimbal.samples)}


def _frames(log: NavpyLog, config: IntegrationTimingConfig) -> tuple[list[Association], list[str]]:
    """The mapped attempts, one per frame sequence, and the map interruptions after the first one."""
    opens = {item.get("open_id") for item in log.associations if item.get("open_id") is not None}
    if len(opens) > 1:
        raise JoinError("join_ambiguous", f"{len(opens)} capture opens in one navpy log; one per check")
    if log.run_ids != {config.run_id}:
        raise JoinError("join_invalid", f"navpy log run ids {sorted(log.run_ids)} are not the record's "
                                        f"{config.run_id!r}")
    frames: dict[int, Association] = {}
    interrupted: list[str] = []
    mapped_once = False
    for item in log.associations:
        status = item.status
        if status in _MAP_REFUSALS and (mapped_once or status == STATUS_ABORTED):
            interrupted.append(f"log line {item.order}: {status}")
        if item.get("mapped_s") is None:
            continue
        mapped_once = True
        sequence = item.get("frame_sequence")
        if not isinstance(sequence, int) or not isinstance(item.get("pts_ns"), int):
            raise JoinError("join_invalid", f"log line {item.order}: mapped frame without sequence/PTS")
        known = frames.get(sequence)
        if known is not None:
            if known.get("pts_ns") != item.get("pts_ns") or known.get("mapped_s") != item.get("mapped_s"):
                raise JoinError("join_ambiguous", f"frame {sequence} logged with two PTS or mapped times")
            continue
        frames[sequence] = item
    if not frames:
        raise JoinError("join_missing", "no frame ever mapped")
    return list(frames.values()), interrupted


def join_run(config: IntegrationTimingConfig, log: NavpyLog, publisher, slots, emulator) -> Joined:
    frames, interrupted = _frames(log, config)
    navpy = [NavpyFrame(int(item.get("frame_sequence")), int(item.get("pts_ns"))) for item in frames]
    published = join_publisher(navpy, publisher)
    rendered = join_slots(published.frames, slots)
    board_s = {sequence: slot.board_us / 1e6 for sequence, slot in rendered.items()}
    rendered_t = {sequence: rendered_plant_time(slot, emulator) for sequence, slot in rendered.items()}
    gimbal = join_gimbal(log.gimbal, emulator)
    return Joined(config, log, frames, board_s, rendered_t, gimbal, published.lost, interrupted)


# --- receipt brackets ----------------------------------------------------------------------------------------------
def _attitude_series(receipts: Sequence[AttitudeReceipt]) -> list[AttitudeReceipt]:
    for earlier, later in zip(receipts, receipts[1:]):
        if later.receipt_s <= earlier.receipt_s or later.time_boot_s < earlier.time_boot_s:
            raise JoinError("join_invalid", "navpy ATTITUDE history runs backwards (a reset or reorder)")
    return list(receipts)


def _receipt_at(source_t: Sequence[float], receipts: Sequence[float], at: float, width_max_s: float) -> float | None:
    """The receipt interpolated at source time ``at`` between two adjacent samples, or None (no bracket).

    ``source_t`` never falls (checked by the caller); a pair with equal source times cannot be inverted and is no
    bracket.
    """
    index = bisect.bisect_right(source_t, at)
    if index == len(source_t) and source_t and source_t[-1] == at:
        index -= 1
    if index == 0 or index >= len(source_t):
        return None
    first, second = source_t[index - 1], source_t[index]
    if not first <= at <= second or second <= first:
        return None
    width = receipts[index] - receipts[index - 1]
    if width > width_max_s:
        return None
    return receipts[index - 1] + (at - first) / (second - first) * width


@dataclass(frozen=True)
class Pairs:
    values: list[float]
    times: list[float]            # the source time of each pair, for the bootstrap's blocks
    unpaired: int


def calibration_pairs(joined: Joined) -> tuple[Pairs, Pairs]:
    limits = joined.config.limits
    attitude = _attitude_series(joined.log.attitude)
    boots = [sample.time_boot_s for sample in attitude]
    attitude_receipts = [sample.receipt_s for sample in attitude]
    gimbal_receipts = [sample.receipt_s for sample in joined.gimbal.samples]
    plant = list(joined.gimbal.plant_t)
    a_values, a_times, g_values, g_times = [], [], [], []
    a_missing = g_missing = 0
    for item in joined.frames:
        sequence = int(item.get("frame_sequence"))
        mapped_s = float(item.get("mapped_s"))
        board = joined.board_s[sequence]
        receipt = _receipt_at(boots, attitude_receipts, board, BRACKET_MAX_WIDTH_S)
        if receipt is None:
            a_missing += 1
        else:
            a_values.append(receipt - mapped_s)
            a_times.append(board)
        rendered = joined.rendered_t[sequence]
        receipt = _receipt_at(plant, gimbal_receipts, rendered, limits.gimbal_bracket_max_s)
        if receipt is None:
            g_missing += 1
        else:
            g_values.append(receipt - mapped_s)
            g_times.append(rendered)
    return Pairs(a_values, a_times, a_missing), Pairs(g_values, g_times, g_missing)


def fit(joined: Joined) -> dict[str, Any]:
    attitude, gimbal = calibration_pairs(joined)
    result: dict[str, Any] = {"schema": FIT_SCHEMA, "run_ids": sorted(joined.log.run_ids),
                              "source_id": joined.config.source_id, "link_id": joined.config.link_id,
                              "method": {"statistic": "median", "interval": "moving-block bootstrap 95 %",
                                         "block_s": BOOTSTRAP_BLOCK_S, "min_pairs": MIN_PAIRS,
                                         "min_blocks": MIN_BLOCKS}}
    failures = []
    for name, pairs in (("attitude", attitude), ("gimbal", gimbal)):
        section: dict[str, Any] = {"pairs": len(pairs.values), "unpaired": pairs.unpaired}
        if len(pairs.values) < MIN_PAIRS:
            failures.append(f"{name}: {len(pairs.values)} pairs, fewer than {MIN_PAIRS}")
        elif block_count(pairs.times, BOOTSTRAP_BLOCK_S) < MIN_BLOCKS:
            failures.append(f"{name}: pairs span fewer than {MIN_BLOCKS} blocks of {BOOTSTRAP_BLOCK_S:g} s")
        else:
            section.update(fit_median(pairs.values, pairs.times, BOOTSTRAP_BLOCK_S).record())
        result[name] = section
    result["failures"] = failures
    result["passed"] = not failures
    return result


# --- check ---------------------------------------------------------------------------------------------------------
def _dummy_attitude() -> Attitude:
    return Attitude(0.0, 0.0, 0.0)


def _replay_gimbal(joined: Joined, item: Association, lookup_s: float):
    """navpy's own gimbal lookup on the snapshot this attempt logged: (earlier index, later index, weight) or None."""
    snapshot = item.get("gimbal_history")
    if not isinstance(snapshot, dict) or not snapshot.get("count"):
        return None, "replay_no_history"
    if snapshot.get("session") != joined.gimbal.session:
        return None, "replay_other_session"
    first, last = snapshot["first_receipt_s"], snapshot["last_receipt_s"]
    receipts = joined.gimbal_receipts
    indexes = [index for index in range(bisect.bisect_left(receipts, first), bisect.bisect_right(receipts, last))
               if joined.gimbal.samples[index].order < item.order]
    if len(indexes) != snapshot["count"]:
        raise JoinError("join_missing", f"log line {item.order}: {len(indexes)} logged gimbal receipts, the "
                                        f"attempt saw {snapshot['count']}")
    now_s = item.get("now_s")
    if not isinstance(now_s, (int, float)) or now_s - last > joined.config.limits.max_source_age_s:
        return None, "replay_source_stale"
    samples = tuple(GimbalHistorySample(joined.gimbal.session, joined.gimbal.samples[index].seq,
                                        joined.gimbal.samples[index].receipt_s, _dummy_attitude())
                    for index in indexes)
    history = GimbalCaptureHistory(joined.gimbal.session, samples, GimbalData(att=_dummy_attitude()))
    found, status = interpolate_gimbal_at(history, lookup_s, joined.config.limits.gimbal_bracket_max_s)
    if found is None:
        return None, f"replay_{status}"
    by_receipt = joined.gimbal_index
    return (by_receipt[found.earlier.receipt_s], by_receipt[found.later.receipt_s], found.fraction), "replayed"


def _corrections_match(item: Association, mapped_s: float, config: IntegrationTimingConfig) -> list[str]:
    problems = []
    for name, correction in (("attitude_lookup_s", config.attitude_correction_s),
                             ("gimbal_lookup_s", config.gimbal_correction_s)):
        value = item.get(name)
        if value is not None and abs(float(value) - mapped_s - correction) > CORRECTION_MATCH_S:
            problems.append(f"log line {item.order}: {name} - mapped_s is not the frozen {correction}")
    return problems


def check(joined: Joined, constants: dict[str, Any], observation: bool) -> dict[str, Any]:
    config = joined.config
    failures: list[str] = list(f"map interrupted: {text}" for text in joined.interrupted)
    if set(constants.get("run_ids", [])) & joined.log.run_ids:
        failures.append("the checked run was used to fit the constants (not held out)")
    for name, key in (("attitude", "attitude_correction_s"), ("gimbal", "gimbal_correction_s")):
        if constants.get(name, {}).get("correction_s") != getattr(config, key):
            failures.append(f"run record {key} {getattr(config, key)} is not the fitted constant")
    samples = joined.gimbal.samples
    plant = joined.gimbal.plant_t
    attitude_skew, attitude_t, gimbal_skew, gimbal_t = [], [], [], []
    counts: dict[str, int] = {}
    for item in joined.frames:
        sequence = int(item.get("frame_sequence"))
        mapped_s = float(item.get("mapped_s"))
        failures += _corrections_match(item, mapped_s, config)
        boot = item.get("attitude_time_boot_s")
        if boot is None:
            counts[f"attitude:{item.status}"] = counts.get(f"attitude:{item.status}", 0) + 1
            continue
        attitude_skew.append(float(boot) - joined.board_s[sequence])
        attitude_t.append(joined.board_s[sequence])
        live = item.get("gimbal_receipts_s")
        replay = None
        if live is not None or item.status == STATUS_ZOOM_UNSETTLED:
            replay, how = _replay_gimbal(joined, item, mapped_s + config.gimbal_correction_s)
        else:
            how = f"not_looked_up:{item.status}"
        if live is not None:
            by_receipt = joined.gimbal_index
            try:
                found = (by_receipt[live[0]], by_receipt[live[1]], float(item.get("gimbal_weight")))
            except KeyError:
                raise JoinError("join_missing", f"log line {item.order}: associated gimbal receipts not logged")
            if replay is None or replay[:2] != found[:2] or abs(replay[2] - found[2]) > 1e-9:
                failures.append(f"log line {item.order}: live gimbal association differs from its replay")
            replay, how = found, "live"
        counts[f"gimbal:{how}"] = counts.get(f"gimbal:{how}", 0) + 1
        if replay is None:
            continue
        earlier, later, weight = replay
        associated = plant[earlier] + weight * (plant[later] - plant[earlier])
        gimbal_skew.append(associated - joined.rendered_t[sequence])
        gimbal_t.append(joined.board_s[sequence])
    report: dict[str, Any] = {"schema": REPORT_SCHEMA, "run_ids": sorted(joined.log.run_ids),
                              "frames_mapped": len(joined.frames), "counts": counts,
                              "sender_frames_unlogged": joined.sender_frames_unlogged,
                              "budget_s": SKEW_BUDGET_S, "observation": observation}
    for name, skews, times, resolution in (("attitude", attitude_skew, attitude_t, ATTITUDE_JOIN_RESOLUTION_S),
                                            ("gimbal", gimbal_skew, gimbal_t, GIMBAL_JOIN_RESOLUTION_S)):
        if not skews:
            failures.append(f"{name}: no associated frames")
            report[name] = {"count": 0}
            continue
        summary = summarize(skews)
        uncertainty = float(constants.get(name, {}).get("uncertainty_s", math.inf)) + resolution
        section = {**summary.record(), "uncertainty_s": uncertainty,
                   "span_s": max(times) - min(times), "max_gap_s": _max_gap(times)}
        if summary.max_abs_s + uncertainty > SKEW_BUDGET_S:
            failures.append(f"{name}: max |skew| {summary.max_abs_s * 1e3:.3f} ms + uncertainty "
                            f"{uncertainty * 1e3:.3f} ms is over {SKEW_BUDGET_S * 1e3:g} ms")
        if observation:
            if section["span_s"] < OBSERVATION_S:
                failures.append(f"{name}: scored frames span {section['span_s']:.1f} s, under {OBSERVATION_S:g} s")
            if section["max_gap_s"] > COVERAGE_GAP_S:
                failures.append(f"{name}: a {section['max_gap_s']:.3f} s hole between scored frames")
        report[name] = section
    report["intervals"] = _intervals(joined, failures)
    report["failures"] = failures
    report["passed"] = not failures
    return report


def _max_gap(times: Sequence[float]) -> float:
    ordered = sorted(times)
    return max((later - earlier for earlier, later in zip(ordered, ordered[1:])), default=0.0)


def _intervals(joined: Joined, failures: list[str]) -> dict[str, float]:
    """Consecutive mapped frames: navpy's PTS interval against the represented board-time interval."""
    ordered = sorted(joined.frames, key=lambda item: int(item.get("frame_sequence")))
    represented, errors = [], []
    for earlier, later in zip(ordered, ordered[1:]):
        first, second = int(earlier.get("frame_sequence")), int(later.get("frame_sequence"))
        board = joined.board_s[second] - joined.board_s[first]
        pts = (int(later.get("pts_ns")) - int(earlier.get("pts_ns"))) / NS_PER_S
        represented.append(board)
        errors.append(abs(pts - board))
    if not represented:
        failures.append("intervals: fewer than two mapped frames")
        return {}
    resolution = INTERVAL_TOLERANCE_NS / NS_PER_S
    worst = max(errors) + resolution
    smallest = min(represented)
    if smallest < INTERVAL_RATIO * worst:
        failures.append(f"intervals: smallest represented {smallest * 1e3:.3f} ms is under {INTERVAL_RATIO:g} x "
                        f"{worst * 1e3:.4f} ms")
    return {"min_represented_s": smallest, "max_pts_error_s": max(errors), "resolution_s": resolution}


# --- command line --------------------------------------------------------------------------------------------------
def _config(record: dict[str, Any]) -> IntegrationTimingConfig:
    section = record["integration_timing"]
    return IntegrationTimingConfig(
        run_id=record["run"]["run_id"], mode=record["run"]["mode"], source_id=section["source_id"],
        link_id=section["link_id"], limits=IntegrationTimingLimits(**section["limits"]),
        attitude_correction_s=section["attitude_correction_s"], gimbal_correction_s=section["gimbal_correction_s"])


def _read_lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8", errors="strict").splitlines()


def load(args: argparse.Namespace) -> Joined:
    config = _config(approach_records.read_record(Path(args.run_record)))
    log = read_navpy_log(_read_lines(Path(args.navpy)), args.navpy)
    publisher = read_publisher_frames(Path(path) for path in args.publisher)
    slots = read_unreal_slots(_read_lines(Path(args.unreal)), args.unreal)
    emulator = read_emulator_log(_read_lines(Path(args.emulator)), args.emulator)
    return join_run(config, log, publisher, slots, emulator)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.check_integration_timing",
                                     description=__doc__.split("\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("fit", "check"):
        command = commands.add_parser(name)
        command.add_argument("--run-record", required=True, help="the run's record (tools/hitl/approach_records.py)")
        command.add_argument("--navpy", required=True, help="navpy's integration log (JSONL), one receiver")
        command.add_argument("--publisher", required=True, nargs="+", help="the publisher's frames-NNN.csv, in order")
        command.add_argument("--unreal", required=True, help="Unreal's log with NavPyCaptureSlot lines")
        command.add_argument("--emulator", required=True, help="the ZR10 emulator's JSONL log")
        command.add_argument("--out", required=True, help="where the JSON result goes")
        if name == "check":
            command.add_argument("--constants", required=True, help="the fit's JSON")
            command.add_argument("--observation", action="store_true",
                                 help="the held-out 10-minute observation's coverage rules")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "check":
            constants = json.loads(Path(args.constants).read_text(encoding="utf-8"))
            if constants.get("schema") != FIT_SCHEMA or not constants.get("passed"):
                raise TimingInputError(f"{args.constants}: not a passing {FIT_SCHEMA}")
        joined = load(args)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        if isinstance(exc, JoinError):
            result = {"passed": False, "failures": [str(exc)]}
            Path(args.out).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            print(f"FAIL {exc}", flush=True)
            return 1
        print(f"input error: {exc}", file=sys.stderr, flush=True)
        return 2
    try:
        result = fit(joined) if args.command == "fit" else check(joined, constants, args.observation)
    except JoinError as exc:
        result = {"passed": False, "failures": [str(exc)]}
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(("PASS" if result["passed"] else "FAIL") + f" {args.command}: " + "; ".join(result["failures"]), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
