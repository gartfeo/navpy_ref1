"""SCRATCH: multi-segment open-loop plant baseline. No vision, no navigation law.

One flight, many commanded attitudes. Most of a plant run's cost is climbing to
altitude and flying to the gate, so holding a staircase of attitudes in a single
flight buys 8-10 measurements for the price of one climb.

Each segment yields two things:

* the STEADY map -- commanded attitude to achieved pitch, flight-path angle,
  angle of attack, airspeed and sink rate
* the STEP response from the previous segment -- how long pitch and, more
  importantly, GAMMA take to arrive, which is what converts into "how much
  range does a correction consume"

Gamma here comes from GLOBAL_POSITION_INT NED velocity (~32 Hz, part of the
pose-stream request), NOT from VFR_HUD climb rate: VFR_HUD was measured updating
at only 4 Hz, which quantises a ~0.6 s response into 0.25 s steps.

Analysis helpers are imported from `scratch_plant_id_child` rather than copied,
so the two harnesses cannot drift apart. Delete both once the baseline is done.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from navpy.args.conn_args import ConnArgs  # noqa: E402
from navpy.args.logger_args import LoggerArgs  # noqa: E402
from navpy.logger.logger_factory import initialize_logger  # noqa: E402
from navpy.modules.vehicle.flight_mode import FlightMode  # noqa: E402
from navpy.modules.vehicle.pose_streams import request_pose_streams  # noqa: E402
from navpy.modules.vehicle.vehicle_factory import create_vehicle  # noqa: E402

from scratch_plant_id_child import (  # noqa: E402
    COMMAND_RATE_HZ,
    MAX_CLOCK_DRIFT_FRAC,
    MIN_ATTITUDE_RATE_FOR_STEP_HZ,
    OVERSHOOT_FIRST_ORDER_MAX,
    SAMPLE_RATE_HZ,
    PlantTrace,
    _attitude_update_rate_hz,
    _navpy_args,
    _clock_fidelity,
    _clock_stability,
    _wait_for_scoring_interval,
    _wait_for_mode,
)

# Fraction of each segment treated as settled. The leading part is transient.
SEGMENT_TAIL_FRACTION = 0.45
# A segment must hold long enough for the tail to contain real samples.
MIN_SEGMENT_S = 1.5
# Absolute band on the simulator clock rate, as a multiple of the requested
# speedup. Wide because an uncontended SITL asked for 1.0x measures 1.05x here;
# narrow enough that a steadily starved simulator (which drifts 0.0 and so
# passes the drift test) is still caught.
MIN_CLOCK_RATE = 0.90
MAX_CLOCK_RATE = 1.20
# Gamma must come from a stream fast enough to time a ~0.6 s response.
# VFR_HUD, the fallback, was measured at 4 Hz and cannot.
MIN_GAMMA_RATE_HZ = 20.0
# Settling band, as a fraction of the step and as an absolute floor.
#
# t63/t90 are FIRST-crossing times. On a response that overshoots 13-100% the
# first 90% crossing happens on the way UP and the signal then leaves the band
# again, so it is not a completion time and must not be multiplied by a closing
# speed as if it were. `*_settle_s` below is the time after which the signal
# STAYS inside the band for the rest of the segment.
#
# The absolute floor exists because the band must not be tighter than the
# measurement: steady-state pitch scatter was MEASURED at 0.055-0.225 deg
# (per-segment population stdev, A1 and D1 traces), so a 10% band on a 1 deg
# step would be timing noise rather than the response.
SETTLE_BAND_FRACTION = 0.10
SETTLE_BAND_MIN_DEG = 0.20


class Segment:
    __slots__ = ("pitch_deg", "roll_deg", "hold_s", "start_s", "end_s")

    def __init__(self, pitch_deg: float, roll_deg: float, hold_s: float) -> None:
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg
        self.hold_s = hold_s
        self.start_s = 0.0
        self.end_s = 0.0


def _segments(raw: str) -> list[Segment]:
    """Parse `pitch:roll:hold,pitch:roll:hold` into an ordered staircase."""
    segments: list[Segment] = []
    for piece in raw.split(","):
        piece = piece.strip()
        if not piece:
            continue
        parts = piece.split(":")
        if len(parts) != 3:
            raise ValueError(f"segment must be pitch:roll:hold, got {piece!r}")
        pitch, roll, hold = (float(part) for part in parts)
        if hold < MIN_SEGMENT_S:
            raise ValueError(f"segment hold {hold}s below {MIN_SEGMENT_S}s floor")
        segments.append(Segment(pitch, roll, hold))
    if not segments:
        raise ValueError("no segments given")
    return segments


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", required=True)
    parser.add_argument("--sysid", required=True, type=int)
    parser.add_argument("--engage-seq", required=True, type=int, dest='scoring_start_seq')
    parser.add_argument("--timeout", required=True, type=float)
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--engaged", required=True, type=Path, dest='scoring_active')
    parser.add_argument("--segments", required=True)
    parser.add_argument("--throttle", type=float, default=0.55)
    parser.add_argument("--floor-rel-alt-m", type=float, default=60.0)
    parser.add_argument(
        "--check-args",
        action="store_true",
        help="validate every argument, including the navpy parser, then exit",
    )
    return parser


# `_navpy_args` is imported from the plant-ID child, not rewritten here: the
# navpy parser needs -c/-ss/-ll/-lsd/-ut/-udt, and a hand-written version using
# the long names died at flight time with "required: -ss/--source-system".


def _gamma_fast(row: dict) -> float | None:
    """Flight-path angle from NED velocity, at the pose-stream rate.

    Uses GLOBAL_POSITION_INT velocity (~32 Hz, in the pose-stream request), NOT
    VFR_HUD climb rate / ground speed. VFR_HUD is not in that request and was
    MEASURED updating at 4 Hz, which quantises a ~0.6 s response into 0.25 s
    steps -- useless for timing it. Falls back to VFR_HUD only when velocity is
    absent, and `_gamma_source` reports which was used so a coarse run cannot be
    mistaken for a fine one.

    atan2, not asin: the horizontal speed is a leg, not the hypotenuse, so asin
    biases the angle (a true -20 deg descent reads -21.34 deg).
    """
    north, east, down = (
        row.get("vel_n_m_s"), row.get("vel_e_m_s"), row.get("vel_d_m_s")
    )
    if north is not None and east is not None and down is not None:
        horizontal = math.hypot(north, east)
        if horizontal > 0.0:
            return math.degrees(math.atan2(-down, horizontal))
    climb = row.get("climb_rate_mps")
    speed = row.get("ground_speed_mps")
    if climb is None or speed is None or speed <= 0.0:
        return None
    return math.degrees(math.atan2(climb, speed))


def _gamma_source(rows: list[dict]) -> str:
    """Which stream the gamma series came from, and therefore its resolution."""
    if any(row.get("vel_d_m_s") is not None for row in rows):
        return "global_position_int_velocity"
    return "vfr_hud_climb_rate"


def _update_rate_hz(rows: list[dict], key: str) -> float | None:
    """Rate at which a field actually CHANGED, not the rate it was sampled."""
    changes = [
        rows[i]["t_s"]
        for i in range(1, len(rows))
        if rows[i].get(key) is not None and rows[i][key] != rows[i - 1].get(key)
    ]
    if len(changes) < 3:
        return None
    periods = [changes[i + 1] - changes[i] for i in range(len(changes) - 1)]
    median = statistics.median(periods)
    return None if median <= 0.0 else 1.0 / median


def _segment_rows(rows: list[dict], index: int) -> list[dict]:
    """Rows belonging to a segment, by recorded membership rather than by time.

    Segment boundaries are scheduled on the loop clock, but every measurement
    below is timed on the AUTOPILOT clock, and the two do not advance at the
    same rate. Selecting by the recorded index keeps the two concerns apart.
    """
    return [r for r in rows if r.get("segment") == index]


def _aircraft_time(row: dict) -> float | None:
    """ATTITUDE's autopilot clock if available, else the loop clock.

    The physics happens in simulator time, so a response measured against wall
    time is scaled by however far the simulator's clock is from real time --
    measured at 1.05 on this machine even with no contention at all.

    This is the clock for PITCH and ROLL only. Gamma has its own; see
    `_gamma_time`.
    """
    boot = row.get("boot_s")
    return boot if boot is not None else row.get("t_s")


def _gamma_time(row: dict) -> float | None:
    """GLOBAL_POSITION_INT's own autopilot clock, for gamma.

    Gamma is built from `vel_*`, which arrives on GLOBAL_POSITION_INT. ATTITUDE
    is a separate stream with independent arrival phase, so stamping gamma with
    `boot_s` adds up to one frame (~31 ms at 32 Hz) of unmeasured phase error --
    the same order as the pitch-to-gamma lag such numbers get used to claim.

    Both fields are ArduPilot `time_boot_ms`, so they share an origin and pitch
    and gamma times stay directly comparable.

    Falls back to the attitude clock ONLY for traces recorded before
    `pos_boot_s` existed; `_gamma_clock_source` reports which was used so an
    old trace cannot be mistaken for a correctly stamped one.
    """
    position = row.get("pos_boot_s")
    return position if position is not None else _aircraft_time(row)


def _gamma_clock_source(rows: list[dict]) -> str:
    stamped = sum(1 for r in rows if r.get("pos_boot_s") is not None)
    return "global_position_int_boot_ms" if stamped else "attitude_boot_ms"


def _distinct(series: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """One entry per distinct clock value, keeping the first sighting.

    The sample loop runs faster than the message streams, so consecutive rows
    re-read the same message. Keeping repeats would not corrupt a time (the
    stamp is the message's own) but it makes "when did this cross" ambiguous.
    """
    out: list[tuple[float, float]] = []
    for t, value in series:
        if not out or t != out[-1][0]:
            out.append((t, value))
    return out


def _steady(rows: list[dict], segment: Segment, index: int) -> dict[str, object]:
    member = _segment_rows(rows, index)
    if not member:
        member = [r for r in rows if segment.start_s <= r["t_s"] <= segment.end_s]
    stamped = [(r, _aircraft_time(r)) for r in member]
    stamped = [(r, t) for r, t in stamped if t is not None]
    if not stamped:
        tail = []
    else:
        first, last = stamped[0][1], stamped[-1][1]
        tail_from = last - (last - first) * SEGMENT_TAIL_FRACTION
        tail = [r for r, t in stamped if t >= tail_from]

    def mean(key: str) -> float | None:
        values = [r[key] for r in tail if r.get(key) is not None]
        return statistics.fmean(values) if values else None

    gammas = [g for g in (_gamma_fast(r) for r in tail) if g is not None]
    gamma = statistics.fmean(gammas) if gammas else None
    pitch = mean("act_pitch_deg")
    alt = [
        (_aircraft_time(r), r["rel_alt_m"])
        for r in tail
        if r.get("rel_alt_m") is not None and _aircraft_time(r) is not None
    ]
    sink = None
    if len(alt) > 1 and alt[-1][0] > alt[0][0]:
        sink = (alt[0][1] - alt[-1][1]) / (alt[-1][0] - alt[0][0])
    return {
        "cmd_pitch_deg": segment.pitch_deg,
        "cmd_roll_deg": segment.roll_deg,
        "act_pitch_deg": pitch,
        "act_roll_deg": mean("act_roll_deg"),
        "gamma_deg": gamma,
        # pitch = gamma + alpha, so this is the settled angle of attack.
        "implied_aoa_deg": None if pitch is None or gamma is None else pitch - gamma,
        "pitch_tracking_error_deg": (
            None if pitch is None else pitch - segment.pitch_deg
        ),
        "air_speed_mps": mean("air_speed_mps"),
        "ground_speed_mps": mean("ground_speed_mps"),
        "sink_m_s": sink,
        "tail_samples": len(tail),
    }


def _response(
    rows: list[dict],
    segment: Segment,
    index: int,
    previous: dict[str, object] | None,
    settled: dict[str, object],
) -> dict[str, object] | None:
    """Time for pitch and gamma to arrive after the step into this segment.

    All times are AIRCRAFT seconds (autopilot clock), so they can be multiplied
    by a closing speed to get range without a simulator-clock correction.
    """
    if previous is None:
        return None
    member = _segment_rows(rows, index)
    if not member:
        return None
    out: dict[str, object] = {
        "step_pitch_deg": segment.pitch_deg - previous["cmd_pitch_deg"],
        "step_roll_deg": segment.roll_deg - previous["cmd_roll_deg"],
    }
    for label, start, end, extract, clock in (
        ("pitch", previous.get("act_pitch_deg"), settled.get("act_pitch_deg"),
         lambda r: r.get("act_pitch_deg"), _aircraft_time),
        # Gamma is timed on GLOBAL_POSITION_INT's clock, the stream it is built
        # from -- not on ATTITUDE's.
        ("gamma", previous.get("gamma_deg"), settled.get("gamma_deg"),
         _gamma_fast, _gamma_time),
    ):
        if start is None or end is None:
            continue
        span = end - start
        if abs(span) < 0.25:
            # Too small to time: the crossing would be noise, not a response.
            out[f"{label}_span_deg"] = span
            continue
        series = [(clock(r), extract(r)) for r in member]
        series = _distinct([(t, v) for t, v in series if t is not None and v is not None])
        if not series:
            continue
        # Each channel's step reference is its OWN first sample in the segment.
        step_t = series[0][0]
        for frac in (0.63, 0.90):
            threshold = start + frac * span
            for t, value in series:
                if (value <= threshold) if span < 0 else (value >= threshold):
                    out[f"{label}_t{int(frac * 100)}_s"] = t - step_t
                    break
        peak = min(series, key=lambda pair: pair[1] * (1.0 if span < 0 else -1.0))
        out[f"{label}_peak_deg"] = peak[1]
        out[f"{label}_time_to_peak_s"] = peak[0] - step_t
        overshoot = (peak[1] - end) / span
        out[f"{label}_overshoot_frac"] = overshoot
        out[f"{label}_first_order_valid"] = overshoot <= OVERSHOOT_FIRST_ORDER_MAX
        out[f"{label}_span_deg"] = span

        # SETTLING, not first crossing. The response overshoots, so the first
        # 90% crossing happens on the way up and the signal leaves the band
        # again afterwards. This is the time after which it STAYS in band.
        band = max(SETTLE_BAND_FRACTION * abs(span), SETTLE_BAND_MIN_DEG)
        out[f"{label}_settle_band_deg"] = band
        last_outside = None
        for t, value in series:
            if abs(value - end) > band:
                last_outside = t
        if last_outside is None:
            out[f"{label}_settle_s"] = 0.0
            out[f"{label}_settled_in_segment"] = True
        elif last_outside >= series[-1][0]:
            # Still outside the band at the final sample: this segment did not
            # hold long enough to observe settling. Reporting the segment length
            # would understate it, so report nothing and say so.
            out[f"{label}_settle_s"] = None
            out[f"{label}_settled_in_segment"] = False
        else:
            after = [t for t, _ in series if t > last_outside]
            out[f"{label}_settle_s"] = after[0] - step_t
            out[f"{label}_settled_in_segment"] = True
    return out


def _summary(
    trace: PlantTrace,
    segments: list[Segment],
    options: argparse.Namespace,
) -> dict[str, object]:
    rows = trace.rows
    reports: list[dict[str, object]] = []
    previous: dict[str, object] | None = None
    for index, segment in enumerate(segments):
        settled = _steady(rows, segment, index)
        reports.append({
            "index": index,
            "hold_s": segment.hold_s,
            "steady": settled,
            "response": _response(rows, segment, index, previous, settled),
        })
        previous = settled
    return {
        "commanded": {
            "segments": [
                {"pitch_deg": s.pitch_deg, "roll_deg": s.roll_deg, "hold_s": s.hold_s}
                for s in segments
            ],
            "throttle": options.throttle,
        },
        "segment_reports": reports,
    }


def run(options: argparse.Namespace) -> dict[str, object]:
    segments = _segments(options.segments)
    args = _navpy_args(options)
    logger = initialize_logger(LoggerArgs(args), options.sysid)
    vehicle = None
    trace = PlantTrace(options.result.parent / "plant_trace.csv")
    try:
        vehicle = create_vehicle(ConnArgs(args), logger)
        # Same request the navigation runtime makes at startup; without it the
        # autopilot streams ATTITUDE at its slow default on this link and no
        # sub-second response is resolvable.
        request_pose_streams(vehicle, logger)
        print("PLANT_MATRIX_READY", flush=True)
        _wait_for_scoring_interval(vehicle, options.scoring_start_seq, options.timeout)
        if not vehicle.set_mode(FlightMode.GUIDED):
            raise RuntimeError("GUIDED mode request was rejected")
        _wait_for_mode(vehicle, FlightMode.GUIDED, 5.0)
        options.scoring_active.write_text("PLANT_MATRIX_NAV\n", encoding="utf-8")
        print("PLANT_MATRIX_NAV", flush=True)

        # `sim_speedup` is a METHOD, not a property. Every duration below is an
        # AIRCRAFT-time quantity, so wall time must be converted or an
        # accelerated run holds each segment for the wrong number of simulated
        # seconds and reports response times scaled by the speedup.
        speedup = float(vehicle.sim_speedup())
        if not math.isfinite(speedup) or speedup <= 0.0:
            raise RuntimeError(f"unusable sim speedup {speedup!r}")

        elapsed = 0.0
        for segment in segments:
            segment.start_s = elapsed
            elapsed += segment.hold_s
            segment.end_s = elapsed
        total_s = elapsed

        start_s = time.monotonic()
        next_command_s = 0.0
        next_sample_s = 0.0
        aborted: str | None = None
        index = 0
        while True:
            t_s = (time.monotonic() - start_s) * speedup
            if t_s >= total_s:
                break
            while index + 1 < len(segments) and t_s >= segments[index].end_s:
                index += 1
            segment = segments[index]
            if t_s >= next_command_s:
                vehicle.set_attitude(
                    math.radians(segment.roll_deg),
                    math.radians(segment.pitch_deg),
                    yaw=None,
                    thr=options.throttle,
                )
                next_command_s = t_s + 1.0 / COMMAND_RATE_HZ
            if t_s >= next_sample_s:
                row = trace.sample(vehicle, t_s, segment.roll_deg, segment.pitch_deg)
                row["segment"] = index
                alt = row["rel_alt_m"]
                if alt is not None and alt <= options.floor_rel_alt_m:
                    aborted = f"altitude floor {options.floor_rel_alt_m:g} m reached"
                    break
                next_sample_s = t_s + 1.0 / SAMPLE_RATE_HZ
            time.sleep(0.002 / speedup)

        payload = _summary(trace, segments, options)
        payload["aborted"] = aborted
        payload["samples"] = len(trace.rows)
        payload["sim_speedup"] = speedup
        payload["attitude_update_rate_hz"] = _attitude_update_rate_hz(trace.rows)
        payload["clock_fidelity"] = _clock_fidelity(trace.rows, speedup)

        # A run that produced no usable measurement has not passed, however
        # cleanly it exited.
        missing: list[str] = []
        for report in payload["segment_reports"]:
            steady = report["steady"]
            for name in ("act_pitch_deg", "gamma_deg", "implied_aoa_deg"):
                if steady.get(name) is None:
                    missing.append(f"segment{report['index']}.{name}")
        if not trace.rows:
            missing.append("samples")
        rate_hz = payload["attitude_update_rate_hz"]
        if rate_hz is None or rate_hz < MIN_ATTITUDE_RATE_FOR_STEP_HZ:
            missing.append(f"attitude_rate({rate_hz})")
        # The clock needs BOTH tests.
        #
        # Drift alone would accept a simulator starved by a steady factor: its
        # rate never varies, so drift reads 0.0. That still matters even though
        # every measurement above is timed on the autopilot clock, because
        # SEGMENT SCHEDULING runs on the wall-derived loop clock -- a 0.5x
        # simulator holds each segment for half the intended aircraft seconds
        # and burns twice the predicted altitude.
        #
        # The absolute band is NOT centred on 1.0: an uncontended SITL asked
        # for 1.0x steadily runs at 1.05x while `sim_speedup()` reports 1.0, so
        # a tight band around 1.0 rejects every healthy run. It is wide enough
        # to admit that offset and narrow enough to catch real starvation.
        payload["clock_stability"] = _clock_stability(trace.rows)
        stability = payload["clock_stability"]
        if stability is None:
            missing.append("clock_stability")
        else:
            if stability["drift_frac"] > MAX_CLOCK_DRIFT_FRAC:
                missing.append(f"clock_drift({stability['drift_frac']:.4f})")
            rate = stability["rate_mean"] / speedup
            if not (MIN_CLOCK_RATE <= rate <= MAX_CLOCK_RATE):
                missing.append(f"clock_rate({rate:.4f})")

        # The gamma stream decides whether a response time means anything:
        # VFR_HUD updates at 4 Hz, which cannot resolve a ~0.6 s response.
        payload["gamma_source"] = _gamma_source(trace.rows)
        payload["gamma_clock_source"] = _gamma_clock_source(trace.rows)
        payload["gamma_update_rate_hz"] = _update_rate_hz(trace.rows, "vel_d_m_s")
        # Gamma timed on the attitude clock carries up to a frame of unmeasured
        # phase error. That is a defect in the measurement, not a warning.
        if payload["gamma_clock_source"] != "global_position_int_boot_ms":
            missing.append(f"gamma_clock({payload['gamma_clock_source']})")
        gamma_hz = payload["gamma_update_rate_hz"]
        if gamma_hz is None or gamma_hz < MIN_GAMMA_RATE_HZ:
            missing.append(f"gamma_rate({gamma_hz})")

        # A flight exists to collect responses. Passing while every response is
        # absent would make an empty table look like a clean result -- the
        # evaluator prints only the rows it has, so nothing else would notice.
        expected = [
            report for report in payload["segment_reports"]
            if report["index"] > 0
            and abs(
                report["steady"]["cmd_pitch_deg"]
                - payload["segment_reports"][report["index"] - 1]
                ["steady"]["cmd_pitch_deg"]
            ) > 0.0
        ]
        for report in expected:
            response = report.get("response") or {}
            for field in ("pitch_t90_s", "gamma_t90_s"):
                if response.get(field) is None:
                    missing.append(f"segment{report['index']}.{field}")
        payload["missing"] = missing
        payload["passed"] = aborted is None and not missing
        return payload
    finally:
        trace.write()
        if vehicle is not None:
            vehicle.close()
        logger.close()


def main() -> int:
    options = _parser().parse_args()
    if options.check_args:
        # Exercise the SAME parsing the real run does -- segments and the navpy
        # parser both. `--help` short-circuits before required-argument checks,
        # so a probe built on it passes commands that die at flight time.
        _segments(options.segments)
        _navpy_args(options)
        return 0
    options.result.parent.mkdir(parents=True, exist_ok=True)
    payload = run(options)
    options.result.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0 if payload.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
