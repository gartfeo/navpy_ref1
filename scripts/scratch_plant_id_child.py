"""SCRATCH: open-loop plant identification. No vision, no navigation law.

Commands a FIXED attitude and records what the airframe actually does, so the
plant can be characterised independently of any navigation question:

* the steady flight-path angle a commanded pitch produces (the pitch -> gamma
  map, which is what the navigation law is implicitly assuming)
* the angle of attack implied by that map, and whether it stays constant as
  airspeed builds
* the closed-loop pitch response time constant, measured from a real step
  rather than fitted over an arbitrary window of a guided run
* what a commanded bank does to pitch hold, sink rate and airspeed

The run is a step: hold LEVEL for `--level-s`, then step to the commanded
attitude and hold. The step is what makes the time constant measurable; a run
that starts already at the target attitude cannot give one.

Delete with the other scratch harnesses once plant ID is closed out.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKTREE / "src"))

from navpy.args.conn_args import ConnArgs  # noqa: E402
from navpy.args.logger_args import LoggerArgs  # noqa: E402
from navpy.args.navpy_argparse import make_parser  # noqa: E402
from navpy.logger.logger_factory import initialize_logger  # noqa: E402
from navpy.modules.vehicle.flight_mode import FlightMode  # noqa: E402
from navpy.modules.vehicle.message_store import MessageStore  # noqa: E402
from navpy.modules.vehicle.pose_streams import request_pose_streams  # noqa: E402
from navpy.modules.vehicle.pose_telemetry import (  # noqa: E402
    attitude_sample_from_message,
)
from navpy.modules.vehicle.vehicle_factory import create_vehicle  # noqa: E402
from navpy.modules.vehicle.vehicle_interface import IVehicle  # noqa: E402

COMMAND_RATE_HZ = 20.0
SAMPLE_RATE_HZ = 50.0
# The flight-path angle is differentiated from position, so it needs a baseline
# long enough to be quiet but short enough to still be local.
GAMMA_WINDOW_S = 1.0
# Steady state is read from the last part of the hold, after the step transient.
STEADY_TAIL_FRACTION = 0.4
# The autopilot's DEFAULT ATTITUDE rate on a fresh link is a few Hz, which is
# far too coarse to resolve a sub-second pitch step. The navigation runtime does
# not hit this because it calls `request_pose_streams` at startup; this harness
# must make the same request or it measures the telemetry rate, not the plant.
MIN_ATTITUDE_RATE_FOR_STEP_HZ = 20.0
# How much the simulator clock rate may VARY within a run.
#
# Deliberately not a band around 1.0. Measured on this machine: an uncontended
# SITL asked for 1.0x steadily advances its clock at 1.05x while
# `sim_speedup()` still reports 1.0, so an absolute band rejects every healthy
# run. A steady offset is harmless as long as timings are read off the
# autopilot clock; what CPU contention produces is a clock that WANDERS, and
# that is what this gate catches.
MAX_CLOCK_DRIFT_FRAC = 0.05


def _clock_stability(rows: list[dict]) -> dict[str, object] | None:
    """Spread of the simulator clock rate across quarters of the run."""
    pairs: list[tuple[float, float]] = []
    previous: float | None = None
    for row in rows:
        boot, loop = row.get("boot_s"), row.get("t_s")
        if boot is None or loop is None or boot == previous:
            continue
        pairs.append((loop, boot))
        previous = boot
    if len(pairs) < 40:
        return None
    quarter = len(pairs) // 4
    rates: list[float] = []
    for part in range(4):
        chunk = pairs[part * quarter:(part + 1) * quarter]
        if len(chunk) < 2:
            continue
        loop_span = chunk[-1][0] - chunk[0][0]
        if loop_span <= 0.0:
            continue
        rates.append((chunk[-1][1] - chunk[0][1]) / loop_span)
    if len(rates) < 3:
        return None
    mean_rate = statistics.fmean(rates)
    if mean_rate <= 0.0:
        return None
    return {
        "rate_mean": mean_rate,
        "rate_min": min(rates),
        "rate_max": max(rates),
        "drift_frac": (max(rates) - min(rates)) / mean_rate,
        "quarters": rates,
    }
# A true first-order response cannot overshoot at all, so any real overshoot
# means the 63.2% crossing is not a plant time constant and must never be
# reported as one. The bound is small and non-zero only to absorb measurement
# noise on the settled value, not to tolerate actual overshoot.
OVERSHOOT_FIRST_ORDER_MAX = 0.05


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", required=True)
    parser.add_argument("--sysid", required=True, type=int)
    parser.add_argument("--scoring-start-seq", required=True, type=int, dest='scoring_start_seq')
    parser.add_argument("--timeout", required=True, type=float)
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--scoring-active", required=True, type=Path, dest='scoring_active')
    parser.add_argument("--pitch-deg", required=True, type=float)
    parser.add_argument("--roll-deg", default=0.0, type=float)
    parser.add_argument("--throttle", default=0.55, type=float)
    parser.add_argument("--level-s", default=3.0, type=float)
    parser.add_argument("--hold-s", default=20.0, type=float)
    parser.add_argument(
        "--floor-rel-alt-m",
        default=60.0,
        type=float,
        help="abort the hold above this relative altitude, so a steep command "
             "cannot fly the aircraft into the ground",
    )
    return parser


def _navpy_args(options: argparse.Namespace) -> argparse.Namespace:
    return make_parser(description="plant id child").parse_args([
        "-c", options.connection,
        "-ss", str(options.sysid),
        "-ll", "DEBUG",
        "-lsd", "Vehicle",
        "-ut", "false",
        "-udt", "false",
    ])


def _wait_for_scoring_interval(vehicle: IVehicle, scoring_start_seq: int, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if vehicle.is_armed and (
            vehicle.mission_items_next is not None
            and vehicle.mission_items_next >= scoring_start_seq
        ):
            return
        time.sleep(0.02)
    raise TimeoutError(f"mission did not reach scoring start sequence {scoring_start_seq}")


def _wait_for_mode(vehicle: IVehicle, mode: FlightMode, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if vehicle.get_mode is mode:
            return
        time.sleep(0.02)
    raise TimeoutError(f"vehicle did not enter {mode.value}")


def _message_store(vehicle: IVehicle) -> MessageStore:
    """The MAVLink message store behind the public vehicle facets.

    Needed because every public accessor performs its OWN `latest()` call, and
    two accessors therefore cannot be read as one consistent observation. See
    `_position_reading` for why that matters.

    Raises rather than returning None if the chain ever changes: falling back to
    a mismatched clock is precisely the defect this exists to remove, and it
    must not happen silently.
    """
    parts = getattr(vehicle, "_parts", None)
    pose = getattr(parts, "pose", None)
    store = getattr(pose, "_messages", None)
    if store is None:
        raise AttributeError(
            "cannot reach the MAVLink message store via vehicle._parts.pose."
            "_messages; gamma cannot be timed on its own clock"
        )
    return store


def _position_reading(
    store: MessageStore,
) -> tuple[tuple[float, float, float] | None, float | None]:
    """NED velocity and its OWN timestamp, from ONE GLOBAL_POSITION_INT sample.

    `MessageStore.latest` is individually atomic (message_store.py:73-75), but
    there is NO atomicity ACROSS two calls. Reading `vehicle.velocity`
    (flight_telemetry.py:17-21, its own `message()` call) and then separately
    reading the sample for its timestamp lets the reader thread publish a new
    GLOBAL_POSITION_INT in between, pairing one sample's velocity with a
    different sample's `time_boot_ms`. The window is short, so it would occur
    rarely -- which makes it worse, not better: an intermittent one-frame
    (~31 ms at 32 Hz) error in scattered rows is exactly the phase artifact the
    per-stream clock was added to remove, and it would be invisible.

    One `latest()` call, both values from that single `MessageSample`.

    vx/vy/vz are cm/s, matching `FlightTelemetry.velocity`'s /100.0.
    """
    sample = store.latest("GLOBAL_POSITION_INT")
    message = None if sample is None else sample.message
    if message is None:
        return None, None
    velocity = (message.vx / 100.0, message.vy / 100.0, message.vz / 100.0)
    boot_ms = getattr(message, "time_boot_ms", None)
    return velocity, (None if boot_ms is None else boot_ms / 1000.0)


class PlantTrace:
    """Diagnostics-side recorder. Nothing here feeds a command."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self.rows: list[dict[str, float | None]] = []

    def sample(
        self, vehicle: IVehicle, t_s: float, cmd_roll: float, cmd_pitch: float
    ) -> dict[str, float | None]:
        # ONE store read per stream. `vehicle.attitude` and
        # `vehicle.attitude_sample` are two independent `latest()` calls, so
        # reading both would pair one ATTITUDE message's angles with another
        # one's `time_boot_ms` whenever the reader thread publishes between
        # them -- the same defect as the position pairing below. The sample
        # already carries the angles, so take only the sample.
        store = _message_store(vehicle)
        sample = store.latest("ATTITUDE")
        sample = None if sample is None else attitude_sample_from_message(
            sample.message, sample.receipt_time_s
        )
        attitude = None if sample is None else sample.attitude
        # `location` REQUIRES is_relative; calling it without the argument
        # raises TypeError, and a broad except would turn that into a silent
        # None that disables the altitude floor and every gamma result.
        location = vehicle.location(True)
        # `sample.time_boot_s` is the autopilot's OWN clock next to our wall
        # clock. Their ratio is the only way to tell a starved simulator from a
        # healthy one: a CPU-contended SITL still integrates its physics
        # correctly in SIM time, so every attitude/gamma number looks plausible
        # while the measured "seconds" quietly stop meaning seconds.
        #
        # NED velocity comes from GLOBAL_POSITION_INT, which the pose-stream
        # request raises to ~32 Hz, WITH that message's own timestamp.
        # `climb_rate`/`ground_speed` below come from VFR_HUD instead, which is
        # NOT in that request and updates at only 4 Hz -- far too coarse to
        # time a response that completes in ~0.6 s.
        velocity, position_boot_s = _position_reading(store)
        row = {
            "t_s": t_s,
            "cmd_roll_deg": cmd_roll,
            "cmd_pitch_deg": cmd_pitch,
            "act_roll_deg": getattr(attitude, "roll", None),
            "act_pitch_deg": getattr(attitude, "pitch", None),
            "air_speed_mps": getattr(vehicle, "air_speed", None),
            "ground_speed_mps": getattr(vehicle, "ground_speed", None),
            "climb_rate_mps": getattr(vehicle, "climb_rate", None),
            "rel_alt_m": getattr(location, "alt", None),
            "vel_n_m_s": None if velocity is None else velocity[0],
            "vel_e_m_s": None if velocity is None else velocity[1],
            "vel_d_m_s": None if velocity is None else velocity[2],
            "boot_s": getattr(sample, "time_boot_s", None),
            "receipt_s": getattr(sample, "receipt_time_s", None),
            # Gamma's own clock, from the SAME sample as vel_* above.
            # Everything derived from vel_* must be timed on this, never on
            # `boot_s`.
            "pos_boot_s": position_boot_s,
            "pitch_rate_deg_s": (
                None if sample is None or sample.body_rates_rad_s is None
                else math.degrees(sample.body_rates_rad_s[1])
            ),
        }
        self.rows.append(row)
        return row

    def write(self) -> None:
        if not self.rows:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        with self._path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)


def _gamma_series(rows: list[dict]) -> list[tuple[float, float]]:
    """Flight-path angle from altitude rate and HORIZONTAL ground speed.

    `ground_speed` is VFR_HUD groundspeed, which is horizontal only, and the
    climb rate is orthogonal to it -- so the angle is atan2(climb, ground),
    not asin(climb / ground).  asin treats the horizontal component as the
    hypotenuse and biases the result: a true -20 deg descent reads -21.34 deg.
    """
    series: list[tuple[float, float]] = []
    for index, row in enumerate(rows):
        t = row["t_s"]
        alt = row["rel_alt_m"]
        speed = row["ground_speed_mps"]
        if alt is None or speed is None or speed <= 0.0:
            continue
        later = None
        for candidate in rows[index + 1:]:
            if candidate["rel_alt_m"] is None:
                continue
            if candidate["t_s"] - t >= GAMMA_WINDOW_S:
                later = candidate
                break
        if later is None:
            continue
        dt = later["t_s"] - t
        climb_rate = (later["rel_alt_m"] - alt) / dt
        series.append((t, math.degrees(math.atan2(climb_rate, speed))))
    return series


def _clock_fidelity(rows: list[dict], speedup: float) -> dict | None:
    """Autopilot sim-clock advance against the intended aircraft-time elapsed.

    1.0 means the simulator kept the clock it was asked for. Below 1.0 it is
    falling behind -- which is what CPU contention from parallel instances
    looks like, and it silently rescales every response time this harness
    reports. Measured, never assumed.

    Timed against `t_s`, which the run loop derives from `time.monotonic()` and
    has already scaled by the speedup, so this ratio is the fidelity directly.
    It deliberately does NOT use `receipt_s`: those come from `time.time()`
    (`inbound_router.py:96`), which an NTP correction or a manual clock change
    can step backwards mid-run, either failing a healthy run or hiding real
    starvation.
    """
    # First sighting of each NEW autopilot timestamp. The sample loop re-reads
    # one cached attitude many times, and those repeats carry no clock news.
    seen: list[tuple[float, float]] = []
    previous_boot: float | None = None
    for row in rows:
        boot = row.get("boot_s")
        elapsed = row.get("t_s")
        if boot is None or elapsed is None or boot == previous_boot:
            continue
        seen.append((elapsed, boot))
        previous_boot = boot
    if len(seen) < 20:
        return None
    intended_span = seen[-1][0] - seen[0][0]
    boot_span = seen[-1][1] - seen[0][1]
    if intended_span <= 0.0:
        return None
    return {
        "ratio": boot_span / intended_span,
        "intended_aircraft_s": intended_span,
        "boot_span_s": boot_span,
        "speedup": speedup,
        "samples": len(seen),
    }


def _attitude_update_rate_hz(rows: list[dict]) -> float | None:
    """Rate at which ATTITUDE actually CHANGED, not the rate we sampled it.

    The sample loop reads a cached attitude, so it happily logs the same value
    many times over. The time constant can only be resolved to one update
    period, so the update rate -- not `SAMPLE_RATE_HZ` -- is the real
    resolution limit and must be reported alongside every step number.
    """
    changes = [
        rows[index]["t_s"]
        for index in range(1, len(rows))
        if rows[index]["act_pitch_deg"] is not None
        and rows[index]["act_pitch_deg"] != rows[index - 1]["act_pitch_deg"]
    ]
    if len(changes) < 3:
        return None
    periods = [changes[i + 1] - changes[i] for i in range(len(changes) - 1)]
    median = statistics.median(periods)
    return None if median <= 0.0 else 1.0 / median


def _time_constant(
    rows: list[dict], step_t_s: float, target_deg: float
) -> dict[str, float | bool | None] | None:
    """Pitch step response: 63.2% crossing, overshoot, and time to peak.

    The crossing is a first-order reading, so it is only meaningful if the
    response is actually first-order. `overshoot_frac` and `time_to_peak_s`
    are reported next to it precisely so that can be judged: a response that
    overshoots is second order and its `tau_s` must not be quoted as a plant
    time constant. `resolution_s` bounds the crossing by one attitude update
    period.
    """
    before = [r for r in rows if r["t_s"] < step_t_s and r["act_pitch_deg"] is not None]
    after = [r for r in rows if r["t_s"] >= step_t_s and r["act_pitch_deg"] is not None]
    if not before or not after:
        return None
    start = statistics.fmean(r["act_pitch_deg"] for r in before[-10:])
    tail = after[int(len(after) * (1.0 - STEADY_TAIL_FRACTION)):]
    if not tail:
        return None
    settled = statistics.fmean(r["act_pitch_deg"] for r in tail)
    span = settled - start
    if abs(span) < 1.0:
        return None
    peak = min(after, key=lambda r: r["act_pitch_deg"] * (1.0 if span < 0 else -1.0))
    overshoot_frac = (peak["act_pitch_deg"] - settled) / span
    update_rate_hz = _attitude_update_rate_hz(rows)
    threshold = start + 0.632 * span
    for row in after:
        reached = (
            row["act_pitch_deg"] >= threshold
            if span > 0
            else row["act_pitch_deg"] <= threshold
        )
        if reached:
            return {
                "tau_s": row["t_s"] - step_t_s,
                "resolution_s": (
                    None if update_rate_hz is None else 1.0 / update_rate_hz
                ),
                "start_pitch_deg": start,
                "settled_pitch_deg": settled,
                "commanded_pitch_deg": target_deg,
                "peak_pitch_deg": peak["act_pitch_deg"],
                "time_to_peak_s": peak["t_s"] - step_t_s,
                "overshoot_frac": overshoot_frac,
                # Carried in the result so a reader of the JSON alone cannot
                # mistake `tau_s` for a plant time constant.
                "first_order_valid": overshoot_frac <= OVERSHOOT_FIRST_ORDER_MAX,
            }
    return None


def _summary(
    trace: PlantTrace, options: argparse.Namespace, step_t_s: float
) -> dict[str, object]:
    rows = trace.rows
    gamma = _gamma_series(rows)
    steady_from = step_t_s + options.hold_s * (1.0 - STEADY_TAIL_FRACTION)
    steady = [r for r in rows if r["t_s"] >= steady_from]
    steady_gamma = [g for t, g in gamma if t >= steady_from]

    def _mean(key: str) -> float | None:
        values = [r[key] for r in steady if r[key] is not None]
        return statistics.fmean(values) if values else None

    pitch = _mean("act_pitch_deg")
    gamma_mean = statistics.fmean(steady_gamma) if steady_gamma else None
    return {
        "commanded": {
            "pitch_deg": options.pitch_deg,
            "roll_deg": options.roll_deg,
            "throttle": options.throttle,
        },
        "steady": {
            "act_pitch_deg": pitch,
            "act_roll_deg": _mean("act_roll_deg"),
            "air_speed_mps": _mean("air_speed_mps"),
            "ground_speed_mps": _mean("ground_speed_mps"),
            "gamma_deg": gamma_mean,
            # pitch = gamma + alpha, so this is the angle of attack the
            # airframe settled at for this commanded attitude.
            "implied_aoa_deg": (
                None if pitch is None or gamma_mean is None else pitch - gamma_mean
            ),
            "pitch_tracking_error_deg": (
                None if pitch is None else pitch - options.pitch_deg
            ),
            "sample_count": len(steady),
        },
        "step_response": _time_constant(rows, step_t_s, options.pitch_deg),
    }


def run(options: argparse.Namespace) -> dict[str, object]:
    args = _navpy_args(options)
    logger = initialize_logger(LoggerArgs(args), options.sysid)
    vehicle = None
    trace = PlantTrace(options.result.parent / "plant_trace.csv")
    try:
        vehicle = create_vehicle(ConnArgs(args), logger)
        # Same request the navigation runtime makes at startup. Without it the
        # autopilot streams ATTITUDE at its slow default on this link and the
        # step response is quantised to that period.
        request_pose_streams(vehicle, logger)
        print("PLANT_ID_READY", flush=True)
        _wait_for_scoring_interval(vehicle, options.scoring_start_seq, options.timeout)
        if not vehicle.set_mode(FlightMode.GUIDED):
            raise RuntimeError("GUIDED mode request was rejected")
        _wait_for_mode(vehicle, FlightMode.GUIDED, 5.0)
        options.scoring_active.write_text("PLANT_ID_NAV\n", encoding="utf-8")
        print("PLANT_ID_NAV", flush=True)

        # SITL can run accelerated. Every duration here -- the hold, the
        # command cadence, the sample cadence, and above all the measured
        # time constant -- is an AIRCRAFT-time quantity, so wall time must be
        # converted or a 10x run would report tau at a tenth of its true value
        # and hold for 200 simulated seconds instead of 20.
        # `sim_speedup` is a METHOD, not a property -- the other children pass
        # the callable itself to SchedulerCadence.  Reading it as an attribute
        # yields a bound method and float() raises.
        speedup = float(vehicle.sim_speedup())
        if not math.isfinite(speedup) or speedup <= 0.0:
            raise RuntimeError(f"unusable sim speedup {speedup!r}")
        start_s = time.monotonic()
        step_t_s = options.level_s
        end_s = options.level_s + options.hold_s
        next_command_s = 0.0
        next_sample_s = 0.0
        aborted: str | None = None
        while True:
            t_s = (time.monotonic() - start_s) * speedup
            if t_s >= end_s:
                break
            stepped = t_s >= step_t_s
            cmd_roll = options.roll_deg if stepped else 0.0
            cmd_pitch = options.pitch_deg if stepped else 0.0
            if t_s >= next_command_s:
                vehicle.set_attitude(
                    math.radians(cmd_roll),
                    math.radians(cmd_pitch),
                    yaw=None,
                    thr=options.throttle,
                )
                next_command_s = t_s + 1.0 / COMMAND_RATE_HZ
            if t_s >= next_sample_s:
                row = trace.sample(vehicle, t_s, cmd_roll, cmd_pitch)
                alt = row["rel_alt_m"]
                if alt is not None and alt <= options.floor_rel_alt_m:
                    aborted = f"altitude floor {options.floor_rel_alt_m:g} m reached"
                    break
                next_sample_s = t_s + 1.0 / SAMPLE_RATE_HZ
            time.sleep(0.002 / speedup)

        payload = _summary(trace, options, step_t_s)
        payload["aborted"] = aborted
        payload["samples"] = len(trace.rows)
        payload["sim_speedup"] = speedup
        payload["attitude_update_rate_hz"] = _attitude_update_rate_hz(trace.rows)
        payload["clock_fidelity"] = _clock_fidelity(trace.rows, speedup)
        # A plant-ID run that produced no measurements has not passed, however
        # cleanly it exited. Reporting success on all-null metrics would let a
        # silent telemetry failure look like a result.
        missing = [
            name
            for name in ("act_pitch_deg", "gamma_deg", "implied_aoa_deg")
            if payload["steady"].get(name) is None
        ]
        if payload["step_response"] is None:
            missing.append("step_response")
        if not trace.rows:
            missing.append("samples")
        # A step measured against a slow attitude stream cannot resolve the
        # time constant it exists to measure, so it is a missing measurement,
        # not a present one with a wide error bar.
        rate_hz = payload["attitude_update_rate_hz"]
        if rate_hz is None or rate_hz < MIN_ATTITUDE_RATE_FOR_STEP_HZ:
            missing.append(f"attitude_rate({rate_hz})")
        # A run whose simulator clock WANDERED is not a slightly worse
        # measurement, it is several measurements of different aircraft-seconds
        # stitched together. Refuse it rather than average it in with healthy
        # runs. A steady offset is fine and is reported, not gated.
        payload["clock_stability"] = _clock_stability(trace.rows)
        stability = payload["clock_stability"]
        if stability is None:
            missing.append("clock_stability")
        elif stability["drift_frac"] > MAX_CLOCK_DRIFT_FRAC:
            missing.append(f"clock_drift({stability['drift_frac']:.4f})")
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
    try:
        payload = run(options)
    except BaseException as error:  # noqa: BLE001 - the parent reads the payload
        payload = {"passed": False, "error": f"{type(error).__name__}: {error}"}
    options.result.parent.mkdir(parents=True, exist_ok=True)
    options.result.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("PLANT_ID_RESULT " + json.dumps(payload), flush=True)
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
