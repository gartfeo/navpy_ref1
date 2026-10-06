"""Simulator-truth pose stream for isolated navigation testing on SITL.

WHY TRUTH AND NOT THE RECEIVED ESTIMATE
---------------------------------------
A synthetic camera has to be told where the POI appears, and that means
rotating the POI vector from NED into the body frame, which needs a yaw.
Taking that yaw from the ATTITUDE message is wrong here: ATTITUDE carries the
vehicle's ESTIMATE, and its yaw is compass-derived. Compass yaw can hold a large
bias, and this project forbids it in the final-approach command path precisely because
that bias would be indistinguishable from a navigation error. Feeding a
compass-biased yaw into the sensor rotates the synthesized line of sight by that
same bias, so the contamination survives into the ray even though the law never
reads yaw itself.

SIM_STATE is the simulator's own truth: the attitude the aircraft actually has,
not the attitude it believes it has. Building the ray from truth is what the
existing accepted ideal sensor does -- see the docstring at
scripts/vision_static_point_mass_sensor.py:97, "Use simulator truth only to
synthesize one frame-local pixel", fed from plant state rather than from any
estimator. This module is the SITL equivalent of that plant state.

The distinction is not academic. Holding the POI fixed and changing yaw alone
by 15 degrees moves the synthesized pixel by hundreds of pixels, so a compass
bias of a few degrees is not a rounding difference in the ray.

WHY TWO STREAMS AND A SKEW GATE
-------------------------------
SIM_STATE has no timestamp field (see MAVLink message 108: attitude, rates,
lat/lon/alt, but no time_boot_ms or time_usec). It cannot be its own clock. The
autopilot's clock arrives on ATTITUDE as time_boot_ms, which is also the only
time base that is immune to host stalls, since it advances with the simulated
aircraft rather than with the wall clock.

So a usable pose needs one sample from each stream, and the two are only one
pose if they are close together in time. At ArduPilot's default rates the gap is
brutal: measured over 37952 real sample intervals, one 250 ms attitude gap moves
the aircraft's attitude by 1.69 degrees at the 90th percentile and 17.3 degrees
at the 99th. At 300 m range the 90th percentile alone is 8.8 m of aim error --
larger than the approach error the law is being judged on. Both streams therefore
have to be raised, and the pairing skew has to be enforced rather than assumed.

WHY AN OBSERVED-CADENCE GATE AND NOT JUST THE ACK
-------------------------------------------------
MAV_CMD_SET_MESSAGE_INTERVAL returning MAV_RESULT_ACCEPTED means the request was
accepted, not that the data arrived. ArduPilot caps scheduled MAVLink rates, and
runtime load can delay emission regardless of the cap. A run that silently
degrades to a few Hz still produces plausible-looking numbers, which is the
worst possible failure: wrong, but not obviously wrong. This module therefore
records what actually arrived and refuses to certify a run whose delivered
cadence did not hold up. The repository already draws this distinction for
scoring -- an accepted request in
scripts/eval_navigation_telemetry.py:113 and a separate observed source-time gate
in scripts/eval_navigation_scoring.py:246 -- and this applies the same rule to the
pose feed that drives the navigation loop.
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass
from typing import Protocol

from navpy.modules.vehicle.pose_streams import (
    ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION,
    POSE_STREAM_RATE_HZ,
    pose_frame_association_max_skew_s,
)
from navpy.modules.vehicle.pose_telemetry import sim_state_coordinates_deg


# pymavlink ships no type stubs, so the objects it hands back are described by
# the SHAPE this module actually uses rather than by their concrete classes.
# Same approach as `pose_streams.py:43-56`, which does this for the same reason.
class MavlinkMessage(Protocol):
    def get_type(self) -> str: ...


class MavlinkLink(Protocol):
    @property
    def mav(self) -> object: ...

    def recv_match(self, **criteria: object) -> object: ...


class ParameterReader(Protocol):
    def __call__(self, sysid: int, name: str) -> float | None: ...

# The fastest rate ArduPilot will actually grant, not the fastest we would like.
#
# ArduPilot caps MAVLink messages at 0.8 * SCHED_LOOP_RATE and, from 4.6,
# REJECTS a faster SET_MESSAGE_INTERVAL rather than clamping it
# (pose_streams.py:24-27). An earlier version here asked for a round 50 Hz
# against Plane's 50 Hz scheduler, which is 40 Hz achievable -- so both streams
# came back DENIED and the aircraft kept its default 4 Hz. Every pose was then
# rejected for skew, and the run reached the hung-run guard having never built a
# single line of sight.
#
# Taken from the same module the flight code uses, so the two cannot drift.
TRUTH_POSE_RATE_HZ = POSE_STREAM_RATE_HZ

# A pose is one sample from each of two streams, so it is only meaningful if
# they nearly coincide. Two stream periods, which is the bound the flight code
# already uses for exactly this question
# (pose_streams.py:POSE_FRAME_ASSOCIATION_MAX_PERIODS): one period because a
# sample may land anywhere inside its slot, and a second for link and scheduler
# jitter. At the granted rate that is 50 ms -- a fifth of the 250 ms staleness
# the module docstring is about, and it still rejects a genuine stall.
MAX_POSE_SKEW_S = pose_frame_association_max_skew_s(TRUTH_POSE_RATE_HZ)

# Delivered cadence has to stay near what was requested. A run is rejected if
# any gap exceeds this, which catches a stream that was never actually raised as
# well as one that stalled part way through.
#
# Expressed in PERIODS, not seconds, because the granted rate is not a constant:
# it is 0.8 * SCHED_LOOP_RATE, and SCHED_LOOP_RATE is 50 on this SITL Plane but
# 200 by default on a Cube 6X and up to 400. A fixed 100 ms gate would be four
# missed samples at 40 Hz and sixteen at 160 Hz -- the same number meaning two
# very different standards, and the faster the aircraft the weaker the gate.
# Four periods is three consecutive poses lost.
POSE_GAP_PERIODS = 4.0


def pose_gap_limit_s(rate_hz: float) -> float:
    """Largest tolerable hole in the pose feed, at this feed's own rate."""
    return POSE_GAP_PERIODS / rate_hz


MAX_POSE_GAP_S = pose_gap_limit_s(TRUTH_POSE_RATE_HZ)

# Below this there is not enough evidence that the stream ever ran properly,
# independent of the gaps between the samples that did arrive.
MIN_POSE_SAMPLES = 20


@dataclass(frozen=True)
class TruthPose:
    """One simulator-truth pose, timestamped on the autopilot's own clock.

    `t_s` comes from ATTITUDE.time_boot_ms and is SIMULATED seconds: it advances
    with the aircraft, not with the host, so it does not move when the host
    stalls or when the simulation is sped up.

    Everything else is truth from SIM_STATE. `yaw_deg` is present because a
    synthetic sensor cannot resolve a direction into the body frame without it;
    it must not be handed to the navigation law.
    """

    t_s: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    skew_s: float
    # ESTIMATED aircraft state, from ATTITUDE -- what the aircraft believes,
    # not what is true. Kept separate from the truth fields above because they
    # are for different consumers and must not be mixed up: truth answers "where
    # is the POI relative to the nose", which stands in for a camera, while
    # these are what the navigation law is allowed to read about itself.
    #
    # A real aircraft has no truth. Handing the law a perfect pitch or a perfect
    # body rate would flatter it with information the flying article never has,
    # and the isolated test would then pass for a reason that does not transfer.
    # Estimated roll and pitch are safe to use: unlike yaw they are not
    # compass-referenced, so they carry no compass bias.
    est_roll_deg: float
    est_pitch_deg: float
    est_roll_rate_rad_s: float
    est_pitch_rate_rad_s: float
    est_yaw_rate_rad_s: float


class TruthPoseStream:
    """Pairs SIM_STATE truth with the autopilot clock, and audits the cadence.

    Feed it every message; it keeps the most recent of each kind and emits a
    pose when a fresh pair is available within the skew limit. Nothing here
    interpolates: an invented sample between two real ones would be a guess
    about the very motion the pose is supposed to measure.
    """

    def __init__(
        self,
        *,
        max_skew_s: float = MAX_POSE_SKEW_S,
        max_gap_s: float = MAX_POSE_GAP_S,
        min_samples: int = MIN_POSE_SAMPLES,
    ) -> None:
        self._max_skew_s = max_skew_s
        self._max_gap_s = max_gap_s
        self._min_samples = min_samples
        self._truth: tuple[float, ...] | None = None
        self._truth_arrival_s: float | None = None
        self._degE7_fields = 0
        self._position_samples = 0
        self._arrival_offsets: list[float] = []
        self._estimate: tuple[float, ...] | None = None
        self._clock_t_s: float | None = None
        self._last_emitted_t_s: float | None = None
        self._gaps: list[float] = []
        self._skews: list[float] = []
        self._rejected_skew = 0
        self._poses = 0

    def absorb(self, message: MavlinkMessage) -> TruthPose | None:
        """Take one MAVLink message; return a pose when a fresh pair completes.

        SIM_STATE carries no clock, so its time cannot be measured -- only
        BOUNDED. A truth sample that arrives between two ATTITUDE messages
        happened somewhere in that interval, so stamping it with the later
        ATTITUDE is wrong by at most the length of that interval. That bound is
        the skew, and it is derived purely from the autopilot's own clock, so no
        host timing enters it.

        An earlier version stamped SIM_STATE with the most recent ATTITUDE time
        at the moment it arrived, which made the skew identically zero and left
        the gate unable to reject anything. Bounding is what makes it real.
        """
        kind = message.get_type()
        if kind == "SIM_STATE":
            # Held until an ATTITUDE closes the interval. If several arrive
            # first, the last one is nearest that ATTITUDE and wins.
            #
            # SIM_STATE mixes its units: attitude is RADIANS but lat/lon are
            # DEGREES, and there are separate degE7 lat_int/lon_int fields that
            # some builds populate instead. Converting all four the same way put
            # the aircraft at latitude 2291 degrees. sim_state_coordinates_deg
            # already resolves that -- including the degE7 variant and a range
            # check -- so the coordinates go through it rather than through a
            # second, private guess about the encoding.
            coordinates = sim_state_coordinates_deg(message)
            if coordinates is None:
                return None
            # WHICH DECODE PATH the position came through, recorded because the
            # two differ by a third of a metre and nothing else would say.
            # `sim_state_coordinates_deg` prefers the degE7 lat_int/lon_int
            # fields and falls back to SIM_STATE's float32 lat/lon when they are
            # absent -- a MAVLink1 link or an older firmware is enough. Float32
            # at this latitude quantises to 0.42 m north / 0.32 m east, which
            # would manufacture miss differences of 0.05-0.17 m: the same size
            # as the effects this harness exists to measure.
            #
            # The RAW EVIDENCE is recorded, not a second copy of the decision.
            # Re-deriving "was it degE7?" here would be a private duplicate of
            # production's rule and would drift from it.
            self._degE7_fields += bool(
                getattr(message, "lat_int", 0) or getattr(message, "lon_int", 0))
            self._position_samples += 1
            self._truth = (
                coordinates[0], coordinates[1], float(message.alt),
                float(message.roll), float(message.pitch), float(message.yaw),
            )
            # DIAGNOSTIC ONLY. Host arrival, never a pose field and never read
            # by anything that produces a command -- the whole point of this
            # module is that decisions run on the autopilot's clock.
            #
            # It exists because the offset between the two streams is the one
            # thing this class admits it cannot measure, and a delay sweep found
            # the navigation miss improving 8x when the de-rotating attitude was
            # held back ~75 ms. Bounding the offset at one ATTITUDE period was
            # evidently wrong; this records how far apart the two messages
            # ACTUALLY arrive so the bound can be replaced by a measurement.
            self._truth_arrival_s = time.monotonic()
            return None
        if kind != "ATTITUDE":
            return None
        previous_clock = self._clock_t_s
        self._clock_t_s = int(message.time_boot_ms) / 1000.0
        if self._truth_arrival_s is not None:
            self._arrival_offsets.append(
                time.monotonic() - self._truth_arrival_s)
            # CONSUMED. Only the FIRST ATTITUDE after a truth sample closes its
            # interval; leaving this set let every later ATTITUDE measure itself
            # against the same stale SIM_STATE, and since the streams are not
            # guaranteed to alternate (see
            # test_starved_truth_stream_is_rejected_though_each_pose_is_valid)
            # the extra samples are pure inflation of the statistic this exists
            # to measure.
            self._truth_arrival_s = None
        # The aircraft's OWN estimate of itself, which is all the law may read.
        self._estimate = (
            float(message.roll), float(message.pitch),
            float(getattr(message, "rollspeed", 0.0)),
            float(getattr(message, "pitchspeed", 0.0)),
            float(getattr(message, "yawspeed", 0.0)),
        )
        return self._emit(previous_clock)

    def _emit(self, previous_clock: float | None) -> TruthPose | None:
        if self._truth is None or previous_clock is None:
            return None
        if self._last_emitted_t_s == self._clock_t_s:
            return None
        skew = self._clock_t_s - previous_clock
        # Consumed either way: a truth sample belongs to the interval it landed
        # in, so carrying it forward into a later one would fake its freshness.
        if self._estimate is None:
            return None
        lat, lon, alt, roll, pitch, yaw = self._truth
        est_roll, est_pitch, est_roll_rate, est_pitch_rate, est_yaw_rate = (
            self._estimate
        )
        self._truth = None
        if skew > self._max_skew_s:
            # The pair is not one pose. Dropping it is correct: using it would
            # blend an attitude from one instant with a position from another,
            # which is the exact error this class exists to prevent.
            self._rejected_skew += 1
            return None
        if self._last_emitted_t_s is not None:
            self._gaps.append(self._clock_t_s - self._last_emitted_t_s)
        self._last_emitted_t_s = self._clock_t_s
        self._skews.append(skew)
        self._poses += 1
        return TruthPose(
            t_s=self._clock_t_s,
            # Already degrees, resolved by sim_state_coordinates_deg above.
            lat_deg=lat,
            lon_deg=lon,
            alt_m=alt,
            # Attitude IS radians on this message, unlike the coordinates.
            roll_deg=math.degrees(roll),
            pitch_deg=math.degrees(pitch),
            yaw_deg=math.degrees(yaw) % 360.0,
            skew_s=skew,
            est_roll_deg=math.degrees(est_roll),
            est_pitch_deg=math.degrees(est_pitch),
            est_roll_rate_rad_s=est_roll_rate,
            est_pitch_rate_rad_s=est_pitch_rate,
            est_yaw_rate_rad_s=est_yaw_rate,
        )

    @property
    def pose_count(self) -> int:
        return self._poses

    @property
    def observed_rate_hz(self) -> float | None:
        """Delivered pose rate in SIMULATED seconds, or None if unknown yet."""
        if not self._gaps:
            return None
        span = sum(self._gaps)
        return len(self._gaps) / span if span > 0 else None

    def certification_error(self) -> str | None:
        """Why this run's pose feed may not be trusted, or None if it may be.

        Returned as a reason rather than a bool so a failing run says what went
        wrong in its own record. A run that fails this did not measure the
        navigation law; it measured a starved link.
        """
        if self._poses < self._min_samples:
            return (
                f"insufficient pose cadence evidence: {self._poses} poses "
                f"(need {self._min_samples})"
            )
        worst = max(self._gaps) if self._gaps else math.inf
        if worst > self._max_gap_s:
            return (
                f"pose gap {worst:.3f}s exceeds {self._max_gap_s:.3f}s -- the "
                "stream was not delivered at the requested rate"
            )
        # A gap is only recorded when a pose is EMITTED, so a truth stream that
        # dies never records the gap that would convict it: the last gap stays
        # healthy and the run certifies while the aircraft flew on with no fresh
        # observation at all. The only witness to a trailing stall is the clock
        # continuing past the final pose, so it is measured against the last
        # ATTITUDE rather than against the last pose.
        if self._clock_t_s is not None and self._last_emitted_t_s is not None:
            trailing = self._clock_t_s - self._last_emitted_t_s
            if trailing > self._max_gap_s:
                return (
                    f"truth stream stall: {trailing:.3f}s of simulated time "
                    f"after the last pose exceeds {self._max_gap_s:.3f}s"
                )
        if self._rejected_skew > self._poses:
            return (
                f"{self._rejected_skew} pose pairs rejected for skew against "
                f"{self._poses} accepted -- the two streams are not aligned"
            )
        return None

    def summary(self) -> dict[str, float | int | None]:
        return {
            "poses": self._poses,
            "observed_rate_hz": (round(self.observed_rate_hz, 2)
                                 if self.observed_rate_hz else None),
            "worst_gap_s": round(max(self._gaps), 4) if self._gaps else None,
            "worst_skew_s": round(max(self._skews), 4) if self._skews else None,
            "trailing_stall_s": (
                round(self._clock_t_s - self._last_emitted_t_s, 4)
                if self._clock_t_s is not None
                and self._last_emitted_t_s is not None else None
            ),
            "mean_skew_s": (round(statistics.fmean(self._skews), 4)
                            if self._skews else None),
            "rejected_for_skew": self._rejected_skew,
            # "degE7" is the good path (1.1 cm quantisation). "float32" is the
            # degraded one (0.42 m). "mixed" means the link changed mid-run and
            # the position quality is not uniform across the scoring interval.
            "coordinate_source": (
                None if not self._position_samples
                else "degE7" if self._degE7_fields == self._position_samples
                else "float32" if self._degE7_fields == 0
                else f"mixed ({self._degE7_fields}/{self._position_samples} degE7)"
            ),
            # How long after a SIM_STATE the ATTITUDE that closes its interval
            # actually arrives. HOST time, diagnostic only. If this sits well
            # above one ATTITUDE period, the pose being handed out mixes a truth
            # sample with an attitude from materially later, and `worst_skew_s`
            # above -- derived from the autopilot clock -- understates it.
            "arrival_offset_s": (
                {
                    "median": round(statistics.median(self._arrival_offsets), 4),
                    "mean": round(statistics.fmean(self._arrival_offsets), 4),
                    "p95": round(sorted(self._arrival_offsets)[
                        min(int(len(self._arrival_offsets) * 0.95),
                            len(self._arrival_offsets) - 1)], 4),
                    "max": round(max(self._arrival_offsets), 4),
                    "samples": len(self._arrival_offsets),
                }
                if self._arrival_offsets else None
            ),
        }


def achievable_pose_rate_hz(
    read_param: ParameterReader, sysid: int,
) -> tuple[float, float | None]:
    """The fastest rate THIS aircraft will grant, read from the aircraft.

    ArduPilot caps MAVLink at 0.8 * SCHED_LOOP_RATE and rejects a faster
    request outright. SCHED_LOOP_RATE is not a universal constant: it is 50 on
    this SITL Plane, 200 by default on a Cube 6X, and can be set to 400. Asking
    for a number chosen up front therefore either gets refused on the slow
    aircraft or throws away most of the rate available on the fast one -- and
    pose skew, which is what bounds the whole measurement, scales directly with
    it. 50 Hz scheduler gives a 25 ms period; 200 Hz gives 6.25 ms.

    Returns the rate to request and the scheduler rate it came from, or the
    module fallback and None when the parameter cannot be read.
    """
    scheduler_hz = read_param(sysid, "SCHED_LOOP_RATE")
    try:
        scheduler_hz = float(scheduler_hz)
    except (TypeError, ValueError):
        return POSE_STREAM_RATE_HZ, None
    if not math.isfinite(scheduler_hz) or scheduler_hz <= 0.0:
        return POSE_STREAM_RATE_HZ, None
    return (
        scheduler_hz * ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION,
        scheduler_hz,
    )


def request_truth_pose_streams(
    master: MavlinkLink,
    sysid: int,
    *,
    rate_hz: float = TRUTH_POSE_RATE_HZ,
    timeout_s: float = 5.0,
) -> dict[str, str]:
    """Ask for SIM_STATE and ATTITUDE at `rate_hz`; report what each said.

    Both are required: SIM_STATE is the truth and ATTITUDE is the clock, and a
    pose needs both. Raising only one is the failure this returns evidence
    about -- a 50 Hz position paired with a 4 Hz attitude is not a 50 Hz pose.

    ADDRESSED BY SYSID, not by `master.target_system`. Nothing in this harness
    sets that attribute, so it stays 0 and the command goes out as a broadcast
    the aircraft never answered -- both streams read "refused" while the
    aircraft was healthy and had simply not been asked. Every other command
    here names the sysid (scratch_sitl_uav.py:276), and so does this one.

    The reply is EVIDENCE, not a gate. An accepted result does not mean the data
    arrives at that rate, and a missing acknowledgement does not mean it will
    not: the only thing that settles it is what actually showed up, which
    TruthPoseStream.certification_error() judges after the run. This is recorded
    so that a certification failure can be read back to its cause.
    """
    from pymavlink import mavutil

    interval_us = int(round(1_000_000.0 / rate_hz))
    wanted = {
        "SIM_STATE": mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
        "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    }
    return {
        name: _request_one(master, mavutil, sysid, message_id, interval_us,
                           timeout_s)
        for name, message_id in wanted.items()
    }


# MAV_RESULT names, so a refusal reads as itself in the result file instead of
# as an integer the reader has to look up.
_RESULTS = {
    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
    3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED",
}


def _request_one(master: MavlinkLink, mavutil: object, sysid: int,
                 message_id: int, interval_us: int,
                 timeout_s: float) -> str:
    import time

    master.mav.command_long_send(
        sysid, 0, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(message_id), float(interval_us), 0.0, 0.0, 0.0, 0.0, 0.0,
    )
    # Wall-clock only to bound the wait for a reply. It never reaches a
    # measurement: every pose is timestamped from the autopilot's clock.
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != sysid:
            continue
        if int(getattr(message, "command", -1)) != int(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        ):
            continue
        result = int(getattr(message, "result", -1))
        if result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            continue
        return _RESULTS.get(result, f"result {result}")
    return "no acknowledgement"
